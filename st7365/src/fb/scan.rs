use super::*;

/// Measured limits for scan-aware rectangular packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanBatchConfig {
    /// Maximum adjacent tiles per packet, from 1 through 20.
    pub max_tiles: usize,
    /// Worst-case scanline periods from the accepted GSCAN sample through the
    /// final pixel of the largest allowed packet. Include clock switching,
    /// command overhead, bus contention, executor delays, and safety margin.
    /// Must be at least the single-tile budget in [`ScanSyncConfig`].
    pub write_budget_lines: u16,
}

#[derive(Clone, Copy)]
struct ScanPacket {
    tile_x: usize,
    tile_y: usize,
    tiles_wide: usize,
    tiles_high: usize,
    width: usize,
    height: usize,
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage>
    FrameBuffer<WIDTH, HEIGHT, SIZE, Storage>
{
    /// Flushes dirty tiles only when the physical scan leaves time to finish.
    ///
    /// Unsafe tiles do not block other safe dirty tiles. Every packet is staged
    /// before its final fresh moving GSCAN sample; only completed packets become
    /// clean. Errors and cancellation preserve unfinished work for retry.
    /// Reads need a read-safe clock. A transaction-aware SPI device may select
    /// a separate validated write clock; include switching in the timing budget.
    /// This is not an atomic frame swap. Unbounded executor/HAL delays still
    /// require application-level timeout and cannot be made tear-free here.
    pub async fn draw_sync<SPI, DC, RST, DELAY>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
        config: &ScanSyncConfig,
    ) -> Result<(), DrawSyncError>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        self.draw_sync_packets(
            display,
            config,
            1,
            config.write_budget_lines,
            &mut [0; TILE_SIZE * TILE_SIZE],
        )
        .await
    }

    /// Flushes bounded rectangles using a separately measured packet budget.
    ///
    /// Adjacent tiles merge perpendicular to physical scanning: horizontal
    /// strips for top/bottom scanning, vertical strips for left/right scanning.
    /// This preserves each packet's occupied scan interval while reducing reads
    /// and address commands. Scratch storage is bounded to 5,120 pixels; dense
    /// updates never fall back to an unsynchronized full-frame transfer.
    /// Recalibrate after changing `max_tiles`, clocks, orientation, or load.
    pub async fn draw_sync_batched<SPI, DC, RST, DELAY>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
        config: &ScanSyncConfig,
        batches: &ScanBatchConfig,
    ) -> Result<(), DrawSyncError>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        self.draw_sync_batched_with_scratch(
            display,
            config,
            batches,
            &mut [0; MAX_META_TILES * TILE_SIZE * TILE_SIZE],
        )
        .await
    }

    /// Scan-aware batching with reusable caller-owned scratch (5,120 words).
    /// Staging still finishes before the final fresh scan sample. Include all
    /// other timing overheads in the packet budget, as with `draw_sync_batched`.
    pub async fn draw_sync_batched_with_scratch<SPI, DC, RST, DELAY>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
        config: &ScanSyncConfig,
        batches: &ScanBatchConfig,
        pixels: &mut [u16; MAX_META_TILES * TILE_SIZE * TILE_SIZE],
    ) -> Result<(), DrawSyncError>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        if !self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].any() {
            return Ok(());
        }
        if batches.max_tiles == 0
            || batches.max_tiles > MAX_META_TILES
            || batches.write_budget_lines < config.write_budget_lines
        {
            return Err(DrawSyncError::InvalidConfig);
        }
        self.draw_sync_packets(
            display,
            config,
            batches.max_tiles,
            batches.write_budget_lines,
            pixels,
        )
        .await
    }

    fn scan_packet(&self, index: usize, direction: ScanDirection, max_tiles: usize) -> ScanPacket {
        let tile_x = index % Self::TILES_X;
        let tile_y = index / Self::TILES_X;
        let mut tiles_wide = 1;
        let mut tiles_high = 1;
        match direction {
            ScanDirection::TopToBottom | ScanDirection::BottomToTop => {
                while tiles_wide < max_tiles
                    && tile_x + tiles_wide < Self::TILES_X
                    && self.dirty_tiles[index + tiles_wide]
                {
                    tiles_wide += 1;
                }
            }
            ScanDirection::LeftToRight | ScanDirection::RightToLeft => {
                while tiles_high < max_tiles
                    && tile_y + tiles_high < Self::TILES_Y
                    && self.dirty_tiles[index + tiles_high * Self::TILES_X]
                {
                    tiles_high += 1;
                }
            }
        }
        ScanPacket {
            tile_x,
            tile_y,
            tiles_wide,
            tiles_high,
            width: (tiles_wide * TILE_SIZE).min(WIDTH - tile_x * TILE_SIZE),
            height: (tiles_high * TILE_SIZE).min(HEIGHT - tile_y * TILE_SIZE),
        }
    }

    fn scan_packet_safe(
        packet: ScanPacket,
        config: &ScanSyncConfig,
        budget: usize,
        current: usize,
    ) -> bool {
        let rect_x = packet.tile_x * TILE_SIZE;
        let rect_y = packet.tile_y * TILE_SIZE;
        let (start, length) = match config.direction {
            ScanDirection::TopToBottom => (rect_y, packet.height),
            ScanDirection::BottomToTop => (HEIGHT - rect_y - packet.height, packet.height),
            ScanDirection::LeftToRight => (rect_x, packet.width),
            ScanDirection::RightToLeft => (WIDTH - rect_x - packet.width, packet.width),
        };
        let start = usize::from(config.first_scanline) + start;
        let distance = (start + usize::from(config.frame_scanlines) - current)
            % usize::from(config.frame_scanlines);
        !(start..start + length).contains(&current) && distance > budget + 1
    }

    fn stage_scan_packet(&self, packet: ScanPacket, destination: &mut [u16]) {
        for row in 0..packet.height {
            let source = (packet.tile_y * TILE_SIZE + row) * WIDTH + packet.tile_x * TILE_SIZE;
            self.buffer.copy_into(
                source,
                &mut destination[row * packet.width..(row + 1) * packet.width],
            );
        }
    }

    async fn draw_sync_packets<SPI, DC, RST, DELAY>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
        config: &ScanSyncConfig,
        max_tiles: usize,
        write_budget_lines: u16,
        pixels: &mut [u16],
    ) -> Result<(), DrawSyncError>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        let visible_count = Self::TILES_X * Self::TILES_Y;
        if !self.dirty_tiles[..visible_count].any() {
            return Ok(());
        }
        let scan_extent = match config.direction {
            ScanDirection::TopToBottom | ScanDirection::BottomToTop => HEIGHT,
            ScanDirection::LeftToRight | ScanDirection::RightToLeft => WIDTH,
        };
        let frame_lines = usize::from(config.frame_scanlines);
        let budget = usize::from(write_budget_lines);
        if usize::from(config.first_scanline) + scan_extent > frame_lines
            || config.write_budget_lines == 0
            || budget == 0
            || budget + TILE_SIZE.min(scan_extent) + 1 >= frame_lines
            || WIDTH - 1 + usize::from(display.dx) > u16::MAX as usize
            || HEIGHT - 1 + usize::from(display.dy) > u16::MAX as usize
        {
            return Err(DrawSyncError::InvalidConfig);
        }

        let mut polls_left = config.max_polls;
        while let Some(index) = self.dirty_tiles[..visible_count].first_one() {
            if polls_left < 2 {
                return Err(DrawSyncError::Timeout);
            }
            let mut packet = self.scan_packet(index, config.direction, max_tiles);
            self.stage_scan_packet(packet, pixels);
            let mut previous = None;
            loop {
                if polls_left == 0 {
                    return Err(DrawSyncError::Timeout);
                }
                if previous.is_some() {
                    display.delay.delay_us(50).await;
                }
                let current = display
                    .read_scanline()
                    .await
                    .map_err(|_| DrawSyncError::Interface)?;
                polls_left -= 1;
                if current >= config.frame_scanlines {
                    return Err(DrawSyncError::InvalidScanline(current));
                }
                let current = usize::from(current);
                let safe = Self::scan_packet_safe(packet, config, budget, current);
                if safe && previous.is_some_and(|last| last != current) {
                    break;
                }
                if !safe {
                    if let Some(candidate) = self.dirty_tiles[..visible_count]
                        .iter_ones()
                        .map(|index| self.scan_packet(index, config.direction, max_tiles))
                        .find(|candidate| {
                            Self::scan_packet_safe(*candidate, config, budget, current)
                        })
                    {
                        packet = candidate;
                        self.stage_scan_packet(packet, pixels);
                    }
                }
                previous = Some(current);
            }
            let rect_x = packet.tile_x * TILE_SIZE;
            let rect_y = packet.tile_y * TILE_SIZE;
            display
                .set_pixels_buffered(
                    rect_x as u16,
                    rect_y as u16,
                    (rect_x + packet.width - 1) as u16,
                    (rect_y + packet.height - 1) as u16,
                    &pixels[..packet.width * packet.height],
                )
                .await
                .map_err(|_| DrawSyncError::Interface)?;
            for tile_y in packet.tile_y..packet.tile_y + packet.tiles_high {
                let start = tile_y * Self::TILES_X + packet.tile_x;
                self.dirty_tiles[start..start + packet.tiles_wide].fill(false);
            }
        }
        Ok(())
    }
}
