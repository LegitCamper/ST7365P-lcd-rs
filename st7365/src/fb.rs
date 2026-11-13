use bitvec::{bitarr, order::Lsb0, BitArr};
use core::ops::Range;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

/// Owned pixel storage used by [`FrameBuffer::new`].
pub type DefaultPixelStorage<const SIZE: usize> = [u16; SIZE];

/// Pixel access independent of allocation and synchronization.
///
/// Words use the driver's buffered wire format: byte-swapped RGB565 on
/// little-endian targets. Length must remain constant while attached.
/// The framebuffer supplies in-bounds indices and ranges.
/// Mutations must go through the framebuffer to maintain dirty tracking.
/// Atomic storage alone does not make concurrent drawing/flushing supported.
pub trait PixelStorage {
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn read(&self, index: usize) -> u16;
    fn write(&mut self, index: usize, value: u16);

    fn fill_range(&mut self, range: Range<usize>, value: u16) {
        for index in range {
            self.write(index, value);
        }
    }

    fn copy_into(&self, start: usize, destination: &mut [u16]) {
        for (offset, pixel) in destination.iter_mut().enumerate() {
            *pixel = self.read(start + offset);
        }
    }

    /// Optional zero-copy fast path. Concurrently mutable storage must return None.
    fn as_slice(&self) -> Option<&[u16]> {
        None
    }
}

impl<Storage: AsRef<[u16]> + AsMut<[u16]>> PixelStorage for Storage {
    fn len(&self) -> usize {
        self.as_ref().len()
    }

    fn read(&self, index: usize) -> u16 {
        self.as_ref()[index]
    }

    fn write(&mut self, index: usize, value: u16) {
        self.as_mut()[index] = value;
    }

    fn fill_range(&mut self, range: Range<usize>, value: u16) {
        self.as_mut()[range].fill(value);
    }

    fn copy_into(&self, start: usize, destination: &mut [u16]) {
        destination.copy_from_slice(&self.as_ref()[start..start + destination.len()]);
    }

    fn as_slice(&self) -> Option<&[u16]> {
        Some(self.as_ref())
    }
}

pub(crate) const TILE_SIZE: usize = 16; // 16x16 tile
pub(crate) const TILE_COUNT: usize =
    (MAX_SCREEN_WIDTH / TILE_SIZE) * (MAX_SCREEN_HEIGHT / TILE_SIZE); // 600 tiles

// Group of tiles for batching
pub(crate) const MAX_META_TILES: usize = MAX_SCREEN_WIDTH / TILE_SIZE; // max number of meta tiles in buffer

/// Physical scan order expressed in framebuffer coordinates, after rotation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanDirection {
    /// Increasing framebuffer Y.
    TopToBottom,
    /// Decreasing framebuffer Y.
    BottomToTop,
    /// Increasing framebuffer X.
    LeftToRight,
    /// Decreasing framebuffer X.
    RightToLeft,
}

/// Measured panel geometry and transfer limits for [`FrameBuffer::draw_sync`].
///
/// No timing defaults are assumed. Recalibrate after changing orientation,
/// offsets, refresh rate, SPI speed, or executor load. The framebuffer must map
/// to one contiguous physical scan interval; wrapped hardware scrolling is not
/// supported by this mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanSyncConfig {
    /// Number of counter values per frame, including sync and porch lines.
    pub frame_scanlines: u16,
    /// GSCAN value for the first framebuffer row/column scanned in `direction`.
    /// Includes panel porches and the framebuffer's display offset.
    pub first_scanline: u16,
    /// Physical scan direction, not the pixel upload order.
    pub direction: ScanDirection,
    /// Nonzero worst-case duration, in scanline periods, from the accepted
    /// GSCAN sample to the last pixel of a 16x16 tile. Round up and include read
    /// latency, address commands, bus contention, executor delays, and margin.
    /// One additional line is reserved before the scan reaches the tile.
    pub write_budget_lines: u16,
    /// Total GSCAN read budget for the entire call, not per tile. Polls within
    /// each tile wait are spaced by at least 50 us. Not a wall-clock timeout.
    pub max_polls: u32,
}

/// Failure during a synchronized partial framebuffer update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrawSyncError {
    /// Geometry is invalid, address offsets overflow, or the transfer budget
    /// cannot fit outside the largest tile's physical scan interval.
    InvalidConfig,
    /// The controller returned a counter outside `0..frame_scanlines`.
    InvalidScanline(u16),
    /// The total polling budget was exhausted, including a stuck counter.
    Timeout,
    /// A scan read, pixel write, or data/command pin operation failed.
    Interface,
}

/// Simple macro to allow quick initialization without the comberson SIZE const
#[macro_export]
macro_rules! framebuffer {
    ($w:expr, $h:expr) => {
        FrameBuffer::<$w, $h, { $w * $h }>::new()
    };
}

/// provides a synchronous abstraction above the display transport
/// for embedded-graphics to write to, ensure you periodically flush the framebuffer
#[allow(dead_code)]
pub struct FrameBuffer<
    const WIDTH: usize,
    const HEIGHT: usize,
    const SIZE: usize,
    Storage = DefaultPixelStorage<SIZE>,
> {
    pub(crate) buffer: Storage,
    pub(crate) dirty_tiles: BitArr!(for TILE_COUNT, in usize, Lsb0),
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> FrameBuffer<WIDTH, HEIGHT, SIZE> {
    /// Constructs a new framebuffer.
    ///
    /// # Panics
    /// Panics if dimensions are zero, exceed `u16::MAX`, require more than
    /// 600 tiles, or if `SIZE != WIDTH * HEIGHT`.
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self::validate_dimensions();
        Self {
            buffer: [0_u16; SIZE],
            dirty_tiles: bitarr!(usize, Lsb0; 0;TILE_COUNT),
        }
    }
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage>
    FrameBuffer<WIDTH, HEIGHT, SIZE, Storage>
{
    const TILES_X: usize = WIDTH.div_ceil(TILE_SIZE);
    const TILES_Y: usize = HEIGHT.div_ceil(TILE_SIZE);

    const fn validate_dimensions() {
        assert!(WIDTH > 0, "Framebuffer WIDTH must be > 0");
        assert!(HEIGHT > 0, "Framebuffer HEIGHT must be > 0");
        assert!(WIDTH <= u16::MAX as usize && HEIGHT <= u16::MAX as usize);
        assert!(Self::TILES_X * Self::TILES_Y <= TILE_COUNT);
        assert!(
            SIZE == WIDTH * HEIGHT,
            "Framebuffer size mismatch: SIZE must equal WIDTH * HEIGHT",
        );
    }

    /// Attaches caller-provided storage, initially marking every visible tile dirty.
    ///
    /// # Panics
    /// Panics for invalid dimensions or a storage length other than SIZE.
    pub fn with_storage(storage: Storage) -> Self {
        Self::validate_dimensions();
        assert!(storage.len() == SIZE, "Framebuffer storage length mismatch");
        let mut framebuffer = Self {
            buffer: storage,
            dirty_tiles: bitarr!(usize, Lsb0; 0;TILE_COUNT),
        };
        framebuffer.dirty_tiles[..Self::TILES_X * Self::TILES_Y].fill(true);
        framebuffer
    }

    /// Full-width tile-aligned band containing all dirty pixels, clipped to HEIGHT.
    pub fn dirty_rows(&self) -> Option<Range<usize>> {
        let visible = &self.dirty_tiles[..Self::TILES_X * Self::TILES_Y];
        let first = visible.first_one()? / Self::TILES_X;
        let last = visible.last_one()? / Self::TILES_X;
        Some(first * TILE_SIZE..((last + 1) * TILE_SIZE).min(HEIGHT))
    }

    /// Copies dirty rows into a separate whole-screen buffer and clears their flags.
    ///
    /// The caller must serialize this operation with drawing, then may release its
    /// lock while uploading the snapshot. On upload failure, call `retry_rows`.
    /// Invalid destination lengths return Err without changing dirty state.
    pub fn take_dirty_rows(&mut self, destination: &mut [u16]) -> Result<Option<Range<usize>>, ()> {
        if destination.len() != SIZE {
            return Err(());
        }
        let Some(rows) = self.dirty_rows() else {
            return Ok(None);
        };
        let start = rows.start * WIDTH;
        self.buffer.copy_into(start, &mut destination[start..rows.end * WIDTH]);
        self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].fill(false);
        Ok(Some(rows))
    }

    /// Re-marks a failed snapshot without clearing pixels dirtied since capture.
    /// Invalid, empty, or non-tile-aligned bands return Err without mutation.
    pub fn retry_rows(&mut self, rows: Range<usize>) -> Result<(), ()> {
        if rows.start >= rows.end
            || rows.end > HEIGHT
            || rows.start % TILE_SIZE != 0
            || (rows.end != HEIGHT && rows.end % TILE_SIZE != 0)
        {
            return Err(());
        }
        self.dirty_tiles[rows.start / TILE_SIZE * Self::TILES_X
            ..rows.end.div_ceil(TILE_SIZE) * Self::TILES_X].fill(true);
        Ok(())
    }

    fn mark_tiles_dirty(&mut self, rect: Rectangle) {
        let start_tx = (rect.top_left.x as usize) / TILE_SIZE;
        let end_tx = ((rect.top_left.x + rect.size.width as i32 - 1) as usize) / TILE_SIZE;
        let start_ty = (rect.top_left.y as usize) / TILE_SIZE;
        let end_ty = ((rect.top_left.y + rect.size.height as i32 - 1) as usize) / TILE_SIZE;

        for ty in start_ty..=end_ty {
            self.dirty_tiles[ty * Self::TILES_X + start_tx..=ty * Self::TILES_X + end_tx]
                .fill(true);
        }
    }

    fn set_pixel(&mut self, x: usize, y: usize, color: u16) {
        let index = y * WIDTH + x;
        if self.buffer.read(index) != color {
            self.buffer.write(index, color);
            self.dirty_tiles
                .set((y / TILE_SIZE) * Self::TILES_X + x / TILE_SIZE, true);
        }
    }

    /// Sends the entire framebuffer to the display
    pub async fn draw<SPI, DC, RST, DELAY: DelayNs>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
    ) -> Result<(), ()>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
    {
        let Some(pixels) = self.buffer.as_slice() else {
            self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].fill(true);
            return self.partial_draw(display).await;
        };
        display
            .set_pixels_buffered(
                0,
                0,
                self.size().width as u16 - 1,
                self.size().height as u16 - 1,
                pixels,
            )
            .await?;

        self.dirty_tiles.fill(false);

        Ok(())
    }

    /// Sends only dirty tiles (16x16px) individually to the display without batching
    pub async fn partial_draw<SPI, DC, RST, DELAY: DelayNs>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
    ) -> Result<(), ()>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
    {
        if self.dirty_tiles.any() {
            let mut tile_buffer = [0u16; TILE_SIZE * TILE_SIZE];

            for ty in 0..Self::TILES_Y {
                for tx in 0..Self::TILES_X {
                    if !self.dirty_tiles[ty * Self::TILES_X + tx] {
                        continue;
                    }

                    let x = tx * TILE_SIZE;
                    let y = ty * TILE_SIZE;
                    let width = TILE_SIZE.min(WIDTH - x);
                    let height = TILE_SIZE.min(HEIGHT - y);

                    for row in 0..height {
                        let start = (y + row) * WIDTH + x;
                        self.buffer
                            .copy_into(start, &mut tile_buffer[row * width..(row + 1) * width]);
                    }

                    // Send the tile's pixel data to the display
                    display
                        .set_pixels_buffered(
                            x as u16,
                            y as u16,
                            (x + width - 1) as u16,
                            (y + height - 1) as u16,
                            &tile_buffer[..width * height],
                        )
                        .await?;

                    // Mark tile as clean
                    self.dirty_tiles.set(ty * Self::TILES_X + tx, false);
                }
            }
        }

        Ok(())
    }

    /// Flushes dirty tiles only when the physical scan leaves time to finish.
    ///
    /// Each tile gets a fresh moving GSCAN sample before writing. A tile is
    /// eligible before or after its scan interval only if the next visit is
    /// farther away than `write_budget_lines + 1`. Even an all-dirty framebuffer
    /// stays tiled; no unsynchronized full-frame fallback is used.
    ///
    /// Keep SPI at a read-safe clock (6 MHz or less) for this entire call. The
    /// driver does not change clocks. Packets contain at most 256 RGB565 pixels.
    /// Only completed tiles become clean; errors or cancellation leave the
    /// current and remaining tiles dirty for retry. An unchanged buffer does
    /// no I/O, even with an invalid configuration.
    ///
    /// Tear avoidance depends on accurate physical mapping and a conservative
    /// transfer budget. Unbounded async scheduling delays cannot be protected
    /// against. Tiles may appear in different frames: this is not an atomic
    /// frame swap. Apply an application-level timeout if the HAL can stall.
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
        if !self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].any() {
            return Ok(());
        }
        let scan_extent = match config.direction {
            ScanDirection::TopToBottom | ScanDirection::BottomToTop => HEIGHT,
            ScanDirection::LeftToRight | ScanDirection::RightToLeft => WIDTH,
        };
        let frame_lines = usize::from(config.frame_scanlines);
        let first_line = usize::from(config.first_scanline);
        let budget = usize::from(config.write_budget_lines);
        if first_line + scan_extent > frame_lines
            || budget == 0
            || budget + TILE_SIZE.min(scan_extent) + 1 >= frame_lines
            || WIDTH - 1 + usize::from(display.dx) > u16::MAX as usize
            || HEIGHT - 1 + usize::from(display.dy) > u16::MAX as usize
        {
            return Err(DrawSyncError::InvalidConfig);
        }

        // ponytail: fixed-size tiles at one read-safe clock; batch only with measured timing bounds.
        let mut tile_buffer = [0u16; TILE_SIZE * TILE_SIZE];
        let mut polls_left = config.max_polls;
        for tile_y in 0..Self::TILES_Y {
            for tile_x in 0..Self::TILES_X {
                let tile_index = tile_y * Self::TILES_X + tile_x;
                if !self.dirty_tiles[tile_index] {
                    continue;
                }
                if polls_left < 2 {
                    return Err(DrawSyncError::Timeout);
                }
                let rect_x = tile_x * TILE_SIZE;
                let rect_y = tile_y * TILE_SIZE;
                let rect_width = TILE_SIZE.min(WIDTH - rect_x);
                let rect_height = TILE_SIZE.min(HEIGHT - rect_y);
                for row in 0..rect_height {
                    let source = (rect_y + row) * WIDTH + rect_x;
                    self.buffer.copy_into(
                        source,
                        &mut tile_buffer[row * rect_width..(row + 1) * rect_width],
                    );
                }
                let (scan_start, scan_length) = match config.direction {
                    ScanDirection::TopToBottom => (rect_y, rect_height),
                    ScanDirection::BottomToTop => (HEIGHT - rect_y - rect_height, rect_height),
                    ScanDirection::LeftToRight => (rect_x, rect_width),
                    ScanDirection::RightToLeft => (WIDTH - rect_x - rect_width, rect_width),
                };
                let scan_start = first_line + scan_start;
                let scan_end = scan_start + scan_length;
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
                    let distance = (scan_start + frame_lines - current) % frame_lines;
                    if previous.is_some_and(|last| last != current)
                        && !(scan_start..scan_end).contains(&current)
                        && distance > budget + 1
                    {
                        break;
                    }
                    previous = Some(current);
                }
                display
                    .set_pixels_buffered(
                        rect_x as u16,
                        rect_y as u16,
                        (rect_x + rect_width - 1) as u16,
                        (rect_y + rect_height - 1) as u16,
                        &tile_buffer[..rect_width * rect_height],
                    )
                    .await
                    .map_err(|_| DrawSyncError::Interface)?;
                self.dirty_tiles.set(tile_index, false);
            }
        }
        Ok(())
    }

    /// Sends only dirty tiles (16x16px) in batches to the display
    pub async fn partial_draw_batched<SPI, DC, RST, DELAY>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
    ) -> Result<(), ()>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        let visible_tiles = &self.dirty_tiles[..Self::TILES_X * Self::TILES_Y];
        if !visible_tiles.any() {
            return Ok(());
        }
        if visible_tiles.count_ones() * 100 >= visible_tiles.len() * 80 {
            return self.draw(display).await;
        }

        let mut pixel_buffer = [0u16; MAX_META_TILES * TILE_SIZE * TILE_SIZE];

        for ty in 0..Self::TILES_Y {
            let mut tx = 0;
            while tx < Self::TILES_X {
                let start_tile = ty * Self::TILES_X + tx;
                if !self.dirty_tiles[start_tile] {
                    tx += 1;
                    continue;
                }

                let row_end = (ty + 1) * Self::TILES_X;
                let mut width_tiles = if tx == 0
                    && self.buffer.as_slice().is_some()
                    && self.dirty_tiles[start_tile..row_end].all()
                {
                    Self::TILES_X
                } else {
                    1
                };
                while tx + width_tiles < Self::TILES_X
                    && width_tiles < MAX_META_TILES
                    && self.dirty_tiles[start_tile + width_tiles]
                {
                    width_tiles += 1;
                }

                let rect_x = tx * TILE_SIZE;
                let rect_y = ty * TILE_SIZE;
                let rect_width = (width_tiles * TILE_SIZE).min(WIDTH - rect_x);
                let rect_height = TILE_SIZE.min(HEIGHT - rect_y);
                let contiguous = self
                    .buffer
                    .as_slice()
                    .filter(|_| rect_x == 0 && rect_width == WIDTH);
                let pixels = if let Some(buffer) = contiguous {
                    &buffer[rect_y * WIDTH..(rect_y + rect_height) * WIDTH]
                } else {
                    for row in 0..rect_height {
                        let start = (rect_y + row) * WIDTH + rect_x;
                        self.buffer.copy_into(
                            start,
                            &mut pixel_buffer[row * rect_width..(row + 1) * rect_width],
                        );
                    }
                    &pixel_buffer[..rect_width * rect_height]
                };

                display
                    .set_pixels_buffered(
                        rect_x as u16,
                        rect_y as u16,
                        (rect_x + rect_width - 1) as u16,
                        (rect_y + rect_height - 1) as u16,
                        pixels,
                    )
                    .await?;

                self.dirty_tiles[start_tile..start_tile + width_tiles].fill(false);
                tx += width_tiles;
            }
        }

        Ok(())
    }
}

#[cfg(feature = "graphics")]
extern crate embedded_graphics_core;
use crate::{MAX_SCREEN_HEIGHT, MAX_SCREEN_WIDTH, ST7365P};

#[cfg(feature = "graphics")]
use self::embedded_graphics_core::{
    draw_target::DrawTarget,
    pixelcolor::{
        raw::{RawData, RawU16},
        Rgb565,
    },
    prelude::*,
    primitives::Rectangle,
};

#[cfg(feature = "graphics")]
impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage> DrawTarget
    for FrameBuffer<WIDTH, HEIGHT, SIZE, Storage>
{
    type Error = ();
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x >= 0
                && coord.y >= 0
                && (coord.x as usize) < WIDTH
                && (coord.y as usize) < HEIGHT
            {
                self.set_pixel(
                    coord.x as usize,
                    coord.y as usize,
                    RawU16::from(color).into_inner().swap_bytes(),
                );
            }
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&Rectangle::new(Point::zero(), self.size()));

        if drawable_area.size.width == 0 || drawable_area.size.height == 0 {
            return Ok(());
        }

        let start_x = drawable_area.top_left.x as usize;
        let start_y = drawable_area.top_left.y as usize;
        let width = drawable_area.size.width as usize;
        let height = drawable_area.size.height as usize;
        let skipped_rows = (drawable_area.top_left.y as i64 - area.top_left.y as i64) as usize;
        let skipped_columns = (drawable_area.top_left.x as i64 - area.top_left.x as i64) as usize;
        let mut skip = skipped_rows
            .checked_mul(area.size.width as usize)
            .and_then(|pixels| pixels.checked_add(skipped_columns))
            .ok_or(())?;
        let mut colors = colors.into_iter();

        for y in start_y..start_y + height {
            if skip > 0 && colors.nth(skip - 1).is_none() {
                return Ok(());
            }
            for x in start_x..start_x + width {
                let Some(color) = colors.next() else {
                    return Ok(());
                };
                self.set_pixel(x, y, RawU16::from(color).into_inner().swap_bytes());
            }
            skip = area.size.width as usize - width;
        }

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let drawable_area = area.intersection(&Rectangle::new(Point::zero(), self.size()));
        if drawable_area.size.width == 0 || drawable_area.size.height == 0 {
            return Ok(());
        }

        let start_x = drawable_area.top_left.x as usize;
        let start_y = drawable_area.top_left.y as usize;
        let width = drawable_area.size.width as usize;
        let height = drawable_area.size.height as usize;
        let color = RawU16::from(color).into_inner().swap_bytes();

        for y in start_y..start_y + height {
            let start = y * WIDTH + start_x;
            self.buffer.fill_range(start..start + width, color);
        }
        self.mark_tiles_dirty(drawable_area);
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.buffer
            .fill_range(0..SIZE, RawU16::from(color).into_inner().swap_bytes());
        self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].fill(true);
        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage>
    OriginDimensions for FrameBuffer<WIDTH, HEIGHT, SIZE, Storage>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
