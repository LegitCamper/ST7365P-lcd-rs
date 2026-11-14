use super::*;

/// Last successfully uploaded pixels for exact final-frame change detection.
///
/// Storage is supplied by the caller: no extra framebuffer is allocated by the
/// driver. Keep one state per framebuffer/display pair. Use only `draw_diff`
/// for uploads while this state is valid, and invalidate it after display reset,
/// orientation/offset changes, direct writes, or switching framebuffer instances.
/// Access to the framebuffer, this state, and the display must be serialized.
pub struct DisplayState<
    const WIDTH: usize,
    const HEIGHT: usize,
    const SIZE: usize,
    Storage = DefaultPixelStorage<SIZE>,
> {
    pixels: Storage,
    valid_tiles: BitArr!(for TILE_COUNT, in usize, Lsb0),
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage>
    DisplayState<WIDTH, HEIGHT, SIZE, Storage>
{
    /// Attaches history storage without assuming its contents match the panel.
    ///
    /// # Panics
    /// Panics for invalid framebuffer dimensions or a storage length other than SIZE.
    pub fn new(storage: Storage) -> Self {
        FrameBuffer::<WIDTH, HEIGHT, SIZE, Storage>::validate_dimensions();
        assert!(
            storage.len() == SIZE,
            "Display history storage length mismatch"
        );
        Self {
            pixels: storage,
            valid_tiles: bitarr!(usize, Lsb0; 0; TILE_COUNT),
        }
    }

    /// Requires the next diff draw to upload every visible tile again.
    pub fn invalidate(&mut self) {
        self.valid_tiles.fill(false);
    }
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize, Storage: PixelStorage>
    FrameBuffer<WIDTH, HEIGHT, SIZE, Storage>
{
    /// Flushes tiles whose final pixels differ from valid display history.
    ///
    /// Unlike per-write dirty tracking, this suppresses clear-then-redraw work
    /// when the final image matches the last successful upload. Every valid tile
    /// is compared, even when snapshot capture cleared its dirty flag. This costs
    /// up to one full-frame comparison per call, in addition to caller-owned
    /// history storage. Slice-backed storage uses contiguous row comparisons.
    /// Invalid history always forces an upload.
    ///
    /// Errors and cancellation invalidate every attempted tile, including tiles
    /// completed before a later failure. A retry conservatively uploads those
    /// tiles again. Only a completely successful flush updates history. This is
    /// unsynchronized batching; it does not guarantee tear-free presentation.
    pub async fn draw_diff<SPI, DC, RST, DELAY, History: PixelStorage>(
        &mut self,
        display: &mut ST7365P<SPI, DC, RST, DELAY>,
        history: &mut DisplayState<WIDTH, HEIGHT, SIZE, History>,
    ) -> Result<(), ()>
    where
        SPI: SpiDevice,
        DC: OutputPin,
        RST: OutputPin,
        DELAY: DelayNs,
    {
        for tile_y in 0..Self::TILES_Y {
            for tile_x in 0..Self::TILES_X {
                let tile_index = tile_y * Self::TILES_X + tile_x;
                if !history.valid_tiles[tile_index] {
                    self.dirty_tiles.set(tile_index, true);
                    continue;
                }
                let start_x = tile_x * TILE_SIZE;
                let start_y = tile_y * TILE_SIZE;
                let end_x = (start_x + TILE_SIZE).min(WIDTH);
                let slices = self.buffer.as_slice().zip(history.pixels.as_slice());
                let unchanged = (start_y..(start_y + TILE_SIZE).min(HEIGHT)).all(|row| {
                    let mut range = row * WIDTH + start_x..row * WIDTH + end_x;
                    match slices {
                        Some((pixels, previous)) => pixels[range.clone()] == previous[range],
                        None => {
                            range.all(|index| self.buffer.read(index) == history.pixels.read(index))
                        }
                    }
                });
                self.dirty_tiles.set(tile_index, !unchanged);
                if !unchanged {
                    history.valid_tiles.set(tile_index, false);
                }
            }
        }

        self.partial_draw_batched(display).await?;

        for tile_y in 0..Self::TILES_Y {
            for tile_x in 0..Self::TILES_X {
                let tile_index = tile_y * Self::TILES_X + tile_x;
                if history.valid_tiles[tile_index] {
                    continue;
                }
                let start_x = tile_x * TILE_SIZE;
                let start_y = tile_y * TILE_SIZE;
                for row in start_y..(start_y + TILE_SIZE).min(HEIGHT) {
                    for column in start_x..(start_x + TILE_SIZE).min(WIDTH) {
                        let index = row * WIDTH + column;
                        history.pixels.write(index, self.buffer.read(index));
                    }
                }
                history.valid_tiles.set(tile_index, true);
            }
        }
        Ok(())
    }
}
