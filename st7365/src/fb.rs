use bitvec::{bitarr, order::Lsb0, BitArr};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use heapless::Vec;

pub(crate) const TILE_SIZE: usize = 16; // 16x16 tile
pub(crate) const TILE_COUNT: usize =
    (MAX_SCREEN_WIDTH / TILE_SIZE) * (MAX_SCREEN_HEIGHT / TILE_SIZE); // 600 tiles

// Group of tiles for batching
pub(crate) const MAX_META_TILES: usize = MAX_SCREEN_WIDTH / TILE_SIZE; // max number of meta tiles in buffer

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
pub struct FrameBuffer<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> {
    pub(crate) buffer: [u16; SIZE],
    pub(crate) dirty_tiles: BitArr!(for TILE_COUNT, in usize, Lsb0),
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> FrameBuffer<WIDTH, HEIGHT, SIZE> {
    const TILES_X: usize = WIDTH.div_ceil(TILE_SIZE);
    const TILES_Y: usize = HEIGHT.div_ceil(TILE_SIZE);

    /// Constructs a new framebuffer.
    ///
    /// # Panics
    /// Panics if dimensions are zero, exceed `u16::MAX`, require more than
    /// 600 tiles, or if `SIZE != WIDTH * HEIGHT`.
    #[allow(dead_code)]
    pub const fn new() -> Self {
        assert!(WIDTH > 0, "Framebuffer WIDTH must be > 0");
        assert!(HEIGHT > 0, "Framebuffer HEIGHT must be > 0");
        assert!(WIDTH <= u16::MAX as usize && HEIGHT <= u16::MAX as usize);
        assert!(Self::TILES_X * Self::TILES_Y <= TILE_COUNT);
        assert!(
            SIZE == WIDTH * HEIGHT,
            "Framebuffer size mismatch: SIZE must equal WIDTH * HEIGHT",
        );
        Self {
            buffer: [0_u16; SIZE],
            dirty_tiles: bitarr!(usize, Lsb0; 0;TILE_COUNT),
        }
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
        let pixel = &mut self.buffer[y * WIDTH + x];
        if *pixel != color {
            *pixel = color;
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
        display
            .set_pixels_buffered(
                0,
                0,
                self.size().width as u16 - 1,
                self.size().height as u16 - 1,
                &self.buffer,
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
                        tile_buffer[row * width..(row + 1) * width]
                            .copy_from_slice(&self.buffer[start..start + width]);
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
        // ponytail: only all-dirty frames bypass tiling; tune thresholds with hardware timings.
        if visible_tiles.all() {
            return self.draw(display).await;
        }

        let mut pixel_buffer: Vec<u16, { MAX_META_TILES * TILE_SIZE * TILE_SIZE }> = Vec::new();

        for ty in 0..Self::TILES_Y {
            let mut tx = 0;
            while tx < Self::TILES_X {
                let start_tile = ty * Self::TILES_X + tx;
                if !self.dirty_tiles[start_tile] {
                    tx += 1;
                    continue;
                }

                let row_end = (ty + 1) * Self::TILES_X;
                let mut width_tiles = if tx == 0 && self.dirty_tiles[start_tile..row_end].all() {
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
                let pixels = if rect_x == 0 && rect_width == WIDTH {
                    &self.buffer[rect_y * WIDTH..(rect_y + rect_height) * WIDTH]
                } else {
                    pixel_buffer.clear();
                    for row in 0..rect_height {
                        let start = (rect_y + row) * WIDTH + rect_x;
                        pixel_buffer
                            .extend_from_slice(&self.buffer[start..start + rect_width])
                            .map_err(|_| ())?;
                    }
                    pixel_buffer.as_slice()
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
impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> DrawTarget
    for FrameBuffer<WIDTH, HEIGHT, SIZE>
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
            self.buffer[start..start + width].fill(color);
        }
        self.mark_tiles_dirty(drawable_area);
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.buffer
            .fill(RawU16::from(color).into_inner().swap_bytes());
        self.dirty_tiles[..Self::TILES_X * Self::TILES_Y].fill(true);
        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> OriginDimensions
    for FrameBuffer<WIDTH, HEIGHT, SIZE>
{
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
