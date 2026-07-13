use bitvec::{BitArr, bitarr, order::Lsb0};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use heapless::Vec;

pub(crate) const TILE_SIZE: usize = 16; // 16x16 tile
pub(crate) const TILE_COUNT: usize =
    (MAX_SCREEN_WIDTH / TILE_SIZE) * (MAX_SCREEN_HEIGHT / TILE_SIZE); // 600 tiles

// Group of tiles for batching
pub(crate) const MAX_META_TILES: usize = MAX_SCREEN_WIDTH / TILE_SIZE; // max number of meta tiles in buffer
type MetaTileVec = Vec<Rectangle, { TILE_COUNT / MAX_META_TILES }>;

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
    /// Constructs a new framebuffer.
    ///
    /// # Panics
    /// Panics if `SIZE != WIDTH * HEIGHT`. This contract is not enforced
    /// by the Rust compiler on stable and must be upheld by the caller.
    #[allow(dead_code)]
    pub const fn new() -> Self {
        assert!(WIDTH > 0, "Framebuffer WIDTH must be > 0");
        assert!(HEIGHT > 0, "Framebuffer HEIGHT must be > 0");
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
        let tiles_x = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
        let start_tx = (rect.top_left.x as usize) / TILE_SIZE;
        let end_tx = ((rect.top_left.x + rect.size.width as i32 - 1) as usize) / TILE_SIZE;
        let start_ty = (rect.top_left.y as usize) / TILE_SIZE;
        let end_ty = ((rect.top_left.y + rect.size.height as i32 - 1) as usize) / TILE_SIZE;

        for ty in start_ty..=end_ty {
            for tx in start_tx..=end_tx {
                let tile_idx = ty * tiles_x + tx;
                self.dirty_tiles.set(tile_idx, true);
            }
        }
    }

    fn set_pixel(&mut self, x: u16, y: u16, color: u16) -> Result<(), ()> {
        self.buffer[(y as usize * WIDTH) + x as usize] = color;

        Ok(())
    }

    fn set_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        if sx >= self.size().width as u16
            || ex >= self.size().width as u16
            || sy >= self.size().height as u16
            || ey >= self.size().height as u16
        {
            return Err(()); // Bounds check
        }

        let mut color_iter = colors.into_iter();

        for y in sy..=ey {
            for x in sx..=ex {
                if let Some(color) = color_iter.next() {
                    self.buffer[(y as usize * WIDTH) + x as usize] = color;
                } else {
                    return Err(()); // Not enough data
                }
            }
        }

        // Optional: check that we consumed *exactly* the right amount
        if color_iter.next().is_some() {
            return Err(()); // Too much data
        }

        Ok(())
    }

    // walk the dirty tiles and mark groups of tiles(meta-tiles) for batched updates
    fn find_meta_tiles(&mut self, tiles_x: usize, tiles_y: usize) -> MetaTileVec {
        let mut meta_tiles: MetaTileVec = Vec::new();

        for ty in 0..tiles_y {
            let mut tx = 0;
            while tx < tiles_x {
                let idx = ty * tiles_x + tx;
                if !self.dirty_tiles[idx] {
                    tx += 1;
                    continue;
                }

                // Start meta-tile at this tile
                let mut width_tiles = 1;
                let height_tiles = 1;

                // Grow horizontally, but keep under MAX_TILES_PER_METATILE
                while tx + width_tiles < tiles_x
                    && self.dirty_tiles[ty * tiles_x + tx + width_tiles]
                    && (width_tiles + height_tiles) <= MAX_META_TILES
                {
                    width_tiles += 1;
                }

                // TODO: for simplicity, skipped vertical growth

                for x_off in 0..width_tiles {
                    self.dirty_tiles.set(ty * tiles_x + tx + x_off, false);
                }

                // new meta-tile pos
                let rect = Rectangle::new(
                    Point::new((tx * TILE_SIZE) as i32, (ty * TILE_SIZE) as i32),
                    Size::new(
                        (width_tiles * TILE_SIZE) as u32,
                        (height_tiles * TILE_SIZE) as u32,
                    ),
                );

                if meta_tiles.push(rect).is_err() {
                    return meta_tiles;
                };

                tx += width_tiles;
            }
        }

        meta_tiles
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
            let tiles_x = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
            let tiles_y = (HEIGHT + TILE_SIZE - 1) / TILE_SIZE;

            let mut tile_buffer = [0u16; TILE_SIZE * TILE_SIZE];

            for ty in 0..tiles_y {
                for tx in 0..tiles_x {
                    if !self.dirty_tiles[ty * tiles_x + tx] {
                        continue;
                    }

                    let x = tx * TILE_SIZE;
                    let y = ty * TILE_SIZE;

                    // Copy pixels for the tile into tile_buffer
                    for row in 0..TILE_SIZE {
                        for col in 0..TILE_SIZE {
                            let actual_x = x + col;
                            let actual_y = y + row;

                            if actual_x < WIDTH && actual_y < HEIGHT {
                                let idx = actual_y * WIDTH + actual_x;
                                tile_buffer[row * TILE_SIZE + col] = self.buffer[idx];
                            } else {
                                // Out of bounds, fill with zero (or background)
                                tile_buffer[row * TILE_SIZE + col] = 0;
                            }
                        }
                    }

                    // Send the tile's pixel data to the display
                    display
                        .set_pixels_buffered(
                            x as u16,
                            y as u16,
                            (x + TILE_SIZE - 1).min(WIDTH - 1) as u16,
                            (y + TILE_SIZE - 1).min(HEIGHT - 1) as u16,
                            &tile_buffer,
                        )
                        .await?;

                    // Mark tile as clean
                    self.dirty_tiles.set(ty * tiles_x + tx, false);
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
        if self.dirty_tiles.any() {
            let tiles_x = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
            let tiles_y = (HEIGHT + TILE_SIZE - 1) / TILE_SIZE;

            let meta_tiles = self.find_meta_tiles(tiles_x, tiles_y);

            // buffer for copying meta tiles before sending to display
            let mut pixel_buffer: Vec<u16, { MAX_META_TILES * TILE_SIZE * TILE_SIZE }> = Vec::new();

            for rect in meta_tiles {
                let rect_width = rect.size.width as usize;
                let rect_height = rect.size.height as usize;
                let rect_x = rect.top_left.x as usize;
                let rect_y = rect.top_left.y as usize;

                pixel_buffer.clear();

                for row in 0..rect_height {
                    let y = rect_y + row;
                    let start = y * WIDTH + rect_x;
                    let end = start + rect_width;

                    // Safe: we guarantee buffer will not exceed MAX_META_TILE_PIXELS
                    pixel_buffer
                        .extend_from_slice(&self.buffer[start..end])
                        .unwrap();
                }

                display
                    .set_pixels_buffered(
                        rect_x as u16,
                        rect_y as u16,
                        (rect_x + rect_width - 1) as u16,
                        (rect_y + rect_height - 1) as u16,
                        &pixel_buffer,
                    )
                    .await?;

                // walk the meta-tile and set as clean
                let start_tx = rect_x / TILE_SIZE;
                let start_ty = rect_y / TILE_SIZE;
                let end_tx = (rect_x + rect_width - 1) / TILE_SIZE;
                let end_ty = (rect_y + rect_height - 1) / TILE_SIZE;

                for ty in start_ty..=end_ty {
                    for tx in start_tx..=end_tx {
                        let tile_idx = ty * tiles_x + tx;
                        self.dirty_tiles.set(tile_idx, false);
                    }
                }
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
        Rgb565,
        raw::{RawData, RawU16},
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
        let mut dirty_rect: Option<Rectangle> = None;

        for Pixel(coord, color) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                let x = coord.x as i32;
                let y = coord.y as i32;

                if (x as usize) < WIDTH && (y as usize) < HEIGHT {
                    self.buffer[(y as usize) * WIDTH + (x as usize)] =
                        RawU16::from(color).into_inner().swap_bytes();

                    if let Some(ref mut rect) = dirty_rect {
                        rect.top_left.x = rect.top_left.x.min(x);
                        rect.top_left.y = rect.top_left.y.min(y);
                        let max_x = (rect.top_left.x + rect.size.width as i32 - 1).max(x);
                        let max_y = (rect.top_left.y + rect.size.height as i32 - 1).max(y);
                        rect.size.width = (max_x - rect.top_left.x + 1) as u32;
                        rect.size.height = (max_y - rect.top_left.y + 1) as u32;
                    } else {
                        dirty_rect = Some(Rectangle::new(Point::new(x, y), Size::new(1, 1)));
                    }
                }
            }
        }

        if let Some(rect) = dirty_rect {
            self.mark_tiles_dirty(rect);
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&Rectangle::new(Point::zero(), self.size()));

        if drawable_area.size != Size::zero() {
            // We assume that `colors` iterator is in row-major order for the original `area`
            // So we must skip rows/pixels that are clipped
            let area_width = area.size.width;
            let area_height = area.size.height;
            let mut colors = colors.into_iter();

            for y in 0..area_height {
                for x in 0..area_width {
                    let p = area.top_left + Point::new(x as i32, y as i32);

                    if drawable_area.contains(p) {
                        if let Some(color) = colors.next() {
                            self.set_pixel(
                                p.x as u16,
                                p.y as u16,
                                RawU16::from(color).into_inner().swap_bytes(),
                            )?;
                        } else {
                            break;
                        }
                    } else {
                        // Still need to consume the color even if not used!
                        let _ = colors.next();
                    }
                }
            }

            self.mark_tiles_dirty(*area);
        }

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(
            area,
            core::iter::repeat(color).take((self.size().width * self.size().height) as usize),
        )
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.set_pixels_buffered(
            0,
            0,
            self.size().width as u16 - 1,
            self.size().height as u16 - 1,
            core::iter::repeat(RawU16::from(color).into_inner().swap_bytes())
                .take((self.size().width * self.size().height) as usize),
        )?;

        self.dirty_tiles.fill(true);
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
