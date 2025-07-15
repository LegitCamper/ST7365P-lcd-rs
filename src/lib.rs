#![no_std]

//! This crate provides a ST7365P driver to connect to TFT displays.

pub mod instruction;
use crate::instruction::Instruction;

use bitvec::{bitarr, order::Lsb0, BitArr};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use heapless::Vec;

const MAX_SCREEN_WIDTH: usize = 320;
const MAX_SCREEN_HIEGHT: usize = 480;

/// ST7365P driver to connect to TFT displays.
pub struct ST7365P<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// SPI
    spi: SPI,

    /// Data/command pin.
    dc: DC,

    /// Reset pin.
    rst: Option<RST>,

    /// Whether the display is RGB (true) or BGR (false)
    rgb: bool,

    /// Whether the colours are inverted (true) or not (false)
    inverted: bool,

    /// Global image offset
    dx: u16,
    dy: u16,

    /// Delay
    delay: DELAY,
}

/// Display orientation.
#[derive(Clone, Copy)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandscapeSwapped = 0xA0,
}

impl<SPI, DC, RST, DELAY> ST7365P<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// Creates a new driver instance that uses hardware SPI.
    pub fn new(
        spi: SPI,
        dc: DC,
        rst: Option<RST>,
        rgb: bool,
        inverted: bool,
        delay: DELAY,
    ) -> Self {
        ST7365P {
            spi,
            dc,
            rst,
            rgb,
            inverted,
            dx: 0,
            dy: 0,
            delay,
        }
    }

    /// Runs commands to initialize the display.
    pub async fn init(&mut self) -> Result<(), ()> {
        self.hard_reset().await?;
        self.write_command(Instruction::SWRESET, &[]).await?;
        self.delay.delay_ms(200).await;
        self.write_command(Instruction::SLPOUT, &[]).await?;
        self.delay.delay_ms(200).await;

        self.write_command(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])
            .await?;
        if self.inverted {
            self.write_command(Instruction::INVON, &[]).await?;
        } else {
            self.write_command(Instruction::INVOFF, &[]).await?;
        }
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[0x00]).await?;
        } else {
            self.write_command(Instruction::MADCTL, &[0x08]).await?;
        }
        self.write_command(Instruction::COLMOD, &[0x05]).await?;

        Ok(())
    }

    /// Turns display on after init
    pub async fn set_on(&mut self) -> Result<(), ()>
    where
        DELAY: DelayNs,
    {
        self.write_command(Instruction::DISPON, &[]).await?;
        self.delay.delay_ms(200).await;
        Ok(())
    }

    pub async fn hard_reset(&mut self) -> Result<(), ()>
    where
        DELAY: DelayNs,
    {
        if let Some(rst) = &mut self.rst {
            rst.set_high().map_err(|_| ())?;
            self.delay.delay_ms(10).await;
            rst.set_low().map_err(|_| ())?;
            self.delay.delay_ms(10).await;
            rst.set_high().map_err(|_| ())?;
        }
        Ok(())
    }

    async fn write_command(&mut self, command: Instruction, params: &[u8]) -> Result<(), ()> {
        // delay amount empirically determined
        self.delay.delay_ns(1).await;
        self.dc.set_low().map_err(|_| ())?;
        self.spi.write(&[command as u8]).await.map_err(|_| ())?;
        if !params.is_empty() {
            self.start_data()?;
            self.write_data(params).await?;
        }
        Ok(())
    }

    fn start_data(&mut self) -> Result<(), ()> {
        self.dc.set_high().map_err(|_| ())
    }

    async fn write_data(&mut self, data: &[u8]) -> Result<(), ()> {
        // delay amount empirically determined
        self.delay.delay_ns(1).await;
        self.spi.write(data).await.map_err(|_| ())
    }

    /// Writes a data word to the display.
    async fn write_word(&mut self, value: u16) -> Result<(), ()> {
        self.write_data(&value.to_be_bytes()).await
    }

    async fn write_words_buffered<'a>(
        &mut self,
        words: impl IntoIterator<Item = &'a u16>,
    ) -> Result<(), ()> {
        let mut buffer = [0; 32];
        let mut index = 0;
        for word in words {
            let as_bytes = word.to_be_bytes();
            buffer[index] = as_bytes[0];
            buffer[index + 1] = as_bytes[1];
            index += 2;
            if index >= buffer.len() {
                self.write_data(&buffer).await?;
                index = 0;
            }
        }
        self.write_data(&buffer[0..index]).await
    }

    /// ensure you are only setting the top 3 bits for MADCTL (x,y, and x+y)
    pub async fn set_custom_orientation(&mut self, mut madctl: u8) -> Result<(), ()> {
        if !self.rgb {
            madctl |= 0x08
        }
        self.write_command(Instruction::MADCTL, &[madctl]).await?;

        Ok(())
    }

    pub async fn set_orientation(&mut self, orientation: &Orientation) -> Result<(), ()> {
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[*orientation as u8])
                .await?;
        } else {
            self.write_command(Instruction::MADCTL, &[*orientation as u8 | 0x08])
                .await?;
        }
        Ok(())
    }

    /// Sets the global offset of the displayed image
    pub fn set_offset(&mut self, dx: u16, dy: u16) {
        self.dx = dx;
        self.dy = dy;
    }

    /// Sets the address window for the display.
    pub async fn set_address_window(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
    ) -> Result<(), ()> {
        self.write_command(Instruction::CASET, &[]).await?;
        self.start_data()?;
        self.write_word(sx + self.dx).await?;
        self.write_word(ex + self.dx).await?;
        self.write_command(Instruction::RASET, &[]).await?;
        self.start_data()?;
        self.write_word(sy + self.dy).await?;
        self.write_word(ey + self.dy).await
    }

    /// Sets a pixel color at the given coords.
    pub async fn set_pixel(&mut self, x: u16, y: u16, color: u16) -> Result<(), ()> {
        self.set_address_window(x, y, x, y).await?;
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        self.write_word(color).await
    }

    /// Writes pixel colors sequentially into the current drawing window
    pub async fn write_pixels<P: IntoIterator<Item = u16>>(&mut self, colors: P) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        for color in colors {
            self.write_word(color).await?;
        }
        Ok(())
    }
    pub async fn write_pixels_buffered<'a, P: IntoIterator<Item = &'a u16>>(
        &mut self,
        colors: P,
    ) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data()?;
        self.write_words_buffered(colors).await
    }

    /// Sets pixel colors at the given drawing window
    pub async fn set_pixels<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey).await?;
        self.write_pixels(colors).await
    }

    pub async fn set_pixels_buffered<'a, P: IntoIterator<Item = &'a u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey).await?;
        self.write_pixels_buffered(colors).await
    }

    /// Allows adjusting gamma correction on the display.
    ///
    /// Takes in an array `pos` for positive polarity correction and an array `neg` for negative polarity correction.
    ///
    pub async fn adjust_gamma(&mut self, pos: &[u8; 16], neg: &[u8; 16]) -> Result<(), ()> {
        self.write_command(Instruction::PGC, pos).await?;
        self.write_command(Instruction::NGC, neg).await
    }
}

const TILE_SIZE: usize = 16; // 16x16 tile
const TILE_COUNT: usize = (MAX_SCREEN_WIDTH / TILE_SIZE) * (MAX_SCREEN_HIEGHT / TILE_SIZE); // 600 tiles

// Group of tiles for batching
const MAX_META_TILES: usize = MAX_SCREEN_WIDTH / TILE_SIZE; // max number of meta tiles in buffer
type MetaTileVec = Vec<Rectangle, { TILE_COUNT / MAX_META_TILES }>;

/// provides a synchronous abstraction above the display transport
/// for embedded-graphics to write to, ensure you periodically push the framebuffer
pub struct FrameBuffer<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> {
    buffer: [u16; SIZE],
    dirty_tiles: BitArr!(for TILE_COUNT, in usize, Lsb0),
}

impl<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> FrameBuffer<WIDTH, HEIGHT, SIZE> {
    /// Constructs a new framebuffer.
    ///
    /// # Panics
    /// Panics if `SIZE != WIDTH * HEIGHT`. This contract is not enforced
    /// by the Rust compiler on stable and must be upheld by the caller.
    pub fn new() -> Self {
        assert!(WIDTH > 0, "Framebuffer WIDTH must be > 0");
        assert!(HEIGHT > 0, "Framebuffer HEIGHT must be > 0");
        assert_eq!(
            SIZE,
            WIDTH * HEIGHT,
            "Framebuffer size mismatch: SIZE must equal WIDTH * HEIGHT ({} * {})",
            WIDTH,
            HEIGHT
        );
        Self {
            buffer: [0_u16; SIZE],
            dirty_tiles: bitarr!(usize, Lsb0; 0;TILE_COUNT),
        }
    }

    fn bounding_rect<I>(pixels: I) -> Option<Rectangle>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        use core::cmp::{max, min};
        let mut min_x = usize::MAX;
        let mut max_x = 0;
        let mut min_y = usize::MAX;
        let mut max_y = 0;

        let mut any_pixel = false;

        for Pixel(coord, _) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                let x = coord.x as usize;
                let y = coord.y as usize;
                min_x = min(min_x, x);
                max_x = max(max_x, x);
                min_y = min(min_y, y);
                max_y = max(max_y, y);
                any_pixel = true;
            }
        }

        if any_pixel {
            Some(Rectangle::new(
                Point::new(min_x as i32, min_y as i32),
                Size::new((max_x - min_x + 1) as u32, (max_y - min_y + 1) as u32),
            ))
        } else {
            None
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
                let mut height_tiles = 1;

                // Grow horizontally, but keep under MAX_TILES_PER_METATILE
                while tx + width_tiles < tiles_x
                    && self.dirty_tiles[ty * tiles_x + tx + width_tiles]
                    && (width_tiles + height_tiles) <= MAX_META_TILES
                {
                    width_tiles += 1;
                }

                // TODO: for simplicity, skiped vertical growth

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

    /// Sends only dirty tiles (16x16px) individually to the display
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
        if sx >= self.size().width as u16 - 1
            || ex >= self.size().width as u16 - 1
            || sy >= self.size().height as u16 - 1
            || ey >= self.size().height as u16 - 1
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
}

#[cfg(feature = "graphics")]
extern crate embedded_graphics_core;
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
            self.set_pixel(
                coord.x as u16,
                coord.y as u16,
                RawU16::from(color).into_inner(),
            )?;
        }

        self.mark_tiles_dirty(self.bounding_box());

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
                                RawU16::from(color).into_inner(),
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

            self.mark_tiles_dirty(drawable_area);
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.set_pixels_buffered(
            0,
            0,
            self.size().width as u16 - 1,
            self.size().height as u16 - 1,
            core::iter::repeat(RawU16::from(color).into_inner())
                .take((self.size().width * self.size().height) as usize),
        )?;

        self.dirty_tiles.fill(false);
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
