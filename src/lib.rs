#![no_std]

//! This crate provides a ST7365P driver to connect to TFT displays.

pub mod instruction;
use crate::{
    fb::{MAX_META_TILES, TILE_SIZE},
    instruction::Instruction,
};
mod fb;
pub use fb::FrameBuffer;

use embedded_graphics_core::geometry::OriginDimensions;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use heapless::Vec;

const MAX_SCREEN_WIDTH: usize = 320;
const MAX_SCREEN_HEIGHT: usize = 480;

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
        let mut buffer = [0_u8; 2048];
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

    pub async fn set_vertical_scroll_definition(
        &mut self,
        tfa: u16,
        vsa: u16,
        bsa: u16,
    ) -> Result<(), ()> {
        let params = [
            (tfa >> 8) as u8,
            (tfa & 0xFF) as u8,
            (vsa >> 8) as u8,
            (vsa & 0xFF) as u8,
            (bsa >> 8) as u8,
            (bsa & 0xFF) as u8,
        ];
        self.write_command(Instruction::VSCRDEF, &params).await?;

        Ok(())
    }

    pub async fn set_vertical_scroll_start_address(&mut self, vsp: u16) -> Result<(), ()> {
        let params = [(vsp >> 8) as u8, (vsp & 0xFF) as u8];
        self.write_command(Instruction::VSCSAD, &params).await?;

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

    /// Sends the entire framebuffer to the display
    pub async fn draw<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize>(
        &mut self,
        fb: &mut FrameBuffer<WIDTH, HEIGHT, SIZE>,
    ) -> Result<(), ()> {
        self.set_pixels_buffered(
            0,
            0,
            fb.size().width as u16 - 1,
            fb.size().height as u16 - 1,
            &fb.buffer,
        )
        .await?;

        fb.dirty_tiles.fill(false);

        Ok(())
    }

    /// Sends only dirty tiles (16x16px) individually to the display without batching
    pub async fn partial_draw<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize>(
        &mut self,
        fb: &mut FrameBuffer<WIDTH, HEIGHT, SIZE>,
    ) -> Result<(), ()> {
        if fb.dirty_tiles.any() {
            let tiles_x = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
            let tiles_y = (HEIGHT + TILE_SIZE - 1) / TILE_SIZE;

            let mut tile_buffer = [0u16; TILE_SIZE * TILE_SIZE];

            for ty in 0..tiles_y {
                for tx in 0..tiles_x {
                    if !fb.dirty_tiles[ty * tiles_x + tx] {
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
                                tile_buffer[row * TILE_SIZE + col] = fb.buffer[idx];
                            } else {
                                // Out of bounds, fill with zero (or background)
                                tile_buffer[row * TILE_SIZE + col] = 0;
                            }
                        }
                    }

                    // Send the tile's pixel data to the display
                    self.set_pixels_buffered(
                        x as u16,
                        y as u16,
                        (x + TILE_SIZE - 1).min(WIDTH - 1) as u16,
                        (y + TILE_SIZE - 1).min(HEIGHT - 1) as u16,
                        &tile_buffer,
                    )
                    .await?;

                    // Mark tile as clean
                    fb.dirty_tiles.set(ty * tiles_x + tx, false);
                }
            }
        }

        Ok(())
    }

    /// Sends only dirty tiles (16x16px) in batches to the display
    pub async fn partial_draw_batched<
        const WIDTH: usize,
        const HEIGHT: usize,
        const SIZE: usize,
    >(
        &mut self,
        fb: &mut FrameBuffer<WIDTH, HEIGHT, SIZE>,
    ) -> Result<(), ()> {
        if fb.dirty_tiles.any() {
            let tiles_x = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
            let tiles_y = (HEIGHT + TILE_SIZE - 1) / TILE_SIZE;

            let meta_tiles = fb.find_meta_tiles(tiles_x, tiles_y);

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
                        .extend_from_slice(&fb.buffer[start..end])
                        .unwrap();
                }

                self.set_pixels_buffered(
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
                        fb.dirty_tiles.set(tile_idx, false);
                    }
                }
            }
        }

        Ok(())
    }
}
