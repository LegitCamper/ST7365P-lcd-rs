#![no_std]

//! This crate provides a ST7365P driver to connect to TFT displays.

pub mod instruction;
use crate::instruction::Instruction;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

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

/// provides a synchronous abstraction above the display transport
/// for embedded-graphics to write to, ensure you periodically push the framebuffer
pub struct FrameBuffer<const WIDTH: usize, const HEIGHT: usize, const SIZE: usize> {
    buffer: [u16; SIZE],
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
        }
    }

    /// needs to be called to send framebuffer to underlying display asynchronously
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
        Ok(())
    }

    fn set_pixel(&mut self, x: u16, y: u16, color: u16) -> Result<(), ()> {
        if x > self.size().width as u16 - 1 || y > self.size().height as u16 - 1 {
            return Err(());
        }

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
        for Pixel(coord, color) in pixels.into_iter() {
            // Only draw pixels that would be on screen
            if coord.x >= 0
                && coord.y >= 0
                && coord.x < self.size().width as i32
                && coord.y < self.size().height as i32
            {
                self.set_pixel(
                    coord.x as u16,
                    coord.y as u16,
                    RawU16::from(color).into_inner(),
                )?;
            }
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        // Clamp area to drawable part of the display target
        let drawable_area = area.intersection(&Rectangle::new(Point::zero(), self.size()));

        if drawable_area.size != Size::zero() {
            self.set_pixels_buffered(
                drawable_area.top_left.x as u16,
                drawable_area.top_left.y as u16,
                (drawable_area.top_left.x + (drawable_area.size.width - 1) as i32) as u16,
                (drawable_area.top_left.y + (drawable_area.size.height - 1) as i32) as u16,
                area.points()
                    .zip(colors)
                    .filter(|(pos, _color)| drawable_area.contains(*pos))
                    .map(|(_pos, color)| RawU16::from(color).into_inner()),
            )?;
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
        )
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
