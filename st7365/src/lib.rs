#![no_std]
#![allow(async_fn_in_trait)]

//! This crate provides a ST7365P driver to connect to TFT displays.

pub mod instruction;
use crate::instruction::Instruction;
mod fb;
pub use fb::FrameBuffer;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

const MAX_SCREEN_WIDTH: usize = 320;
const MAX_SCREEN_HEIGHT: usize = 480;

// `write_words_buffered` reinterprets pre-swapped `u16` pixels as raw bytes
// instead of copying word-by-word through `to_be_bytes`. That's only valid
// on little-endian targets, where swap_bytes() on write already leaves the
// in-memory byte order MSB-first as the display expects.
const _: () = assert!(cfg!(target_endian = "little"));

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

    // `words` must already be byte-swapped by the caller (see the
    // little-endian assertion above); this just reinterprets them as bytes
    // and issues a single DMA write instead of copying word-by-word.
    async fn write_words_buffered(&mut self, words: &[u16]) -> Result<(), ()> {
        let bytes = unsafe {
            core::slice::from_raw_parts(words.as_ptr() as *const u8, words.len() * 2)
        };
        self.write_data(bytes).await
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
    pub async fn write_pixels_buffered(&mut self, colors: &[u16]) -> Result<(), ()> {
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

    pub async fn set_pixels_buffered(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: &[u16],
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
