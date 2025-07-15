#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    spi,
};
use embassy_time::Delay;
use embedded_graphics::{image::Image, pixelcolor::Rgb565, prelude::*, Drawable};
use embedded_hal_bus::spi::ExclusiveDevice;
use st7365p_lcd::{FrameBuffer, ST7365P};
use tinybmp::Bmp;

const SCREEN_WIDTH: usize = 320;
const SCREEN_HEIGHT: usize = 320;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = spi::Config::default();
    config.frequency = 32_000_000;
    let spi1 = spi::Spi::new(
        p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, p.DMA_CH0, p.DMA_CH1, config,
    );

    let spi_device = ExclusiveDevice::new(spi1, Output::new(p.PIN_13, Level::Low), Delay).unwrap();
    let mut display = ST7365P::new(
        spi_device,
        Output::new(p.PIN_14, Level::Low),
        Some(Output::new(p.PIN_15, Level::High)),
        false,
        true,
        Delay,
    );
    let mut framebuffer: FrameBuffer<
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        { SCREEN_WIDTH * SCREEN_HEIGHT },
    > = FrameBuffer::new();

    display.init().await.unwrap();
    display.set_custom_orientation(0x40).await.unwrap();
    framebuffer.draw(&mut display).await.unwrap();
    display.set_on().await.unwrap();

    let bmp: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../../assets/ferriseyes.bmp")).unwrap();
    let y_offset = bmp.size().height as i32 / 4;
    let image = Image::new(&bmp, Point::new(0, y_offset));

    image.draw(&mut framebuffer).unwrap();

    framebuffer
        .partial_draw_batched(&mut display)
        .await
        .unwrap();

    loop {}
}
