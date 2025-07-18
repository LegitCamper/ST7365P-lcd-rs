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
use embedded_graphics::{
    image::Image, pixelcolor::Rgb565,
    prelude::*, Drawable,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use st7365p_lcd::ST7365P;
use tinybmp::Bmp;

const SCREEN_WIDTH: usize = 320;
const SCREEN_HEIGHT: usize = 320;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
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
        SCREEN_WIDTH as u32,
        SCREEN_HEIGHT as u32,
        Delay,
    );
    display.init().unwrap();
    display.set_custom_orientation(0x40).unwrap();

    let bmp: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../../assets/ferriseyes.bmp")).unwrap();
    let image = Image::new(&bmp, Point::new(0, 0));

    image.draw(&mut display).unwrap();

    loop {}
}
