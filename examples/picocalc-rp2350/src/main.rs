#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    spi,
};
use embassy_time::{Delay, Duration, Instant, block_for};
use embedded_graphics::{Drawable, image::Image, pixelcolor::Rgb565, prelude::*};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use st7365p_lcd::{FrameBuffer, ST7365P, framebuffer, spi::ReadWriteSpi};
use tinybmp::Bmp;

const SCREEN_WIDTH: usize = 320;
const SCREEN_HEIGHT: usize = 320;

struct SpiDelay;

impl DelayNs for SpiDelay {
    async fn delay_ns(&mut self, nanos: u32) {
        if nanos <= 1_000 {
            block_for(Duration::from_nanos(u64::from(nanos)));
        } else {
            Delay.delay_ns(nanos).await;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = spi::Config::default();
    let write_frequency = if cfg!(feature = "fast-spi") {
        37_500_000
    } else {
        32_000_000
    };
    config.frequency = write_frequency;
    let spi1 = spi::Spi::new(
        p.SPI1,
        p.PIN_10,
        p.PIN_11,
        p.PIN_12,
        p.DMA_CH0,
        p.DMA_CH1,
        config.clone(),
    );

    let spi_device =
        ExclusiveDevice::new(spi1, Output::new(p.PIN_13, Level::Low), SpiDelay).unwrap();
    let spi_device = ReadWriteSpi::new(spi_device, move |device, reads| {
        config.frequency = if reads { 6_000_000 } else { write_frequency };
        device.bus_mut().set_config(&config);
        Ok(())
    });
    let mut display = ST7365P::new(
        spi_device,
        Output::new(p.PIN_14, Level::Low),
        Some(Output::new(p.PIN_15, Level::High)),
        false,
        true,
        Delay,
    );
    let mut framebuffer = framebuffer!(SCREEN_WIDTH, SCREEN_HEIGHT);

    display.init().await.unwrap();
    display.set_custom_orientation(0x40).await.unwrap();
    framebuffer.draw(&mut display).await.unwrap();
    display.set_on().await.unwrap();

    let bmp: Bmp<Rgb565> =
        Bmp::from_slice(include_bytes!("../../../assets/ferriseyes.bmp")).unwrap();
    let y_offset = bmp.size().height as i32 / 4;
    let image = Image::new(&bmp, Point::new(0, y_offset));

    let render_started = Instant::now();
    image.draw(&mut framebuffer).unwrap();
    let render_us = render_started.elapsed().as_micros();

    let sync_started = Instant::now();
    let sync = display.wait_for_vsync(2_000).await;
    let sync_us = sync_started.elapsed().as_micros();
    if let Err(error) = sync {
        defmt::warn!(
            "LCD scan sync failed: {:?}; drawing unsynchronized",
            defmt::Debug2Format(&error)
        );
    }

    let flush_started = Instant::now();
    framebuffer
        .partial_draw_batched(&mut display)
        .await
        .unwrap();
    defmt::info!(
        "LCD render={}us sync={}us flush={}us",
        render_us,
        sync_us,
        flush_started.elapsed().as_micros()
    );

    loop {}
}
