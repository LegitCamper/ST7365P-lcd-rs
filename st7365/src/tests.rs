extern crate std;

use super::*;
use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use std::{
    future::{pending, Future},
    task::{Context, Poll, Waker},
    vec,
    vec::Vec,
};

#[derive(Default)]
struct RecordingSpi {
    writes: Vec<Vec<u8>>,
    pointers: Vec<usize>,
    fail_at: Option<usize>,
    pending_at: Option<usize>,
}

impl ErrorType for RecordingSpi {
    type Error = ErrorKind;
}

impl SpiDevice for RecordingSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            if self.fail_at == Some(self.writes.len()) {
                return Err(ErrorKind::Other);
            }
            if self.pending_at == Some(self.writes.len()) {
                pending::<()>().await;
            }
            let Operation::Write(bytes) = operation else {
                panic!("unexpected SPI operation");
            };
            self.pointers.push(bytes.as_ptr() as usize);
            self.writes.push(bytes.to_vec());
        }
        Ok(())
    }
}

struct Pin;

impl embedded_hal::digital::ErrorType for Pin {
    type Error = core::convert::Infallible;
}

impl OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Default)]
struct Delay {
    calls: usize,
}

impl DelayNs for Delay {
    async fn delay_ns(&mut self, nanos: u32) {
        assert_eq!(nanos, 1);
        self.calls += 1;
    }
}

fn display() -> ST7365P<RecordingSpi, Pin, Pin, Delay> {
    ST7365P::new(
        RecordingSpi::default(),
        Pin,
        None,
        true,
        false,
        Delay::default(),
    )
}

fn run<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(output) => output,
        Poll::Pending => panic!("unexpected pending future"),
    }
}

fn assert_panel_matches(writes: &[Vec<u8>], width: usize, height: usize, expected: &[u16]) {
    assert_eq!(writes.len() % 6, 0);
    let mut panel = vec![0; width * height];
    for transfer in writes.chunks(6) {
        assert_eq!(transfer[0], [Instruction::CASET as u8]);
        assert_eq!(transfer[2], [Instruction::RASET as u8]);
        assert_eq!(transfer[4], [Instruction::RAMWR as u8]);
        assert_eq!(transfer[1].len(), 4);
        assert_eq!(transfer[3].len(), 4);
        let start_x = u16::from_be_bytes([transfer[1][0], transfer[1][1]]) as usize;
        let end_x = u16::from_be_bytes([transfer[1][2], transfer[1][3]]) as usize;
        let start_y = u16::from_be_bytes([transfer[3][0], transfer[3][1]]) as usize;
        let end_y = u16::from_be_bytes([transfer[3][2], transfer[3][3]]) as usize;
        assert!(start_x <= end_x && end_x < width);
        assert!(start_y <= end_y && end_y < height);
        let rect_width = end_x - start_x + 1;
        assert_eq!(transfer[5].len(), rect_width * (end_y - start_y + 1) * 2);
        for (index, bytes) in transfer[5].chunks(2).enumerate() {
            let offset = (start_y + index / rect_width) * width + start_x + index % rect_width;
            panel[offset] = u16::from_ne_bytes([bytes[0], bytes[1]]);
        }
    }
    assert_eq!(panel, expected);
}

#[test]
fn address_window_batches_parameters_and_preserves_offsets() {
    let mut display = display();
    display.set_offset(2, 3);
    run(display.set_address_window(0x123, 0x234, 0x345, 0x456)).unwrap();
    assert_eq!(
        display.spi.writes,
        vec![
            vec![Instruction::CASET as u8],
            vec![0x01, 0x25, 0x03, 0x47],
            vec![Instruction::RASET as u8],
            vec![0x02, 0x37, 0x04, 0x59],
        ]
    );
    assert_eq!(display.delay.calls, 4);
}

#[test]
fn sparse_pixels_only_dirty_changed_tiles_in_any_order() {
    let mut framebuffer = FrameBuffer::<64, 48, { 64 * 48 }>::new();
    let pixels = [
        Pixel(Point::new(63, 47), Rgb565::RED),
        Pixel(Point::zero(), Rgb565::GREEN),
        Pixel(Point::new(20, 20), Rgb565::BLACK),
        Pixel(Point::new(-1, 0), Rgb565::RED),
        Pixel(Point::new(64, 48), Rgb565::RED),
    ];
    framebuffer.draw_iter(pixels).unwrap();
    assert_eq!(
        framebuffer.dirty_tiles.iter_ones().collect::<Vec<_>>(),
        vec![0, 11]
    );
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_panel_matches(&display.spi.writes, 64, 48, &framebuffer.buffer);
    framebuffer.draw_iter(pixels).unwrap();
    assert!(!framebuffer.dirty_tiles.any());
    let writes = display.spi.writes.len();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), writes);
}

#[test]
fn clipped_and_short_contiguous_fills_match_pixel_iteration() {
    let areas = [
        Rectangle::new(Point::new(-2, -3), Size::new(22, 25)),
        Rectangle::new(Point::new(-2, 2), Size::new(6, 4)),
        Rectangle::new(Point::new(15, 17), Size::new(6, 4)),
        Rectangle::new(Point::new(3, 4), Size::new(4, 5)),
        Rectangle::new(Point::new(20, 20), Size::new(2, 2)),
        Rectangle::new(Point::zero(), Size::new(0, 5)),
        Rectangle::new(Point::zero(), Size::new(5, 0)),
    ];
    for area in areas {
        for length in [0, 1, 9, 40, 600] {
            let colors: Vec<_> = (0..length)
                .map(|index| Rgb565::from(RawU16::new(index + 1)))
                .collect();
            let mut expected = FrameBuffer::<17, 19, { 17 * 19 }>::new();
            expected
                .draw_iter(
                    area.points()
                        .zip(colors.iter().copied())
                        .map(|(point, color)| Pixel(point, color)),
                )
                .unwrap();
            let mut actual = FrameBuffer::<17, 19, { 17 * 19 }>::new();
            actual
                .fill_contiguous(&area, colors.iter().copied())
                .unwrap();
            assert_eq!(actual.buffer, expected.buffer);
            assert_eq!(actual.dirty_tiles, expected.dirty_tiles);
            actual.dirty_tiles.fill(false);
            actual.fill_contiguous(&area, colors).unwrap();
            assert!(!actual.dirty_tiles.any());
        }
    }
}

#[test]
fn solid_fills_clip_once_and_match_pixel_iteration() {
    for area in [
        Rectangle::new(Point::new(-2, -3), Size::new(22, 25)),
        Rectangle::new(Point::new(15, 17), Size::new(6, 4)),
        Rectangle::new(Point::new(3, 4), Size::new(4, 5)),
        Rectangle::new(Point::new(-20, -20), Size::new(2, 2)),
        Rectangle::new(Point::zero(), Size::new(0, 5)),
        Rectangle::new(Point::zero(), Size::new(5, 0)),
    ] {
        let mut expected = FrameBuffer::<17, 19, { 17 * 19 }>::new();
        expected
            .draw_iter(area.points().map(|point| Pixel(point, Rgb565::RED)))
            .unwrap();
        let mut actual = FrameBuffer::<17, 19, { 17 * 19 }>::new();
        actual.fill_solid(&area, Rgb565::RED).unwrap();
        assert_eq!(actual.buffer, expected.buffer);
        assert_eq!(actual.dirty_tiles, expected.dirty_tiles);
    }
}

#[test]
fn fragmented_flush_drains_more_than_thirty_runs() {
    let mut framebuffer = FrameBuffer::<320, 320, { 320 * 320 }>::new();
    for tile_y in 0..20 {
        for tile_x in (tile_y % 2..20).step_by(2) {
            framebuffer
                .draw_iter([Pixel(Point::new(tile_x * 16, tile_y * 16), Rgb565::RED)])
                .unwrap();
        }
    }
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 200 * 6);
    assert!(!framebuffer.dirty_tiles.any());
    assert_panel_matches(&display.spi.writes, 320, 320, &framebuffer.buffer);
}

#[test]
fn full_width_run_borrows_framebuffer_memory() {
    let mut framebuffer = FrameBuffer::<32, 48, { 32 * 48 }>::new();
    framebuffer
        .fill_solid(
            &Rectangle::new(Point::new(0, 16), Size::new(32, 16)),
            Rgb565::RED,
        )
        .unwrap();
    let expected_pointer = framebuffer.buffer.as_ptr() as usize + 16 * 32 * 2;
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.pointers[5], expected_pointer);
    assert_eq!(&display.spi.writes[5][..2], &[0xf8, 0x00]);
    assert_panel_matches(&display.spi.writes, 32, 48, &framebuffer.buffer);
}

#[test]
fn full_width_run_can_exceed_scratch_capacity_without_copying() {
    let mut framebuffer = FrameBuffer::<480, 32, { 480 * 32 }>::new();
    framebuffer
        .fill_solid(
            &Rectangle::new(Point::zero(), Size::new(480, 16)),
            Rgb565::GREEN,
        )
        .unwrap();
    let expected_pointer = framebuffer.buffer.as_ptr() as usize;
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.pointers[5], expected_pointer);
    assert_panel_matches(&display.spi.writes, 480, 32, &framebuffer.buffer);
}

#[test]
fn clear_flushes_dense_frame_in_one_borrowed_transfer() {
    let mut framebuffer = FrameBuffer::<320, 320, { 320 * 320 }>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    assert_eq!(framebuffer.dirty_tiles.count_ones(), 400);
    assert!(framebuffer
        .buffer
        .iter()
        .all(|pixel| *pixel == 0xf800_u16.swap_bytes()));
    let expected_pointer = framebuffer.buffer.as_ptr() as usize;
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.delay.calls, 6);
    assert_eq!(display.spi.pointers[5], expected_pointer);
    assert!(!framebuffer.dirty_tiles.any());
    assert_panel_matches(&display.spi.writes, 320, 320, &framebuffer.buffer);
}

#[test]
fn both_partial_flushes_pack_edge_tiles_without_padding() {
    for batched in [false, true] {
        let mut framebuffer = FrameBuffer::<17, 19, { 17 * 19 }>::new();
        framebuffer
            .draw_iter([
                Pixel(Point::new(16, 0), Rgb565::RED),
                Pixel(Point::new(0, 18), Rgb565::GREEN),
                Pixel(Point::new(16, 18), Rgb565::BLUE),
            ])
            .unwrap();
        let mut display = display();
        if batched {
            run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        } else {
            run(framebuffer.partial_draw(&mut display)).unwrap();
        }
        assert!(!framebuffer.dirty_tiles.any());
        assert_panel_matches(&display.spi.writes, 17, 19, &framebuffer.buffer);
    }
}

#[test]
fn failed_transfers_preserve_unsent_tiles_for_retry() {
    for fail_at in 0..12 {
        let mut framebuffer = FrameBuffer::<64, 32, { 64 * 32 }>::new();
        framebuffer
            .draw_iter([
                Pixel(Point::zero(), Rgb565::RED),
                Pixel(Point::new(48, 0), Rgb565::GREEN),
            ])
            .unwrap();
        let mut display = display();
        display.spi.fail_at = Some(fail_at);
        assert_eq!(run(framebuffer.partial_draw_batched(&mut display)), Err(()));
        let remaining = if fail_at < 6 { 2 } else { 1 };
        assert_eq!(framebuffer.dirty_tiles.count_ones(), remaining);
        assert!(framebuffer.dirty_tiles[3]);
        display.spi.fail_at = None;
        display.spi.writes.clear();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), remaining * 6);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn cancelled_transfers_preserve_unsent_tiles_for_retry() {
    for pending_at in [5, 11] {
        let mut framebuffer = FrameBuffer::<64, 32, { 64 * 32 }>::new();
        framebuffer
            .draw_iter([
                Pixel(Point::zero(), Rgb565::RED),
                Pixel(Point::new(48, 0), Rgb565::GREEN),
            ])
            .unwrap();
        let mut display = display();
        display.spi.pending_at = Some(pending_at);
        {
            let mut future = std::pin::pin!(framebuffer.partial_draw_batched(&mut display));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        }
        let remaining = if pending_at < 6 { 2 } else { 1 };
        assert_eq!(framebuffer.dirty_tiles.count_ones(), remaining);
        display.spi.pending_at = None;
        display.spi.writes.clear();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), remaining * 6);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn failed_dense_flush_keeps_entire_frame_dirty() {
    let mut framebuffer = FrameBuffer::<17, 19, { 17 * 19 }>::new();
    framebuffer.clear(Rgb565::BLUE).unwrap();
    let mut display = display();
    display.spi.fail_at = Some(5);
    assert_eq!(run(framebuffer.partial_draw_batched(&mut display)), Err(()));
    assert_eq!(framebuffer.dirty_tiles.count_ones(), 4);
    display.spi.fail_at = None;
    display.spi.writes.clear();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_panel_matches(&display.spi.writes, 17, 19, &framebuffer.buffer);
}

#[test]
#[should_panic]
fn oversized_tile_grid_is_rejected_at_construction() {
    let _ = FrameBuffer::<1, 9601, 9601>::new();
}
