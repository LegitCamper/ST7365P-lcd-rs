extern crate std;

use super::*;
use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use std::{
    collections::VecDeque,
    future::{pending, Future},
    task::{Context, Poll, Waker},
    vec,
    vec::Vec,
};

struct AtomicStorage<const SIZE: usize>([core::sync::atomic::AtomicU16; SIZE]);

impl<const SIZE: usize> AtomicStorage<SIZE> {
    fn new() -> Self {
        Self(core::array::from_fn(|_| {
            core::sync::atomic::AtomicU16::new(0)
        }))
    }
}

impl<const SIZE: usize> PixelStorage for AtomicStorage<SIZE> {
    fn len(&self) -> usize {
        SIZE
    }

    fn read(&self, index: usize) -> u16 {
        self.0[index].load(core::sync::atomic::Ordering::Relaxed)
    }

    fn write(&mut self, index: usize, value: u16) {
        self.0[index].store(value, core::sync::atomic::Ordering::Relaxed);
    }
}

#[test]
fn storage_backends_share_drawing_and_wire_output() {
    fn exercise(storage: impl PixelStorage) -> Vec<Vec<u8>> {
        let mut framebuffer = FrameBuffer::<33, 35, { 33 * 35 }, _>::with_storage(storage);
        let mut display = display();
        framebuffer.clear(Rgb565::BLACK).unwrap();
        run(framebuffer.draw(&mut display)).unwrap();
        display.spi.writes.clear();

        framebuffer
            .fill_contiguous(
                &Rectangle::new(Point::new(-1, 16), Size::new(3, 2)),
                [Rgb565::WHITE, Rgb565::RED, Rgb565::BLUE, Rgb565::WHITE],
            )
            .unwrap();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::new(32, 34), Size::new(8, 8)),
                Rgb565::GREEN,
            )
            .unwrap();
        framebuffer
            .draw_iter([Pixel(Point::new(17, 0), Rgb565::WHITE)])
            .unwrap();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        let mut expected = vec![0; 33 * 35];
        expected[16 * 33] = 0xf800_u16.swap_bytes();
        expected[16 * 33 + 1] = 0x001f_u16.swap_bytes();
        expected[34 * 33 + 32] = 0x07e0_u16.swap_bytes();
        expected[17] = 0xffff;
        assert_panel_matches(&display.spi.writes, 33, 35, &expected);
        let writes = display.spi.writes.len();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), writes);
        display.spi.writes
    }

    let owned: DefaultPixelStorage<{ 33 * 35 }> = [0; 33 * 35];
    let expected = exercise(owned);
    let mut borrowed = [0u16; 33 * 35];
    assert_eq!(exercise(&mut borrowed[..]), expected);
    assert_eq!(exercise(AtomicStorage::<{ 33 * 35 }>::new()), expected);
}

#[test]
fn custom_storage_full_draw_packs_edges_and_retries_failed_tiles() {
    let mut framebuffer =
        FrameBuffer::<17, 19, { 17 * 19 }, _>::with_storage(AtomicStorage::<{ 17 * 19 }>::new());
    framebuffer.clear(Rgb565::BLUE).unwrap();
    let mut display = display();
    display.spi.fail_at = Some(6);
    assert_eq!(run(framebuffer.draw(&mut display)), Err(()));
    assert!(!framebuffer.dirty_tiles[0]);
    assert_eq!(framebuffer.dirty_tiles.count_ones(), 3);
    display.spi.fail_at = None;
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert!(!framebuffer.dirty_tiles.any());
    assert_panel_matches(
        &display.spi.writes,
        17,
        19,
        &[0x001f_u16.swap_bytes(); 17 * 19],
    );
}

#[test]
fn custom_storage_batches_wide_rows_with_bounded_scratch() {
    let mut framebuffer =
        FrameBuffer::<480, 32, { 480 * 32 }, _>::with_storage(AtomicStorage::<{ 480 * 32 }>::new());
    let mut display = display();
    run(framebuffer.draw(&mut display)).unwrap();
    display.spi.writes.clear();
    framebuffer
        .fill_solid(
            &Rectangle::new(Point::zero(), Size::new(480, 16)),
            Rgb565::RED,
        )
        .unwrap();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 12);
    let mut expected = vec![0; 480 * 32];
    expected[..480 * 16].fill(0xf800_u16.swap_bytes());
    assert_panel_matches(&display.spi.writes, 480, 32, &expected);
}

#[test]
fn borrowed_storage_is_dirty_on_attach_and_remains_zero_copy() {
    let mut pixels = [0xf800_u16.swap_bytes(); 32 * 16];
    let expected_pointer = pixels.as_ptr() as usize;
    let mut framebuffer = FrameBuffer::<32, 16, 512, _>::with_storage(&mut pixels[..]);
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.pointers[5], expected_pointer);
    assert_eq!(&display.spi.writes[5][..2], &[0xf8, 0x00]);
}

#[test]
#[should_panic(expected = "Framebuffer storage length mismatch")]
fn mismatched_storage_is_rejected_at_construction() {
    FrameBuffer::<16, 16, 256, _>::with_storage([0; 255]);
}

#[test]
fn eighty_percent_dirty_uses_full_frame_transfer() {
    let mut framebuffer = FrameBuffer::<80, 16, { 80 * 16 }>::new();
    framebuffer
        .fill_solid(
            &Rectangle::new(Point::zero(), Size::new(64, 16)),
            Rgb565::RED,
        )
        .unwrap();
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[5].len(), 80 * 16 * 2);
}

#[derive(Default)]
struct RecordingSpi {
    writes: Vec<Vec<u8>>,
    pointers: Vec<usize>,
    fail_at: Option<usize>,
    pending_at: Option<usize>,
    scanlines: VecDeque<Result<u16, ErrorKind>>,
    reads: usize,
    reads_at_write: Vec<usize>,
    pending_read_at: Option<usize>,
}

impl ErrorType for RecordingSpi {
    type Error = ErrorKind;
}

impl SpiDevice for RecordingSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        if let [Operation::DelayNs(60), Operation::Write(command), Operation::Read(response), Operation::DelayNs(65)] =
            operations
        {
            assert_eq!(*command, [Instruction::GSCAN as u8]);
            if self.pending_read_at == Some(self.reads) {
                pending::<()>().await;
            }
            self.reads += 1;
            response.copy_from_slice(
                &self
                    .scanlines
                    .pop_front()
                    .expect("missing scan sample")?
                    .to_be_bytes(),
            );
            return Ok(());
        }
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
            self.reads_at_write.push(self.reads);
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
        assert!(nanos == 1 || nanos == 50_000);
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

fn scan_config() -> ScanSyncConfig {
    ScanSyncConfig {
        frame_scanlines: 80,
        first_scanline: 7,
        direction: ScanDirection::TopToBottom,
        write_budget_lines: 5,
        max_polls: 20,
    }
}

#[test]
fn sync_draw_checks_each_tile_and_never_uses_dense_fallback() {
    let mut framebuffer = FrameBuffer::<17, 19, { 17 * 19 }>::new();
    framebuffer.clear(Rgb565::BLUE).unwrap();
    let mut display = display();
    display.spi.scanlines = [10, 22, 23, 15, 23, 24, 25, 26, 27, 30].map(Ok).into();
    run(framebuffer.draw_sync(&mut display, &scan_config())).unwrap();
    assert_panel_matches(&display.spi.writes, 17, 19, &framebuffer.buffer);
    assert_eq!(display.spi.writes.len(), 24);
    assert_eq!(display.spi.reads_at_write[5], 3);
    assert_eq!(display.spi.reads_at_write[11], 5);
    assert_eq!(display.spi.reads_at_write[17], 8);
    assert_eq!(display.spi.reads_at_write[23], 10);
    assert!(!framebuffer.dirty_tiles.any());
    run(framebuffer.draw_sync(&mut display, &scan_config())).unwrap();
    assert_eq!(display.spi.reads, 10);
    assert_eq!(display.spi.writes.len(), 24);
}

#[test]
fn sync_draw_windows_exclude_scan_and_transfer_deadline_in_every_direction() {
    for direction in [
        ScanDirection::TopToBottom,
        ScanDirection::BottomToTop,
        ScanDirection::LeftToRight,
        ScanDirection::RightToLeft,
    ] {
        for tile_index in 0..4 {
            for scanline in 0..80 {
                let mut framebuffer = FrameBuffer::<17, 19, { 17 * 19 }>::new();
                framebuffer.dirty_tiles.set(tile_index, true);
                let mut display = display();
                display.spi.scanlines = [Ok((scanline + 79) % 80), Ok(scanline)].into();
                let config = ScanSyncConfig {
                    direction,
                    max_polls: 2,
                    ..scan_config()
                };
                let mut occupied = [false; 80];
                for row in tile_index / 2 * 16..(tile_index / 2 * 16 + 16).min(19) {
                    for column in tile_index % 2 * 16..(tile_index % 2 * 16 + 16).min(17) {
                        let physical_line = 7 + match direction {
                            ScanDirection::TopToBottom => row,
                            ScanDirection::BottomToTop => 18 - row,
                            ScanDirection::LeftToRight => column,
                            ScanDirection::RightToLeft => 16 - column,
                        };
                        occupied[physical_line] = true;
                    }
                }
                let safe = (0..=config.write_budget_lines + 1)
                    .all(|advance| !occupied[((scanline + advance) % 80) as usize]);
                let result = run(framebuffer.draw_sync(&mut display, &config));
                assert_eq!(
                    result,
                    if safe {
                        Ok(())
                    } else {
                        Err(DrawSyncError::Timeout)
                    },
                    "direction={direction:?}, tile={tile_index}, scanline={scanline}"
                );
                assert_eq!(display.spi.writes.len(), if safe { 6 } else { 0 });
                assert_eq!(framebuffer.dirty_tiles[tile_index], !safe);
            }
        }
    }
}

#[test]
fn sync_draw_waits_across_frame_wrap_instead_of_racing_scan() {
    let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let config = ScanSyncConfig {
        first_scanline: 0,
        ..scan_config()
    };
    let mut display = display();
    display.spi.scanlines = [73, 74, 79, 0, 15, 16].map(Ok).into();
    run(framebuffer.draw_sync(&mut display, &config)).unwrap();
    assert_eq!(display.spi.reads_at_write[0], 6);
}

#[test]
fn sync_draw_uses_explicit_physical_mapping_with_address_offsets() {
    let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let config = ScanSyncConfig {
        first_scanline: 64,
        ..scan_config()
    };
    let mut display = display();
    display.set_offset(2, 3);
    display.spi.scanlines = [78, 79, 0].map(Ok).into();
    run(framebuffer.draw_sync(&mut display, &config)).unwrap();
    assert_eq!(display.spi.reads_at_write[0], 3);
    assert_eq!(display.spi.writes[1], [0, 2, 0, 17]);
    assert_eq!(display.spi.writes[3], [0, 3, 0, 18]);
}

#[test]
fn sync_draw_uses_one_poll_budget_for_entire_flush() {
    let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let config = ScanSyncConfig {
        max_polls: 4,
        ..scan_config()
    };
    let mut display = display();
    display.spi.scanlines = [23, 24, 10, 11].map(Ok).into();
    assert_eq!(
        run(framebuffer.draw_sync(&mut display, &config)),
        Err(DrawSyncError::Timeout)
    );
    assert_eq!(display.spi.reads, 4);
    assert_eq!(display.spi.writes.len(), 6);
    assert!(!framebuffer.dirty_tiles[0]);
    assert!(framebuffer.dirty_tiles[1]);
    display.spi.scanlines = [Ok(23), Ok(24)].into();
    run(framebuffer.draw_sync(&mut display, &config)).unwrap();
    assert_eq!(display.spi.writes.len(), 12);
    assert_panel_matches(&display.spi.writes, 32, 16, &framebuffer.buffer);
}

#[test]
fn sync_draw_rejects_stuck_invalid_and_failed_reads_without_writing() {
    for (samples, expected) in [
        (vec![Ok(0); 4], DrawSyncError::Timeout),
        (vec![Ok(30); 4], DrawSyncError::Timeout),
        (vec![Ok(10), Ok(11), Ok(12), Ok(13)], DrawSyncError::Timeout),
        (vec![Ok(80)], DrawSyncError::InvalidScanline(80)),
        (vec![Ok(u16::MAX)], DrawSyncError::InvalidScanline(u16::MAX)),
        (vec![Err(ErrorKind::Other)], DrawSyncError::Interface),
        (
            vec![Ok(23), Err(ErrorKind::Other)],
            DrawSyncError::Interface,
        ),
    ] {
        let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = samples.into();
        let config = ScanSyncConfig {
            max_polls: 4,
            ..scan_config()
        };
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &config)),
            Err(expected)
        );
        assert!(display.spi.writes.is_empty());
        assert!(framebuffer.dirty_tiles[0]);
    }
}

#[test]
fn sync_draw_read_failure_after_progress_preserves_only_unfinished_tiles() {
    for (sample, error) in [
        (Err(ErrorKind::Other), DrawSyncError::Interface),
        (Ok(80), DrawSyncError::InvalidScanline(80)),
    ] {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [Ok(23), Ok(24), sample].into();
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &scan_config())),
            Err(error)
        );
        assert!(!framebuffer.dirty_tiles[0]);
        assert!(framebuffer.dirty_tiles[1]);
        assert_eq!(display.spi.writes.len(), 6);
    }
}

#[test]
fn sync_draw_validates_configuration_and_poll_budget_before_io() {
    for config in [
        ScanSyncConfig {
            frame_scanlines: 0,
            ..scan_config()
        },
        ScanSyncConfig {
            first_scanline: 65,
            ..scan_config()
        },
        ScanSyncConfig {
            write_budget_lines: 0,
            ..scan_config()
        },
        ScanSyncConfig {
            write_budget_lines: 63,
            ..scan_config()
        },
        ScanSyncConfig {
            write_budget_lines: u16::MAX,
            ..scan_config()
        },
    ] {
        let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
        let mut display = display();
        run(framebuffer.draw_sync(&mut display, &config)).unwrap();
        framebuffer.clear(Rgb565::RED).unwrap();
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &config)),
            Err(DrawSyncError::InvalidConfig)
        );
        assert!(display.spi.writes.is_empty());
        assert_eq!(display.spi.reads, 0);
    }
    for max_polls in [0, 1] {
        let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        let config = ScanSyncConfig {
            max_polls,
            ..scan_config()
        };
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &config)),
            Err(DrawSyncError::Timeout)
        );
        assert_eq!(display.spi.reads, 0);
    }
    for offsets in [(u16::MAX, 0), (0, u16::MAX)] {
        let mut framebuffer = FrameBuffer::<16, 16, 256>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.set_offset(offsets.0, offsets.1);
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &scan_config())),
            Err(DrawSyncError::InvalidConfig)
        );
        assert_eq!(display.spi.reads, 0);
    }
}

#[test]
fn sync_draw_write_failures_preserve_unsent_tiles_for_retry() {
    for fail_at in 0..12 {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [23, 24, 25, 26].map(Ok).into();
        display.spi.fail_at = Some(fail_at);
        assert_eq!(
            run(framebuffer.draw_sync(&mut display, &scan_config())),
            Err(DrawSyncError::Interface)
        );
        let remaining = if fail_at < 6 { 2 } else { 1 };
        assert_eq!(framebuffer.dirty_tiles.count_ones(), remaining);
        display.spi.fail_at = None;
        display.spi.writes.clear();
        display.spi.scanlines = [23, 24, 25, 26].map(Ok).into();
        run(framebuffer.draw_sync(&mut display, &scan_config())).unwrap();
        assert_eq!(display.spi.writes.len(), remaining * 6);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn sync_draw_cancellation_during_poll_or_write_preserves_unsent_tiles() {
    for (pending_read_at, pending_at, remaining) in [
        (Some(1), None, 2),
        (Some(3), None, 1),
        (None, Some(5), 2),
        (None, Some(11), 1),
    ] {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [23, 24, 25, 26].map(Ok).into();
        display.spi.pending_read_at = pending_read_at;
        display.spi.pending_at = pending_at;
        let config = scan_config();
        {
            let mut future = std::pin::pin!(framebuffer.draw_sync(&mut display, &config));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        }
        assert_eq!(framebuffer.dirty_tiles.count_ones(), remaining);
        display.spi.pending_read_at = None;
        display.spi.pending_at = None;
        display.spi.writes.clear();
        display.spi.scanlines = [23, 24, 25, 26].map(Ok).into();
        run(framebuffer.draw_sync(&mut display, &config)).unwrap();
        assert_eq!(display.spi.writes.len(), remaining * 6);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
#[should_panic]
fn oversized_tile_grid_is_rejected_at_construction() {
    let _ = FrameBuffer::<1, 9601, 9601>::new();
}
