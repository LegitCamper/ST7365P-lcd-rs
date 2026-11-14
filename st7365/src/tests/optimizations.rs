use super::*;
use std::cell::Cell;

#[test]
fn iterator_pixels_are_chunked_in_wire_order_without_empty_writes() {
    for length in [0usize, 1, 63, 64, 65, 128, 129] {
        let colors: Vec<u16> = (0..length).map(|index| 0x1234 + index as u16).collect();
        let mut display = display();
        run(display.write_pixels(colors.iter().copied())).unwrap();
        assert_eq!(display.spi.writes[0], [Instruction::RAMWR as u8]);
        assert_eq!(display.spi.writes.len(), 1 + length.div_ceil(64));
        let bytes: Vec<u8> = display.spi.writes[1..].iter().flatten().copied().collect();
        let expected: Vec<u8> = colors
            .iter()
            .flat_map(|color| color.to_be_bytes())
            .collect();
        assert_eq!(bytes, expected);
        assert_eq!(display.delay.calls, 0);
        assert_eq!(display.spi.write_delays, display.spi.writes.len());
    }
}

#[test]
fn iterator_pixels_stop_on_failed_chunk() {
    let consumed = Cell::new(0);
    let colors = (0..200).inspect(|_| {
        consumed.set(consumed.get() + 1);
    });
    let mut display = display();
    display.spi.fail_at = Some(2);
    assert_eq!(run(display.write_pixels(colors)), Err(()));
    assert_eq!(consumed.get(), 128);
    assert_eq!(display.spi.writes[1].len(), 128);
}

#[test]
fn address_cache_skips_only_unchanged_axes() {
    let mut display = display();
    run(display.set_pixel(0, 0, 0x1234)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    run(display.set_pixel(0, 0, 0x5678)).unwrap();
    assert_eq!(&display.spi.writes[6..], &[vec![0x2c], vec![0x56, 0x78]]);
    run(display.set_pixel(1, 0, 0)).unwrap();
    assert_eq!(&display.spi.writes[8..10], &[vec![0x2a], vec![0, 1, 0, 1]]);
    assert_eq!(display.spi.writes.len(), 12);
    run(display.set_pixel(1, 1, 0)).unwrap();
    assert_eq!(&display.spi.writes[12..14], &[vec![0x2b], vec![0, 1, 0, 1]]);
    assert_eq!(display.spi.writes.len(), 16);
}

#[test]
fn address_cache_invalidates_on_state_changes_and_raw_access() {
    let mut display = display();
    for change in 0..6 {
        run(display.set_address_window(0, 0, 15, 15)).unwrap();
        match change {
            0 => {
                display.spi_mut();
            }
            1 => display.set_offset(1, 2),
            2 => run(display.set_orientation(&Orientation::Landscape)).unwrap(),
            3 => run(display.set_custom_orientation(0x40)).unwrap(),
            4 => run(display.hard_reset()).unwrap(),
            _ => run(display.init()).unwrap(),
        }
        let before = display.spi.writes.len();
        run(display.set_address_window(0, 0, 15, 15)).unwrap();
        assert_eq!(display.spi.writes.len() - before, 4);
    }
}

#[test]
fn interrupted_window_changes_cannot_reuse_old_axis_values() {
    for cancelled in [false, true] {
        for offset in 0..4 {
            let mut display = display();
            run(display.set_address_window(0, 0, 15, 15)).unwrap();
            if cancelled {
                display.spi.pending_at = Some(4 + offset);
                let mut future = std::pin::pin!(display.set_address_window(16, 16, 31, 31));
                assert!(future
                    .as_mut()
                    .poll(&mut Context::from_waker(Waker::noop()))
                    .is_pending());
            } else {
                display.spi.fail_at = Some(4 + offset);
                assert_eq!(run(display.set_address_window(16, 16, 31, 31)), Err(()));
            }
            display.spi.pending_at = None;
            display.spi.fail_at = None;
            let before = display.spi.writes.len();
            run(display.set_address_window(0, 0, 15, 15)).unwrap();
            assert_eq!(
                display.spi.writes.len() - before,
                if offset < 2 { 2 } else { 4 }
            );
            assert_eq!(display.column_window, Some((0, 15)));
            assert_eq!(display.row_window, Some((0, 15)));
        }
    }
}

#[test]
fn invalid_windows_fail_without_spi_or_cache_changes() {
    let mut display = display();
    display.set_offset(1, 1);
    run(display.set_address_window(0, 0, 15, 15)).unwrap();
    for (start_x, start_y, end_x, end_y) in [
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, u16::MAX, 0),
        (0, 0, 0, u16::MAX),
    ] {
        assert_eq!(
            run(display.set_address_window(start_x, start_y, end_x, end_y)),
            Err(())
        );
        assert_eq!(display.spi.writes.len(), 4);
        assert_eq!(display.column_window, Some((1, 16)));
        assert_eq!(display.row_window, Some((1, 16)));
    }
}

struct CountedStorage<const SIZE: usize> {
    pixels: [u16; SIZE],
    reads: Cell<usize>,
    bulk_writes: usize,
}

impl<const SIZE: usize> CountedStorage<SIZE> {
    fn new() -> Self {
        Self {
            pixels: [0; SIZE],
            reads: Cell::new(0),
            bulk_writes: 0,
        }
    }
}

impl<const SIZE: usize> PixelStorage for CountedStorage<SIZE> {
    fn len(&self) -> usize {
        SIZE
    }
    fn read(&self, index: usize) -> u16 {
        self.reads.set(self.reads.get() + 1);
        self.pixels[index]
    }
    fn write(&mut self, index: usize, value: u16) {
        self.pixels[index] = value;
    }
    fn write_from(&mut self, start: usize, source: &[u16]) {
        self.bulk_writes += 1;
        self.pixels[start..start + source.len()].copy_from_slice(source);
    }
}

#[test]
fn dirty_diff_skips_clean_tiles_and_suppresses_reverted_drawing() {
    let mut framebuffer =
        FrameBuffer::<32, 32, 1024, _>::with_storage(CountedStorage::<1024>::new());
    let mut history = DisplayState::new([0; 1024]);
    let mut display = display();
    run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
    framebuffer.buffer.reads.set(0);
    let initial_writes = display.spi.writes.len();
    run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
    assert_eq!(framebuffer.buffer.reads.get(), 0);
    framebuffer
        .draw_iter([Pixel(Point::zero(), Rgb565::RED)])
        .unwrap();
    framebuffer
        .draw_iter([Pixel(Point::zero(), Rgb565::BLACK)])
        .unwrap();
    framebuffer.buffer.reads.set(0);
    run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
    assert_eq!(framebuffer.buffer.reads.get(), 256);
    assert_eq!(display.spi.writes.len(), initial_writes);
    assert!(!framebuffer.dirty_tiles.any());
    history.invalidate();
    run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
    assert!(display.spi.writes.len() > initial_writes);
}

#[test]
fn dirty_diff_retries_invalid_history_after_error_or_cancellation() {
    for cancelled in [false, true] {
        let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
        let mut history = DisplayState::new([0; 1024]);
        let mut display = display();
        run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
        framebuffer
            .draw_iter([Pixel(Point::zero(), Rgb565::RED)])
            .unwrap();
        display.spi_mut().writes.clear();
        if cancelled {
            display.spi.pending_at = Some(5);
            let mut future =
                std::pin::pin!(framebuffer.draw_diff_dirty(&mut display, &mut history));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        } else {
            display.spi.fail_at = Some(5);
            assert_eq!(
                run(framebuffer.draw_diff_dirty(&mut display, &mut history)),
                Err(())
            );
        }
        framebuffer
            .draw_iter([Pixel(Point::zero(), Rgb565::BLACK)])
            .unwrap();
        display.spi.pending_at = None;
        display.spi.fail_at = None;
        let before = display.spi.writes.len();
        run(framebuffer.draw_diff_dirty(&mut display, &mut history)).unwrap();
        assert!(display.spi.writes.len() > before);
        assert_eq!(display.spi.writes.last().unwrap(), &vec![0; 512]);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn history_commit_uses_bulk_rows_for_slice_and_custom_storage() {
    fn check(storage: impl PixelStorage) {
        let mut framebuffer = FrameBuffer::<17, 19, 323, _>::with_storage(storage);
        let mut history_storage = CountedStorage::<323>::new();
        {
            let mut history = DisplayState::new(&mut history_storage);
            framebuffer.clear(Rgb565::BLUE).unwrap();
            run(framebuffer.draw_diff(&mut display(), &mut history)).unwrap();
        }
        assert_eq!(history_storage.bulk_writes, 38);
        assert_eq!(history_storage.pixels, [0x001f_u16.swap_bytes(); 323]);
    }
    check([0; 323]);
    check(AtomicStorage::<323>::new());
}

impl<const SIZE: usize> PixelStorage for &mut CountedStorage<SIZE> {
    fn len(&self) -> usize {
        SIZE
    }
    fn read(&self, index: usize) -> u16 {
        (**self).read(index)
    }
    fn write(&mut self, index: usize, value: u16) {
        (**self).write(index, value);
    }
    fn write_from(&mut self, start: usize, source: &[u16]) {
        (**self).write_from(start, source);
    }
}

#[test]
fn reused_scratch_never_sends_stale_pixels() {
    fn check(storage: impl PixelStorage, synchronized: bool) {
        let mut framebuffer = FrameBuffer::<17, 19, 323, _>::with_storage(storage);
        let mut scratch = [0xdead; 5120];
        let mut display = display();
        for color in [Rgb565::RED, Rgb565::BLUE] {
            framebuffer.clear(color).unwrap();
            display.spi_mut().writes.clear();
            if synchronized {
                display.spi.scanlines = [60, 61, 62, 63].map(Ok).into();
                run(framebuffer.draw_sync_batched_with_scratch(
                    &mut display,
                    &scan_config(),
                    &ScanBatchConfig {
                        max_tiles: 2,
                        write_budget_lines: 9,
                    },
                    &mut scratch,
                ))
                .unwrap();
            } else {
                framebuffer.dirty_tiles.set(0, false);
                run(framebuffer.partial_draw_batched_with_scratch(&mut display, &mut scratch))
                    .unwrap();
            }
            let mut expected = [0; 323];
            for row in 0..19 {
                for column in 0..17 {
                    if synchronized || row >= 16 || column >= 16 {
                        expected[row * 17 + column] = RawU16::from(color).into_inner().swap_bytes();
                    }
                }
            }
            assert_panel_matches(&display.spi.writes, 17, 19, &expected);
            assert_eq!(scratch[5119], 0xdead);
        }
    }
    for synchronized in [false, true] {
        check([0; 323], synchronized);
        check(AtomicStorage::<323>::new(), synchronized);
    }
}
