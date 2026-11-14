use super::*;

#[test]
fn new_framebuffer_uploads_black_even_without_pixel_changes() {
    const INITIAL: FrameBuffer<17, 19, 323> = FrameBuffer::new();
    let mut framebuffer = INITIAL;
    assert_eq!(framebuffer.dirty_tiles.count_ones(), 4);
    framebuffer
        .draw_iter([Pixel(Point::zero(), Rgb565::BLACK)])
        .unwrap();
    framebuffer.clear(Rgb565::BLACK).unwrap();
    assert_eq!(framebuffer.dirty_tiles.count_ones(), 4);
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[5], vec![0; 323 * 2]);
    assert!(!framebuffer.dirty_tiles.any());
}

#[test]
fn identical_fills_preserve_pending_changes_without_dirtying_other_tiles() {
    fn check(storage: impl PixelStorage) {
        let mut framebuffer = FrameBuffer::<33, 35, 1155, _>::with_storage(storage);
        let mut display = display();
        run(framebuffer.draw(&mut display)).unwrap();
        display.spi.writes.clear();
        framebuffer.clear(Rgb565::BLACK).unwrap();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::new(-4, -3), Size::new(40, 40)),
                Rgb565::BLACK,
            )
            .unwrap();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert!(display.spi.writes.is_empty());
        framebuffer
            .draw_iter([Pixel(Point::new(32, 34), Rgb565::RED)])
            .unwrap();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::new(32, 34), Size::new(5, 5)),
                Rgb565::RED,
            )
            .unwrap();
        assert_eq!(
            framebuffer.dirty_tiles.iter_ones().collect::<Vec<_>>(),
            vec![8]
        );
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        display.spi.writes.clear();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::new(32, 34), Size::new(5, 5)),
                Rgb565::RED,
            )
            .unwrap();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert!(display.spi.writes.is_empty());
        framebuffer.clear(Rgb565::BLACK).unwrap();
        assert_eq!(
            framebuffer.dirty_tiles.iter_ones().collect::<Vec<_>>(),
            vec![8]
        );
    }
    check([0; 1155]);
    check(&mut [0; 1155][..]);
    check(AtomicStorage::<1155>::new());
}

#[test]
fn full_draw_error_or_cancellation_marks_clean_frame_for_partial_retry() {
    fn check(storage: impl PixelStorage, cancelled: bool) {
        let mut framebuffer = FrameBuffer::<17, 19, 323, _>::with_storage(storage);
        let mut display = display();
        run(framebuffer.draw(&mut display)).unwrap();
        display.spi.writes.clear();
        if cancelled {
            display.spi.pending_at = Some(5);
            let mut future = std::pin::pin!(framebuffer.draw(&mut display));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        } else {
            display.spi.fail_at = Some(5);
            assert_eq!(run(framebuffer.draw(&mut display)), Err(()));
        }
        assert_eq!(framebuffer.dirty_tiles.count_ones(), 4);
        display.spi.pending_at = None;
        display.spi.fail_at = None;
        display.spi.writes.clear();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), 6);
        assert!(!framebuffer.dirty_tiles.any());
    }
    for cancelled in [false, true] {
        check([0; 323], cancelled);
        check(&mut [0; 323][..], cancelled);
    }
}

#[test]
fn vertical_strip_and_full_width_band_merge_without_extra_pixels() {
    for (width, height, expected_pointer) in [(16, 320, false), (320, 64, true)] {
        let mut framebuffer = synchronized_black_framebuffer::<320, 320, 102400>();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::zero(), Size::new(width, height)),
                Rgb565::RED,
            )
            .unwrap();
        let mut display = display();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), 6);
        assert_eq!(display.spi.writes[5].len(), (width * height * 2) as usize);
        if expected_pointer {
            assert_eq!(
                display.spi.pointers[5],
                framebuffer.buffer.as_ptr() as usize
            );
        }
        assert_panel_matches(&display.spi.writes, 320, 320, &framebuffer.buffer);
    }
}

#[test]
fn dense_non_slice_storage_stays_batched_and_preserves_clean_pixels() {
    for height in [240, 256, 320] {
        let mut framebuffer =
            FrameBuffer::<320, 320, 102400, _>::with_storage(AtomicStorage::<102400>::new());
        let mut display = display();
        run(framebuffer.draw(&mut display)).unwrap();
        display.spi.writes.clear();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::zero(), Size::new(320, height)),
                Rgb565::RED,
            )
            .unwrap();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), height as usize / 16 * 6);
        let payload: usize = display
            .spi
            .writes
            .chunks_exact(6)
            .map(|packet| packet[5].len())
            .sum();
        assert_eq!(payload, 320 * height as usize * 2);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn vertical_batches_split_at_scratch_limit_and_preserve_failed_rectangles() {
    for cancelled in [false, true] {
        let mut framebuffer = synchronized_black_framebuffer::<32, 480, 15360>();
        framebuffer
            .fill_solid(
                &Rectangle::new(Point::zero(), Size::new(16, 480)),
                Rgb565::RED,
            )
            .unwrap();
        let mut display = display();
        if cancelled {
            display.spi.pending_at = Some(11);
            let mut future = std::pin::pin!(framebuffer.partial_draw_batched(&mut display));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        } else {
            display.spi.fail_at = Some(11);
            assert_eq!(run(framebuffer.partial_draw_batched(&mut display)), Err(()));
        }
        assert_eq!(display.spi.writes[5].len(), 16 * 320 * 2);
        assert_eq!(framebuffer.dirty_tiles.count_ones(), 10);
        assert!(!framebuffer.dirty_tiles[..40].any());
        display.spi.fail_at = None;
        display.spi.pending_at = None;
        display.spi.writes.clear();
        run(framebuffer.partial_draw_batched(&mut display)).unwrap();
        assert_eq!(display.spi.writes.len(), 6);
        assert_eq!(display.spi.writes[5].len(), 16 * 160 * 2);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn vertical_merging_respects_gaps_and_clipped_edges() {
    let mut framebuffer = synchronized_black_framebuffer::<33, 51, 1683>();
    for point in [
        Point::new(16, 0),
        Point::new(32, 0),
        Point::new(16, 16),
        Point::new(32, 32),
        Point::new(32, 50),
    ] {
        framebuffer
            .draw_iter([Pixel(point, Rgb565::GREEN)])
            .unwrap();
    }
    let mut display = display();
    run(framebuffer.partial_draw_batched(&mut display)).unwrap();
    assert_panel_matches(&display.spi.writes, 33, 51, &framebuffer.buffer);
    assert_eq!(display.spi.writes.len(), 18);
    assert_eq!(display.spi.writes[17].len(), 19 * 2);
    assert!(!framebuffer.dirty_tiles.any());
}
