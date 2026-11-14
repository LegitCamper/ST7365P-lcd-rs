use super::*;

#[test]
fn diff_suppresses_clear_then_redraw_for_all_storage_backends() {
    fn check(storage: impl PixelStorage, history_storage: impl PixelStorage) {
        let mut framebuffer = FrameBuffer::<19, 35, 665, _>::with_storage(storage);
        let mut history = DisplayState::new(history_storage);
        let mut display = display();
        let pixel = Pixel(Point::new(18, 34), Rgb565::RED);
        framebuffer.draw_iter([pixel]).unwrap();
        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        let initial_writes = display.spi.writes.len();
        assert!(initial_writes > 0);

        framebuffer.clear(Rgb565::BLACK).unwrap();
        framebuffer.draw_iter([pixel]).unwrap();
        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        assert_eq!(display.spi.writes.len(), initial_writes);
        assert!(!framebuffer.dirty_tiles.any());

        framebuffer
            .draw_iter([Pixel(Point::new(18, 34), Rgb565::BLUE)])
            .unwrap();
        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        assert_eq!(display.spi.writes.len(), initial_writes + 6);
        let mut expected = [0; 665];
        expected[664] = RawU16::from(Rgb565::BLUE).into_inner().swap_bytes();
        assert_panel_matches(&display.spi.writes, 19, 35, &expected);
    }

    check([0; 665], [0; 665]);
    check(&mut [0; 665][..], &mut [0; 665][..]);
    check(AtomicStorage::<665>::new(), AtomicStorage::<665>::new());
}

#[test]
fn invalid_history_uploads_black_even_when_framebuffer_flags_are_clean() {
    let mut framebuffer = FrameBuffer::<17, 19, 323>::new();
    framebuffer.dirty_tiles.fill(false);
    let mut history = DisplayState::new([0; 323]);
    let mut display = display();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[5], vec![0; 323 * 2]);
    display.spi.writes.clear();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    assert!(display.spi.writes.is_empty());

    history.invalidate();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
}

#[test]
fn failed_diff_then_reverted_pixels_still_require_upload() {
    let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
    let mut history = DisplayState::new([0; 1024]);
    let mut display = display();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    display.spi.writes.clear();

    framebuffer
        .draw_iter([Pixel(Point::zero(), Rgb565::RED)])
        .unwrap();
    display.spi.fail_at = Some(5);
    assert_eq!(
        run(framebuffer.draw_diff(&mut display, &mut history)),
        Err(())
    );
    framebuffer
        .draw_iter([Pixel(Point::zero(), Rgb565::BLACK)])
        .unwrap();
    display.spi.fail_at = None;
    display.spi.writes.clear();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[5], vec![0; 16 * 16 * 2]);
}

#[test]
fn cancelled_diff_retries_completed_tiles_even_after_snapshot_clears_flags() {
    let mut framebuffer = FrameBuffer::<64, 32, 2048>::new();
    let mut history = DisplayState::new([0; 2048]);
    let mut display = display();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    display.spi.writes.clear();
    framebuffer
        .draw_iter([
            Pixel(Point::zero(), Rgb565::RED),
            Pixel(Point::new(48, 0), Rgb565::BLUE),
        ])
        .unwrap();
    display.spi.pending_at = Some(11);
    {
        let mut future = std::pin::pin!(framebuffer.draw_diff(&mut display, &mut history));
        assert!(future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_pending());
    }
    let mut snapshot = [0; 2048];
    framebuffer.take_dirty_rows(&mut snapshot).unwrap();
    framebuffer
        .draw_iter([
            Pixel(Point::zero(), Rgb565::BLACK),
            Pixel(Point::new(48, 0), Rgb565::BLACK),
        ])
        .unwrap();
    framebuffer.take_dirty_rows(&mut snapshot).unwrap();
    assert!(!framebuffer.dirty_tiles.any());

    display.spi.pending_at = None;
    display.spi.writes.clear();
    run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
    assert_eq!(display.spi.writes.len(), 12);
    assert!(display.spi.writes[5].iter().all(|byte| *byte == 0));
    assert!(display.spi.writes[11].iter().all(|byte| *byte == 0));
}

#[test]
fn valid_history_detects_changes_after_snapshot_capture_without_upload() {
    fn check(storage: impl PixelStorage, history_storage: impl PixelStorage) {
        let mut framebuffer = FrameBuffer::<19, 35, 665, _>::with_storage(storage);
        let mut history = DisplayState::new(history_storage);
        let mut display = display();
        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        display.spi.writes.clear();

        framebuffer
            .draw_iter([Pixel(Point::new(18, 34), Rgb565::RED)])
            .unwrap();
        let mut snapshot = [0; 665];
        assert_eq!(framebuffer.take_dirty_rows(&mut snapshot), Ok(Some(32..35)));
        assert!(!framebuffer.dirty_tiles.any());

        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        assert_eq!(display.spi.writes.len(), 6);
        assert_panel_matches(&display.spi.writes, 19, 35, &snapshot);
        run(framebuffer.draw_diff(&mut display, &mut history)).unwrap();
        assert_eq!(display.spi.writes.len(), 6);
    }

    check([0; 665], [0; 665]);
    check(AtomicStorage::<665>::new(), AtomicStorage::<665>::new());
}

#[test]
#[should_panic(expected = "Display history storage length mismatch")]
fn history_rejects_wrong_storage_length() {
    let _ = DisplayState::<16, 16, 256, _>::new([0; 255]);
}
