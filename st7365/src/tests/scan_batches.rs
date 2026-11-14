use super::*;

fn batch_config(max_tiles: usize) -> ScanBatchConfig {
    ScanBatchConfig {
        max_tiles,
        write_budget_lines: 9,
    }
}

#[test]
fn scheduler_uploads_safe_tile_before_waiting_on_first_dirty_tile() {
    let mut framebuffer = FrameBuffer::<16, 32, 512>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let mut display = display();
    display.spi.scanlines = [Ok(10), Ok(11)].into();
    let config = ScanSyncConfig {
        max_polls: 2,
        ..scan_config()
    };
    assert_eq!(
        run(framebuffer.draw_sync(&mut display, &config)),
        Err(DrawSyncError::Timeout)
    );
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[3], [0, 16, 0, 31]);
    assert!(framebuffer.dirty_tiles[0]);
    assert!(!framebuffer.dirty_tiles[1]);
}

#[test]
fn switched_candidate_requires_fresh_safe_scan_after_staging() {
    let mut framebuffer = FrameBuffer::<16, 32, 512>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let mut display = display();
    display.spi.scanlines = [10, 25, 26].map(Ok).into();
    let config = ScanSyncConfig {
        max_polls: 3,
        ..scan_config()
    };
    assert_eq!(
        run(framebuffer.draw_sync(&mut display, &config)),
        Err(DrawSyncError::Timeout)
    );
    assert_eq!(display.spi.writes.len(), 6);
    assert_eq!(display.spi.writes[3], [0, 0, 0, 15]);
    assert_eq!(display.spi.reads_at_write[5], 3);
    assert!(!framebuffer.dirty_tiles[0]);
    assert!(framebuffer.dirty_tiles[1]);
}

#[test]
fn synchronized_packets_merge_perpendicular_to_all_scan_directions() {
    for direction in [
        ScanDirection::TopToBottom,
        ScanDirection::BottomToTop,
        ScanDirection::LeftToRight,
        ScanDirection::RightToLeft,
    ] {
        let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [60, 61, 62, 63].map(Ok).into();
        let config = ScanSyncConfig {
            direction,
            ..scan_config()
        };
        run(framebuffer.draw_sync_batched(&mut display, &config, &batch_config(2))).unwrap();
        assert_eq!(display.spi.writes.len(), 10);
        assert_eq!(display.spi.reads, 4);
        match direction {
            ScanDirection::TopToBottom | ScanDirection::BottomToTop => {
                assert_eq!(display.spi.writes[1], [0, 0, 0, 31]);
                assert_eq!(display.spi.writes[3], [0, 0, 0, 15]);
            }
            ScanDirection::LeftToRight | ScanDirection::RightToLeft => {
                assert_eq!(display.spi.writes[1], [0, 0, 0, 15]);
                assert_eq!(display.spi.writes[3], [0, 0, 0, 31]);
            }
        }
        assert_panel_matches(&display.spi.writes, 32, 32, &framebuffer.buffer);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn rectangular_packets_exclude_entire_scan_interval_and_transfer_deadline() {
    for direction in [
        ScanDirection::TopToBottom,
        ScanDirection::BottomToTop,
        ScanDirection::LeftToRight,
        ScanDirection::RightToLeft,
    ] {
        let vertical = matches!(
            direction,
            ScanDirection::LeftToRight | ScanDirection::RightToLeft
        );
        for scanline in 0..80 {
            let mut framebuffer = FrameBuffer::<17, 19, 323>::new();
            framebuffer.dirty_tiles.fill(false);
            framebuffer.dirty_tiles.set(0, true);
            framebuffer
                .dirty_tiles
                .set(if vertical { 2 } else { 1 }, true);
            let mut display = display();
            display.spi.scanlines = [Ok((scanline + 79) % 80), Ok(scanline)].into();
            let config = ScanSyncConfig {
                direction,
                max_polls: 2,
                ..scan_config()
            };
            let batches = batch_config(2);
            let mut occupied = [false; 80];
            for row in 0..if vertical { 19 } else { 16 } {
                for column in 0..if vertical { 16 } else { 17 } {
                    let physical = 7 + match direction {
                        ScanDirection::TopToBottom => row,
                        ScanDirection::BottomToTop => 18 - row,
                        ScanDirection::LeftToRight => column,
                        ScanDirection::RightToLeft => 16 - column,
                    };
                    occupied[physical] = true;
                }
            }
            let safe = (0..=batches.write_budget_lines + 1)
                .all(|advance| !occupied[((scanline + advance) % 80) as usize]);
            assert_eq!(
                run(framebuffer.draw_sync_batched(&mut display, &config, &batches)),
                if safe {
                    Ok(())
                } else {
                    Err(DrawSyncError::Timeout)
                },
                "direction={direction:?}, scanline={scanline}"
            );
            assert_eq!(display.spi.writes.len(), if safe { 6 } else { 0 });
            assert_eq!(
                framebuffer.dirty_tiles.count_ones(),
                if safe { 0 } else { 2 }
            );
        }
    }
}

#[test]
fn synchronized_packets_clip_edges_and_support_non_slice_storage() {
    for direction in [ScanDirection::TopToBottom, ScanDirection::LeftToRight] {
        let mut framebuffer =
            FrameBuffer::<17, 19, 323, _>::with_storage(AtomicStorage::<323>::new());
        framebuffer.clear(Rgb565::BLUE).unwrap();
        let mut display = display();
        display.spi.scanlines = [60, 61, 62, 63].map(Ok).into();
        let config = ScanSyncConfig {
            direction,
            ..scan_config()
        };
        run(framebuffer.draw_sync_batched(&mut display, &config, &batch_config(20))).unwrap();
        assert_eq!(display.spi.writes.len(), 10);
        assert_panel_matches(&display.spi.writes, 17, 19, &[0x001f_u16.swap_bytes(); 323]);
    }
}

#[test]
fn synchronized_packets_respect_cap_and_do_not_cross_clean_gaps() {
    let mut framebuffer = FrameBuffer::<320, 16, 5120>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let mut display = display();
    display.spi.scanlines = (40..54).map(Ok).collect();
    run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batch_config(3))).unwrap();
    assert_eq!(display.spi.writes.len(), 30);
    assert!(packets(&display.spi.writes)
        .iter()
        .all(|packet| packet.4.len() <= 3 * 512));
    assert_panel_matches(&display.spi.writes, 320, 16, &framebuffer.buffer);

    let mut framebuffer = FrameBuffer::<64, 16, 1024>::new();
    framebuffer.dirty_tiles.fill(false);
    framebuffer
        .draw_iter([
            Pixel(Point::new(0, 0), Rgb565::RED),
            Pixel(Point::new(32, 0), Rgb565::RED),
            Pixel(Point::new(48, 0), Rgb565::RED),
        ])
        .unwrap();
    let mut display = super::display();
    display.spi.scanlines = [60, 61, 62, 63].map(Ok).into();
    run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batch_config(4))).unwrap();
    assert_eq!(display.spi.writes.len(), 10);
    assert_eq!(display.spi.writes[1], [0, 0, 0, 15]);
    assert_eq!(display.spi.writes[7], [0, 32, 0, 63]);
    assert_panel_matches(&display.spi.writes, 64, 16, &framebuffer.buffer);
}

#[test]
fn synchronized_packet_failure_and_cancellation_preserve_whole_packet() {
    for fail_at in 0..6 {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [60, 61].map(Ok).into();
        display.spi.fail_at = Some(fail_at);
        assert_eq!(
            run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batch_config(2))),
            Err(DrawSyncError::Interface)
        );
        assert_eq!(framebuffer.dirty_tiles.count_ones(), 2);
        display.spi.fail_at = None;
        display.spi_mut().writes.clear();
        display.spi.scanlines = [60, 61].map(Ok).into();
        run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batch_config(2))).unwrap();
        assert_panel_matches(&display.spi.writes, 32, 16, &framebuffer.buffer);
    }
    for pending_read in [false, true] {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = [60, 61].map(Ok).into();
        if pending_read {
            display.spi.pending_read_at = Some(1);
        } else {
            display.spi.pending_at = Some(5);
        }
        let config = scan_config();
        let batches = batch_config(2);
        {
            let mut future =
                std::pin::pin!(framebuffer.draw_sync_batched(&mut display, &config, &batches));
            assert!(future
                .as_mut()
                .poll(&mut Context::from_waker(Waker::noop()))
                .is_pending());
        }
        assert_eq!(framebuffer.dirty_tiles.count_ones(), 2);
        display.spi.pending_read_at = None;
        display.spi.pending_at = None;
        display.spi_mut().writes.clear();
        display.spi.scanlines = [60, 61].map(Ok).into();
        run(framebuffer.draw_sync_batched(&mut display, &config, &batches)).unwrap();
        assert_panel_matches(&display.spi.writes, 32, 16, &framebuffer.buffer);
        assert!(!framebuffer.dirty_tiles.any());
    }
}

#[test]
fn synchronized_packet_limits_fail_before_io() {
    for batches in [
        batch_config(0),
        batch_config(21),
        ScanBatchConfig {
            max_tiles: 2,
            write_budget_lines: 4,
        },
        ScanBatchConfig {
            max_tiles: 2,
            write_budget_lines: 63,
        },
    ] {
        let mut framebuffer = FrameBuffer::<32, 16, 512>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        assert_eq!(
            run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batches)),
            Err(DrawSyncError::InvalidConfig)
        );
        assert!(display.spi.writes.is_empty());
        assert_eq!(display.spi.reads, 0);
        framebuffer.dirty_tiles.fill(false);
        run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batches)).unwrap();
    }
}

#[test]
fn batched_scheduler_bypasses_unsafe_packets_in_reversed_scan_orders() {
    for direction in [
        ScanDirection::TopToBottom,
        ScanDirection::BottomToTop,
        ScanDirection::LeftToRight,
        ScanDirection::RightToLeft,
    ] {
        let reversed = matches!(
            direction,
            ScanDirection::BottomToTop | ScanDirection::RightToLeft
        );
        let vertical = matches!(
            direction,
            ScanDirection::LeftToRight | ScanDirection::RightToLeft
        );
        let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
        framebuffer.clear(Rgb565::RED).unwrap();
        let mut display = display();
        display.spi.scanlines = if reversed { [25, 26] } else { [10, 11] }.map(Ok).into();
        let config = ScanSyncConfig {
            direction,
            max_polls: 2,
            ..scan_config()
        };
        assert_eq!(
            run(framebuffer.draw_sync_batched(&mut display, &config, &batch_config(2))),
            Err(DrawSyncError::Timeout)
        );
        assert_eq!(display.spi.writes.len(), 6);
        assert_eq!(framebuffer.dirty_tiles.count_ones(), 2);
        assert!(framebuffer.dirty_tiles[0]);
        assert!(!framebuffer.dirty_tiles[if vertical { 1 } else { 2 }]);
        if vertical {
            assert_eq!(display.spi.writes[1], [0, 16, 0, 31]);
        } else {
            assert_eq!(display.spi.writes[3], [0, 16, 0, 31]);
        }
    }
}

#[test]
fn batched_scan_uses_read_clock_for_samples_and_write_clock_for_packets() {
    let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let mut clocks = Vec::new();
    let spi = RecordingSpi {
        scanlines: [60, 61, 62, 63].map(Ok).into(),
        ..RecordingSpi::default()
    };
    let spi = crate::spi::ReadWriteSpi::new(spi, |_: &mut RecordingSpi, reads| {
        clocks.push(reads);
        Ok(())
    });
    let mut display = ST7365P::new(spi, Pin, None::<Pin>, true, false, Delay::default());
    run(framebuffer.draw_sync_batched(&mut display, &scan_config(), &batch_config(2))).unwrap();
    assert_panel_matches(
        &display.spi.device_mut().writes,
        32,
        32,
        &framebuffer.buffer,
    );
    drop(display);
    assert_eq!(clocks, [true, false].repeat(2));
}

#[test]
fn cancellation_after_completed_packet_retains_only_remaining_packet() {
    let mut framebuffer = FrameBuffer::<32, 32, 1024>::new();
    framebuffer.clear(Rgb565::RED).unwrap();
    let mut display = display();
    display.spi.scanlines = [60, 61, 62, 63].map(Ok).into();
    display.spi.pending_at = Some(9);
    let config = scan_config();
    let batches = batch_config(2);
    {
        let mut future =
            std::pin::pin!(framebuffer.draw_sync_batched(&mut display, &config, &batches));
        assert!(future
            .as_mut()
            .poll(&mut Context::from_waker(Waker::noop()))
            .is_pending());
    }
    assert!(!framebuffer.dirty_tiles[0]);
    assert!(!framebuffer.dirty_tiles[1]);
    assert!(framebuffer.dirty_tiles[2]);
    assert!(framebuffer.dirty_tiles[3]);
    display.spi.pending_at = None;
    display.spi.writes.truncate(8);
    display.spi.scanlines = [60, 61].map(Ok).into();
    run(framebuffer.draw_sync_batched(&mut display, &config, &batches)).unwrap();
    assert_panel_matches(&display.spi.writes, 32, 32, &framebuffer.buffer);
    assert!(!framebuffer.dirty_tiles.any());
}
