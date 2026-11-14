# ST7365P-lcd-rs

This is a Rust library for displays using the ST7365P driver with embedded_graphics, embedded_hal, and no_std, no_alloc support.

Two versions:
- Synchrouous (which is in master branch)
- Asynchronous `branch = "async"`

## Scan synchronization (async branch)

`display.wait_for_vsync(max_polls).await` polls **GSCAN (0x45)** until
the physical scanline counter wraps. Call it immediately before a framebuffer
flush, once per update, not once per tile. It does not require TE output to be
enabled. `read_scanline()` also exposes the counter for diagnostics.

The old experimental read split the command and response across two SPI
transactions, deasserting CS between them. Reads now keep CS asserted throughout,
with D/C low, and decode two MSB-first serial bytes. The dummy read in the parallel
command table is not an extra serial byte. The extra serial dummy clock shown for
RDDID/RDDST is specific to those commands, not GSCAN.

**SPI reads need a slower clock than writes.** The datasheet specifies a minimum
150 ns read clock period (6.66 MHz maximum) and 60 ns high/low times. Use 6 MHz or
less. `SpiDevice` has no portable clock configuration API. The example wraps its
Embassy `ExclusiveDevice` in `spi::ReadWriteSpi`, selecting clocks before each
direction change using the HAL's existing configuration API:

```rust,ignore
use st7365p_lcd::spi::ReadWriteSpi;

let device = ReadWriteSpi::new(spi_device, move |device, reads| {
    config.frequency = if reads { 6_000_000 } else { 32_000_000 };
    device.bus_mut().set_config(&config);
    Ok(())
});
let mut display = ST7365P::new(device, dc, rst, false, true, Delay);
```

The GSCAN command and response stay in one read-speed transaction. Writes select
the write clock, including after an error. Successful consecutive transactions
reuse their configuration. Errors, dropped futures, and `device_mut()` invalidate
the clock cache. The callback requires an exclusively
owned idle bus; it cannot reconfigure a shared bus outside its lock. After
cancellation, recover CS/DMA if the underlying transport requires it before
starting another transaction. Without this wrapper, keep synchronized drawing
at a read-safe clock, or restore clocks explicitly when manually switching.
The PicoCalc example logs failures and falls back to unsynchronized drawing
rather than hanging. It logs render, sync-wait, and flush durations separately;
these measurements do not substitute for worst-case packet timing calibration.
The existing 32 MHz write clock is empirical, not a datasheet-rated guarantee
(the documented write period is at least 66 ns).
With the example's default 150 MHz peripheral clock, that request rounds down to
25 MHz. The example's opt-in `fast-spi` feature requests an exact 37.5 MHz divider:
`cargo +nightly build -p picocalc-rp2350 --release --target thumbv8m.main-none-eabihf --features fast-spi`.
**37.5 MHz is an unverified overclock.** Validate your panel and wiring before
using it; the default request remains unchanged. No hardware speedup is claimed.

Write setup guards now use `Operation::DelayNs(1)` in the SPI transaction rather
than a separate driver timer await. The SPI device must support delay operations;
do not use a `new_no_delay` device. The PicoCalc example's `SpiDelay` busy-waits
for delays up to 1 us, avoiding executor round trips while retaining the requested
minimum delay. Longer delays still await the timer. Other SPI devices may still
yield for their delays; measure your HAL rather than assuming zero overhead.

- Stock PicoCalc V2.0 connects LCD SDO to GP12, but leaves TE unconnected. Use
  scanline polling there, not a made-up GPIO for TE. The panel must be awake.
- Polls are spaced by at least 50 us, plus SPI overhead. A 2,000-read budget
  allows roughly 100 ms of polling delays. Constant zero or all-one responses
  time out; check MISO wiring, SPI mode/speed and sleep state. A stalled HAL call
  still needs an application-level wall-clock timeout.
- Scanline zero starts vertical sync. Counts include porches and can exceed the
  visible height. Polling detects a wrap, not an exact zero sample; it can return
  shortly after vertical blanking starts, rather than precisely on its edge.
- Starting in sync does **not** guarantee tear-free uploads: a 320x320 RGB565
  frame takes at least 65.5 ms at the default example's actual 25 MHz, longer
  than a 60 Hz frame (16.7 ms).
  Prefer small dirty updates; matching scan/write order and avoiding the panel
  catching the writer still matter. No automatic waits are added to existing
  drawing methods, and panel refresh settings are left unchanged.

For boards with TE physically connected, `set_tear_scanline(line).await` sends
**STE (0x44)** with a big-endian line number. Line zero selects vertical-only TE,
equivalent to **TEON (0x35), TEM=0**. This programs an output signal, not a delayed
RAM write. It takes effect next frame; await a fresh TE edge in your application
before uploading, with a timeout. `disable_tearing_effect()` sends **TEOFF (0x34)**.
Choose a physical line within the configured panel scan period; rotation does
not convert framebuffer coordinates into TE timing.

References: ST7365P specification V1.0, sections 7.4.3 (SPI timing), 8.4.2
(4-wire serial reads), 9.2.35/9.2.36 (STE/GSCAN), and 10.8 (tearing examples),
plus `clockwork_Mainboard_V2.0_Schematic.pdf` in `clockworkpi/PicoCalc`.
Host checks cover command framing and polling behavior; panel timing still needs
verification on real hardware.

Run host regression checks with `cargo test -p st7365p-lcd --test scan_sync`.

## Drawing and flushing

`FrameBuffer<W, H, SIZE>` uses `DefaultPixelStorage<SIZE>` (an owned `[u16; SIZE]`)
by default; `FrameBuffer::new()` and `framebuffer!(W, H)` remain const-compatible.
Use `FrameBuffer::<W, H, SIZE, _>::with_storage(&mut pixels[..])` to borrow an
existing pixel allocation instead. Both constructors start entirely dirty:
initial framebuffer contents are not assumed to match the panel. The first
partial flush therefore initializes every visible pixel, including black pixels.

Both backends share clipping, drawing, dirty tracking and every flush method.
Arrays and mutable slices implement `PixelStorage` through `AsRef<[u16]>` and
`AsMut<[u16]>`, including bulk fill/copy and zero-copy transfers. Custom backends
implement `len`, `read`, and `write`; default bulk operations use those methods.
`as_slice` is optional: without it, transfers copy pixels into bounded staging
buffers. Stored words are byte-swapped RGB565, matching the existing buffered
transport; `DrawTarget` handles this conversion for callers.

Do not expose a non-atomic slice over concurrently modified atomic storage.
Storage abstraction does not synchronize dirty tracking or support external
pixel writes: keep all drawing and flushing serialized through the framebuffer.
An atomic backend still needs a separate publication protocol before concurrent
writers can be supported.

All `DrawTarget` methods, including `fill_solid` and `clear`, dirty only tiles
containing changed pixels. Repeating an identical fill after a successful flush
does no SPI work. Identical writes never clear an already pending dirty flag.
Solid fills retain bulk writes within changed tile-row segments.

`partial_draw_batched` merges horizontal dirty runs and extends matching spans
vertically. Staged rectangles fit the existing 5,120-pixel scratch buffer;
full-width contiguous bands borrow framebuffer slices without copying. A
full-frame shortcut applies only when every tile is dirty and storage exposes
a slice. Non-slice storage retains batching even for dense updates. The old 80%
threshold is removed: clean pixels are not uploaded merely to save windows.
Tiles become clean only after a successful transfer; errors or dropped
flush futures leave unsent tiles dirty for retry. Edge tiles use clipped windows
and matching payload lengths. Controller delays remain unchanged; batched address
parameters reduce the number of writes and waits instead.
Explicit `draw()` marks the entire frame dirty before starting, so a failed or
cancelled full refresh can be retried through a partial flush, even when the
framebuffer was previously clean.

Run host regression tests with `cargo test -p st7365p-lcd --lib --tests` (also with
`--release`). Actual draw times still require measurements on the target hardware.

### Exact final-frame comparison

Clearing and redrawing a scene can change pixels temporarily even when its final
image matches the panel. For exact suppression, opt into `draw_diff` with
caller-owned `DisplayState` storage:

```rust,ignore
use st7365p_lcd::DisplayState;

let mut previous_pixels = [0u16; WIDTH * HEIGHT];
let mut history = DisplayState::<WIDTH, HEIGHT, { WIDTH * HEIGHT }, _>::new(
    &mut previous_pixels[..],
);
framebuffer.draw_diff(&mut display, &mut history).await?;
```

History starts invalid, so the first call uploads the whole frame. Later calls
compare final pixels exactly, without hashes or collision risk, even if snapshot
capture cleared dirty flags. This reads the whole framebuffer per call and needs
one extra RGB565 image supplied by the caller; ordinary flush APIs have no extra
frame-history cost. History updates only after complete success. Failed or
cancelled transfers invalidate attempted tiles, so reverting pixels before retry
cannot hide a partially transmitted update.

Keep one history per framebuffer/display pair. Call `history.invalidate()` after
panel reset, orientation/offset changes, other upload APIs, direct display writes,
or switching framebuffer instances. `draw_diff` uses unsynchronized batching;
it does not make frame presentation atomic.

### Snapshot handoff

`take_dirty_rows` copies a full-width band into caller storage and clears captured
dirty flags so drawing can continue while a separate snapshot is uploaded.
Keep that snapshot immutable while it is in flight. Call `retry_rows` after a
failed **or cancelled** upload; it preserves newer drawing. A captured snapshot
is not proof of a successful panel update and must not be treated as valid
display history. This handoff is separate from rectangle batching.

### Scan-aware partial updates

Choose the flush API according to latency and tearing requirements:

- `draw`: sends the entire framebuffer immediately, without scan synchronization.
- `partial_draw`: deprecated unbatched path, retained for compatibility and its
  smaller 512-byte scratch footprint.
- `partial_draw_batched`: batches dirty rectangles for throughput, preserving
  clean tiles and borrowing full-width bands where possible.
- `draw_diff`: suppresses unchanged final tiles using caller-owned display history.
- `draw_diff_dirty`: compares only dirty tiles and invalid history. Use exclusively
  with the same framebuffer/history/display pair; do not capture snapshots or
  flush through other APIs between calls. After doing so, use `draw_diff` once
  or invalidate history before resuming dirty-only comparisons.
- `draw_sync`: sends only dirty tiles, checking GSCAN separately before each
  tile. If a tile is unsafe, it searches for another safe dirty tile before
  waiting. Staging finishes before the final validating scan sample. This trades throughput and latency
  for reduced local tearing; it never silently falls back to unsynchronized
  drawing, even for an all-dirty frame.
- `draw_sync_batched`: applies the same checks to bounded rectangular packets
  with an explicitly measured packet budget, never an unsynchronized fallback.

For repeated batched flushes, reuse initialized scratch instead of zeroing 10 KiB
on each call. `partial_draw_batched_with_scratch(&mut display, &mut scratch)` and
`draw_sync_batched_with_scratch(&mut display, &sync, &batches, &mut scratch)` accept
`let mut scratch = [0u16; 5120];` allocated once outside your render loop. Neither
method reads unstaged scratch or sends padding. Ordinary methods retain their
existing convenience allocation; no uninitialized memory is introduced.

Direct `write_pixels`/`set_pixels` calls accept native RGB565 words and now send
up to 64 pixels per transaction. Buffered methods still accept pre-swapped words
and borrow them without copying. `set_pixel` remains a single-pixel operation.
Unchanged CASET/RASET axes are cached; reset, orientation, offset changes, and
`spi_mut()` invalidate the relevant cache. Interrupted address commands cannot
leave their old cached axis valid. Invalid windows or offset overflow fail before
I/O. Bulk contiguous drawing marks dirty state once per changed tile-row span;
history commits use `PixelStorage::write_from`, with a slice-copy fast path and
a default per-word implementation for custom storage.

`draw_sync` needs a `ScanSyncConfig` calibrated for your panel and workload.
There are deliberately no guessed refresh-rate, porch, or orientation defaults.
`frame_scanlines` is the counter's full period, not the framebuffer height;
`first_scanline` is the first physical line occupied by this framebuffer,
including porches and display offsets. `direction` describes physical scanning
in framebuffer coordinates after rotation. Recalibrate when changing display
orientation, offsets, refresh settings, SPI speed, or executor load. The mapping
must be contiguous: wrapped hardware scrolling is not supported.

```rust,ignore
use st7365p_lcd::{DrawSyncError, ScanDirection, ScanSyncConfig};

let sync = ScanSyncConfig {
    frame_scanlines: measured_frame_lines,
    first_scanline: measured_first_visible_line,
    direction: ScanDirection::TopToBottom, // Use your verified physical direction.
    write_budget_lines: measured_worst_case_tile_lines + safety_margin_lines,
    max_polls: 2_000,
};

let result = framebuffer.draw_sync(&mut display, &sync).await;

match result {
    Ok(()) => {}
    Err(DrawSyncError::Timeout) => { /* Retry remaining dirty tiles later. */ }
    Err(error) => return Err(error),
}
```

**Timing budget matters.** `write_budget_lines` is a nonzero upper bound on
elapsed scanline periods from the accepted GSCAN sample until the last pixel
of a 16x16 RGB565 tile has arrived. Include the remaining read time, address
commands, clock switching, SPI transfer, bus contention, async scheduling delays, and margin;
round up. A tile contains at most 512 pixel bytes. The scheduler reserves one
additional line before the next scan visit. Wire speed alone is not a sufficient
budget, and unbounded executor delays cannot be made tear-free by polling.

Each packet requires at least two reads showing counter movement; stuck values
cannot immediately authorize a write. `max_polls` limits reads across the entire
flush, not per tile. Polls within a tile wait are spaced by at least 50 us. A HAL
call that stalls still needs an application-level timeout. Out-of-range counter
values return `InvalidScanline`; impossible geometry or timing budgets return
`InvalidConfig`; transport failures return `Interface`. No dirty tiles means no
SPI activity. Only successfully uploaded tiles become clean, so failures or
dropped futures preserve unfinished work for retry.

For scan-aware batching, supply a separate limit and budget:

```rust,ignore
use st7365p_lcd::ScanBatchConfig;

let batches = ScanBatchConfig {
    max_tiles: measured_packet_tile_limit,
    write_budget_lines: measured_worst_case_packet_lines + safety_margin_lines,
};
framebuffer.draw_sync_batched(&mut display, &sync, &batches).await?;
```

`max_tiles` must be 1..=20. The packet budget must cover the largest admitted
packet and be at least the single-tile budget. Horizontal strips are selected
for top/bottom physical scanning; vertical strips for left/right scanning.
This preserves the occupied scan interval while increasing payload size.
**Do not reuse an unmeasured single-tile budget for larger packets.** Invalid
geometry, impossible timing budgets, and invalid packet limits fail before I/O.
Single-tile sync uses 512 bytes of pixel scratch; batched sync uses 10 KiB.
Neither API guarantees atomic frame swaps or safety against unbounded delays.
Recalibrate after changing batch limits, SPI clocks, orientation, or executor load.

**Not an atomic frame swap.** Writes may occur before or after the scan passes a
tile, and different tiles may become visible in different refreshes. Large images
can visibly assemble over multiple frames. Accurate timing can reduce tearing
within individual tiles, but cannot guarantee a coherent whole-screen image.
Tests exercise scheduling, all four scan directions, transfer deadlines,
wraparound, clipped tiles, failure, and cancellation. Hardware timing remains
unverified.

![picocalc-example](https://github.com/LegitCamper/ST7365P-lcd-rs/blob/master/assets/picocalc-example.jpg)
