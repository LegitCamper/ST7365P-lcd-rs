# ST7365P-lcd-rs

This is a Rust library for displays using the ST7365P driver with embedded_graphics, embedded_hal, and no_std, no_alloc support.

Two versions:
- Synchrouous (which is in master branch)
- Asynchronous `branch = "async"`

## Drawing and flushing

The `embedded-graphics::DrawTarget` API and framebuffer flush methods are unchanged.
`draw_iter` and `fill_contiguous` dirty only tiles containing changed pixels.
`fill_solid` and `clear` use bulk fills and still mark their affected tiles dirty.

`partial_draw_batched` streams every dirty run, borrows full-width framebuffer
slices without copying, and uses a full-frame transfer when all visible tiles
are dirty. Tiles become clean only after a successful transfer; errors or dropped
flush futures leave unsent tiles dirty for retry. Edge tiles use clipped windows
and matching payload lengths. Controller delays remain unchanged; batched address
parameters reduce the number of writes and waits instead.

Run host regression tests with `cargo test -p st7365p-lcd --lib` (also with
`--release`). Actual draw times still require measurements on the target hardware.

![picocalc-example](https://github.com/LegitCamper/ST7365P-lcd-rs/blob/master/assets/picocalc-example.jpg)
