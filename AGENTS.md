# AGENTS.md — lofi-camera

ESP32-S3-CAM firmware (PlatformIO + Arduino framework) in `src/`, plus a host-side
Python film-look lab in `tools/film_lab/`. `README.md` is the source of truth for
hardware, behaviour and every tunable macro — read the relevant section before
changing firmware.

## Commands

- Build: `pio run` — expect ~72 % RAM, ~16 % flash. The dual-core second ring
  and sharpen halo account for most of the RAM; a further large jump means
  something got linked that should not have been.
- Upload: `pio run --target upload --upload-port COM<n>` — native USB-C
  (USB-Serial-JTAG); no peripherals need disconnecting.
- Serial monitor works over USB CDC; `DBG()` is always compiled in.
- Film lab: `.venv\Scripts\python.exe tools\film_lab\run_film_lab.py`, then open
  `tools/film_lab/out/index.html`.
- There is no firmware unit-test suite. The automated checks are `pio run` and
  `tools/jpeg_filter_test/run_filter_test.py` (host compile + matrix); a
  behaviour change still needs an on-device sanity check, so say explicitly
  what was not verified on hardware.

## Hardware invariants (never break)

- Camera: 4,5 = SCCB; 6,7 = VSYNC/HREF; 8,9,10,11,12,13 = data + PCLK;
  15,16,17,18 = XCLK + D5-D7. 38,39,40 = SD (1-bit, D1-D3 unconnected);
  19,20 = USB; 26-37 = in-package flash + octal PSRAM; 43,44 = UART0;
  45,46 = strapping. 0 = BOOT/shutter, 2 = status LED, 3 = flash LEDs.
  Never assign these to a peripheral.
- The camera has no PWDN or sensor reset GPIO (`CAM_RST` is only pulled up):
  keep both at -1.
- GPIO41/42 are the OLED I2C pins and the JTAG MTDI/MTMS pins; external JTAG
  is unavailable (USB-Serial-JTAG still works).
- The SD slot wires DAT0 only and has no card-detect switch: keep the 1-bit
  mount and the filesystem liveness probe.
- The SD card is mandatory at boot and at runtime. The `NO SD CARD` wait loop
  must stay re-entrant so a reinserted card resumes without a power cycle.

## Architecture rules

- One camera stream (SXGA JPEG) serves both the viewfinder and the stills. Never
  re-init the sensor to take a photo: AWB/AEC settle is what produced the
  washed-out shots, and it costs ~1.1 s per capture.
- Keep preview and capture independent: `PREVIEW_MODE` only affects the OLED,
  the runtime look/border selection only affects the SD file. Neither may
  change the other's output.
- SD writes stream through `JpegWriteFn`; keep the encoded JPEG out of RAM
  (peak PSRAM is ~4.8 MB of 8 MB with the RGB888 canvas and depends on it).
- When `EXIF_ENABLED` is set (default `1`), saved JPEGs carry an APP1 EXIF
  `ImageDescription` inserted by the SD sink:
  look, border/inset, source/output dimensions, re-encode quality and
  per-stage milliseconds (`fb_ms`, `dec_ms`, `stats_ms`, `look_ms`,
  `sharp_ms`, `enc_ms`, `total_ms`). Stage timings are always collected;
  `CAPTURE_TIMING` only controls the optional diagnostic log/selftest. The image
  data is untouched, but `NONE` output is no longer byte-identical at the file
  level. Keep these fields in sync when a pipeline stage or look is added.
- No per-frame or per-shot heap allocation. Use `static`/file-scope buffers
  (`PreviewFrame`, the once-allocated `previewRgb`). The dual-core worker task
  and its semaphores are allocated once, not per shot.
- Dual-core split: stats, look and sharpen run on both cores when
  `JPEG_FILTER_DUALCORE` is `1` (default). The worker is one persistent core-0
  task, never one per shot. Sharpen rewrites the canvas in place, so its two
  bands must read the 2R boundary rows from the halo copy; wider kernels
  (rad > 3) keep the single-core sharpen. Decode and encode stay single-core.

## Conventions

- Modules are header-only and pulled in by `preview.h` / `main.cpp`. Match that;
  only add a `.cpp` when unavoidable. The one exception is `src/usm_simd.S`, the
  S3 PIE dot-product kernel behind the shared sharpen.
- Every tunable is an `#ifndef MACRO` / `#define MACRO default` pair so
  `build_flags` (`-D...`) can override it without editing code.
- Invalid compile-time selections must fail the build (`#error`), never silently
  fall back to a default.
- `DBG(...)` for all logging; fixed-size stack buffers with `snprintf` for
  strings, never `String`.
- Comment *why*, not just *what* — most of the constraints above are non-obvious
  hardware or memory facts. Keep the existing comment density.

## Extension points

New preview mode: add `src/preview_<name>.h` exposing
`void <name>Draw(Adafruit_SSD1306&, const PreviewFrame&)`, register its id and
`#error` guard in `src/preview.h`, dispatch it from `previewDraw()`. A renderer
only maps `PreviewFrame::px` (already aspect-fit, letterboxed and grayscale) onto
the panel — it must not clear the display or call `display()`.

New JPEG look: add a `FilterLook` value, a `FilmLookParams` entry in
`FILM_LOOKS`, and a name in `jpegFilterLookName()` in `src/jpeg_filter.h`. The
shared pipeline and border stage apply automatically; keep the output streaming
(never buffer the JPEG) and keep `jpegFilterWorstCaseBytes()` honest, because it
backs the pre-write free-space check.

## Porting film looks

`tools/film_lab/` is the design bench for new looks; a grade is ported to
firmware only after it is chosen there. When a look is ported to C++,
implement the shared chain once, not one algorithm per preset. Keep the lab
runnable and firmware-free — it must never edit `src/`.

## Do not

- Change a `build_flags` or macro default as a side effect of an unrelated change.
- Leave a truncated JPEG on the card or burn a filename slot on a failed write.
- Replace "retry forever with backoff" with a halt: the unit has no console, so
  OLED messages plus self-healing **are** the recovery mechanism.
- Commit `test-images/`, `.venv/`, `.pio/`, `/out/`, `tools/film_lab/out/`,
  `tools/jpeg_filter_test/out/`, or the generated `.vscode` files.

## Host tools

`tools/film_lab/` uses `uv` (`uv venv .venv --python 3.10`) with `numpy` +
`pillow`. Output goes to `tools/film_lab/out/`, which is git-ignored. Keep the
CLI (`--list`, `--filters`, `--family`, `--input`, `--thumb-width`) working.
