# AGENTS.md — lofi-camera

ESP32-CAM firmware (PlatformIO + Arduino framework) in `src/`, plus a host-side
Python film-look lab in `tools/film_lab/`. `README.md` is the source of truth for
hardware, behaviour and every tunable macro — read the relevant section before
changing firmware.

## Commands

- Build: `pio run` — expect ~23 % RAM, ~14.7 % flash; a large jump means
  something got linked that should not have been.
- Upload: `pio run --target upload --upload-port COM<n>` — OLED SCL must be
  disconnected first (see the README upload checklist).
- Serial monitor only works in builds where GPIO1 is not OLED SCL.
- Film lab: `.venv\Scripts\python.exe tools\film_lab\run_film_lab.py`, then open
  `tools/film_lab/out/index.html`.
- There is no test suite. A clean `pio run` is the only automated check;
  behaviour changes need an on-device sanity check, so say explicitly what was
  not verified on hardware.

## Hardware invariants (never break)

- GPIO 0,5,18,19,21,22,23,25,26,27,32,34,35,36,39 = camera; 2,14,15 = SD (1-bit);
  16 = PSRAM CS; 33 = red LED only; 12 = MTDI strapping (must be LOW at boot);
  4 = flash LED. Never assign these to a peripheral.
- GPIO1 is both UART TX and OLED SCL. Never add `Serial` output while the OLED
  owns GPIO1 — use `DBG()`, which compiles itself out when logging is disabled.
- The SD card is mandatory at boot and at runtime. The `NO SD CARD` wait loop
  must stay re-entrant so a reinserted card resumes without a power cycle.

## Architecture rules

- One camera stream (SXGA JPEG) serves both the viewfinder and the stills. Never
  re-init the sensor to take a photo: AWB/AEC settle is what produced the
  washed-out shots, and it costs ~1.1 s per capture.
- Keep preview and capture independent: `PREVIEW_MODE` only affects the OLED,
  `JPEG_FILTER` only affects the SD file. Neither may change the other's output.
- SD writes stream through `JpegWriteFn`; keep the encoded JPEG out of RAM
  (peak PSRAM is ~2.9 MB of 4 MB and depends on it).
- No per-frame or per-shot heap allocation. Use `static`/file-scope buffers
  (`PreviewFrame`, the once-allocated `previewRgb`).

## Conventions

- Modules are header-only and pulled in by `preview.h` / `main.cpp`. Match that;
  only add a `.cpp` when unavoidable.
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

New JPEG filter: add a `JPEG_FILTER_*` id and an `#elif` branch in
`src/jpeg_filter.h` implementing `jpegFilterStream()` (push bytes to the sink,
never buffer the JPEG) plus an honest `jpegFilterWorstCaseBytes()` bound, which
backs the pre-write free-space check.

## Porting film looks

`tools/film_lab/` is the design bench and nothing in it is on the device yet.
When a look is ported to C++, implement the shared chain once, not one algorithm
per preset. Keep the lab runnable and firmware-free — it must never edit `src/`.

## Do not

- Change a `build_flags` or macro default as a side effect of an unrelated change.
- Leave a truncated JPEG on the card or burn a filename slot on a failed write.
- Replace "retry forever with backoff" with a halt: the unit has no console, so
  OLED messages plus self-healing **are** the recovery mechanism.
- Commit `test-images/`, `.venv/`, `.pio/`, `tools/film_lab/out/`, or the
  generated `.vscode` files.

## Host tools

`tools/film_lab/` uses `uv` (`uv venv .venv --python 3.10`) with `numpy` +
`pillow`. Output goes to `tools/film_lab/out/`, which is git-ignored. Keep the
CLI (`--list`, `--filters`, `--family`, `--input`, `--thumb-width`) working.
