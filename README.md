# lofi-camera

ESP32-CAM lo-fi viewfinder: live grayscale camera preview on a
128×64 SSD1306 OLED, with button-triggered full-resolution JPEG capture to
microSD.

The host-side [film look lab](tools/film_lab/README.md) prototypes the colour
looks for those JPEGs before any of them are ported to C++.

## Hardware

- ESP32-CAM + ESP32-CAM-MB adapter (CH340 USB serial)
- SSD1306 128×64 OLED (I2C, address `0x3C` / `0x3D`)
- Momentary push button (shutter)
- FAT32-formatted microSD card

## Wiring

Use the silkscreen labels on the ESP32-CAM header.

| Peripheral | ESP32-CAM pin | Notes |
|---|---|---|
| OLED SDA | IO13 (GPIO13) | Software I2C via `Wire` |
| OLED SCL | U0T (GPIO1) | Serial TX line — see caveats |
| OLED VCC | 3V3 | |
| OLED GND | GND | |
| Button | U0R (GPIO3) → GND | Internal pull-up, active LOW |
| microSD | Onboard slot | SD_MMC 1-bit: CLK 14, CMD 15, DATA0 2 |

### Pin constraints (why these pins)

- Camera uses GPIO 0, 5, 18, 19, 21, 22, 23, 25, 26, 27, 32, 34, 35, 36, 39 —
  do not reuse.
- SD 1-bit mode uses GPIO 14 (CLK), 15 (CMD), 2 (DATA0).
- GPIO16 is PSRAM chip-select — using it as GPIO crashes after photos.
- GPIO33 is the internal red LED only, not on the header.
- GPIO12 is the MTDI flash-voltage strapping pin (must be LOW at boot) —
  OLED pull-ups on it prevent boot entirely.
- GPIO4 drives the white flash LED — avoided for clean OLED I2C.
- GPIO1 (U0T) caveats: `Serial.print` debug output is unavailable while the
  OLED is attached, and the OLED SCL wire must be disconnected for upload
  via the MB adapter. Do not press the button during upload.

## Behavior

- Boot: microSD is checked **before** the camera starts and is mandatory.
  - Card present → `SD card OK / Ready to capture`, then the camera
    initialises and the viewfinder starts.
  - Card missing → `NO SD CARD / Insert microSD / waiting Ns`. The firmware
    halts at this point and re-probes every 2 s, showing an elapsed-time
    counter, until a card is inserted. Nothing else proceeds.
- Runtime card removal: the mounted root is probed once per second (`SD_MMC.cardType()`
  is cached at mount time, so it cannot detect removal on its own). Two
  consecutive failed probes stop the viewfinder, unmount, and fall back into
  the same `NO SD CARD` wait loop. Re-inserting the card re-mounts and resumes
  automatically — no power cycle.
- Loop: live preview — the **same SXGA JPEG stream** the stills come from is
  decoded at 1/8 scale (160×128), luma-converted, aspect-fit to 80×64 and
  centred on the 128×64 OLED, then rendered by the selected preview mode.
- Button press (debounced 50 ms): captures **SXGA (1280×1024) JPEG at
  `jpeg_quality = 10`**, saves as `/img_NNNN.jpg` (next free index, never
  overwritten). Expect roughly 90–120 KB per photo — measured at 0.57 bpp on
  this sensor.
  - Success → OLED shows `Saved to SD` + filename for ~0.9 s.
  - Failure → `Capture FAILED` (no frame from the sensor) or
    `Save FAILED` (SD write error / filter error).
  - Because preview and capture share one camera configuration, a shot needs
    **no deinit/init cycle and no AWB/AEC settle** — the next frame is simply
    written to the card. Capture is ~0.2–0.4 s instead of ~1.5–2.5 s.
  - That also removes the old washed-out / green-cast failure, which came from
    re-initialising the sensor and grabbing before AWB had converged.
- The sensor runs neutral tone (contrast 0) since it feeds both the preview and
  the saved JPEG; the contrast a 1-bit panel wants is applied in the dither
  renderer (`DITHER_CONTRAST`, default `130`).
- `AE_LEVEL` in `src/main.cpp` is the auto-exposure bias (-2..+2, 0 neutral);
  lower it if highlights blow out.
- `CAM_VFLIP` / `CAM_HMIRROR` (both `1` by default) rotate the image 180° for
  an upside-down module mount. The flip is applied at the sensor, so the OLED
  preview and the saved JPEGs stay consistent. Set both to `0` for a normally
  oriented module.
- Viewfinder self-heal is **time-based**: only after `CAM_STALL_MS` (1500 ms)
  without a good frame does the camera get restarted, so the brief frame
  drops that happen around mode switches and SD activity are ignored.
  Repeated failures back off 1 s → 2 s → 4 s → 8 s, showing
  `Camera offline / retry Ns`, instead of oscillating between messages.

## Preview modes

The OLED viewfinder image processing is chosen at **compile time**. Saving
full-resolution JPEGs to the SD card is a separate path and never changes.

| `PREVIEW_MODE` | Renderer | Notes |
|---|---|---|
| `PREVIEW_MODE_DITHER` (default) | 4×4 Bayer ordered dither | Cheapest, shows tone and greys |
| `PREVIEW_MODE_CANNY` | Canny edge detection | Wireframe look, ~48 KB extra RAM |

Switch it in the existing `build_flags` block in `platformio.ini` — keep the
other lines, only the `PREVIEW_MODE` value changes:

```ini
build_flags =
 -DJPEG_FILTER_SWAP565=1
 -DPREVIEW_MODE=PREVIEW_MODE_CANNY
 -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
```

or for a one-off build:

```powershell
$env:PLATFORMIO_BUILD_FLAGS="-DPREVIEW_MODE=PREVIEW_MODE_CANNY"; pio run
```

Canny tunables (also `-D` overridable), defaults tuned for outline-only output:

| Macro | Default | Effect |
|---|---|---|
| `CANNY_BLUR_PASSES` | `2` | 3×3 binomial blur repetitions (`0` = off). More passes kill skin texture, hair and sensor noise. |
| `CANNY_HIGH` | `160` | Strong-edge threshold — the density knob. Raise for sparser output. |
| `CANNY_LOW` | `80` | Weak-edge threshold; keep at roughly ⅓–½ of `CANNY_HIGH`. |

Magnitudes are `|gx|+|gy|` on 0–255 input, with a maximum of 1530.
If faces still show too much internal detail, raise `CANNY_BLUR_PASSES` to `3`
before pushing `CANNY_HIGH` higher — blurring removes texture without
thinning genuine object outlines.

**Adding a mode:** create `src/preview_<name>.h` with
`void <name>Draw(Adafruit_SSD1306&, const PreviewFrame&)`, add its id and
`#error` guard in `src/preview.h`, and dispatch it from `previewDraw()`. The
frame arrives already aspect-fit, letterboxed and downscaled, so a renderer
only has to map `PreviewFrame::px` onto the panel.

Measured RAM: dither ≈ 59 KB, Canny ≈ 108 KB (of 320 KB). The baseline rose
with the unified stream, which needs a 160×128 grayscale decode buffer plus
tjgd's work area.

## JPEG filters (SD capture)

A compile-time pipeline in `src/jpeg_filter.h` post-processes the captured
JPEG before it is written to the card. `JPEG_FILTER_NONE` is the default and
is a pure pass-through — the camera's JPEG is written verbatim, with no decode
or re-encode, so it costs nothing.

| `JPEG_FILTER` | Effect | Output |
|---|---|---|
| `JPEG_FILTER_NONE` (default) | Pass-through | SXGA 1280×1024 |
| `JPEG_FILTER_WHITE_BORDER` | Paints a 20 px white band over all four frame edges; edge pixels are lost and the visible image area is 1240×984 | SXGA 1280×1024 (canvas unchanged) |

Enable it in the existing `build_flags` block in `platformio.ini`:

```ini
build_flags =
 -DJPEG_FILTER_SWAP565=1
 -DPREVIEW_MODE=PREVIEW_MODE_DITHER
 -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
```

Tunables (all `-D` overridable):

| Macro | Default | Effect |
|---|---|---|
| `JPEG_FILTER_BORDER_PX` | `20` | Border thickness in output pixels, each side |
| `JPEG_FILTER_SWAP565` | `0` | Set to `1` if a filtered photo shows red/blue swapped |
| `JPEG_QUALITY` | `10` | Camera JPEG quality *and* re-encode quality |

**Memory.** The border does not grow the frame: it paints over the outer 20
pixels on each edge (no scaling or resampling), so the canvas stays
1280×1024. Because `esp_jpg_decode`
delivers MCU blocks through a callback, the decode is written straight into
that canvas — there is no second full-frame buffer — and the encoder streams
to the SD file via `fmt2jpg_cb`, so the JPEG is never held in RAM either.

| Item | Bytes |
|---|---|
| RGB565 canvas, 1280×1024 | 2,621,440 |
| Camera buffer (`fb_count = 1`, w×h/5) | 262,144 |
| **Peak** | **~2.9 MB** (of 4 MB PSRAM) |

Border width is free: it does not change the memory requirement, only the
painted area. `jpegFilterWorstCaseBytes()` reports an upper bound (0.25
bytes/pixel) so the free-space check can run before the file is opened.

Adding a filter: extend the `#if` chain in `src/jpeg_filter.h` with a new
`JPEG_FILTER_*` id and a `jpegFilterStream()` that pushes bytes to the
`JpegWriteFn` sink, plus a `jpegFilterWorstCaseBytes()` upper bound. An
unknown id fails the build via `#error`.

## Robustness

- The UART is only started when debug logging is compiled in — otherwise it
  would hold GPIO1, which doubles as OLED SCL.
- Camera init failure never halts the unit: it retries forever with an
  attempt counter, so a brownout or slow sensor power-up recovers by itself.
- Capture outcomes are distinct: `Saved to SD`, `SD card full`,
  `SD write error`, `10000 files`, `camera error`.
- Free space is checked before a write starts, and a short write deletes the
  partial file so a truncated JPEG never consumes a name slot.
- `nextPhotoPath()` writes into a stack buffer (no `String` heap churn) and
  resumes from the last used index instead of rescanning from `0000`.
- Pressing the shutter with a just-removed card short-circuits to the
  `NO SD CARD` wait instead of failing part-way through a write.
- Streaming retries are paced, so a slow frame after a still cannot be
  mistaken for a dead camera.

Known limitations (by design, not bugs):

- GPIO1 is shared between UART TX and I2C SCL, so no serial logs in this
  build and the OLED SCL wire must be detached to upload.
- Capture blocks the loop for ~0.2–0.4 s, so the preview pauses briefly
  while the file is written.
- SXGA stills need PSRAM: the single JPEG frame buffer reserves ~256 KB. If
  that allocation fails, the boot ladder (`CAM_MODES` in `src/main.cpp`) steps
  down to XGA, SVGA and finally VGA on its own, and the OLED reports which
  mode stuck.
- Filenames are index-only — there is no RTC, so no timestamps.

## Build & upload

Requires [PlatformIO](https://platformio.org/) (`pio` on PATH).

```powershell
pio run                                  # build
pio device list                          # find MB adapter (e.g. COM6, CH340)
pio run --target upload                  # upload (auto-detect port)
pio run --target upload --upload-port COM6   # upload to explicit port
pio device monitor -b 115200 -p COM6     # serial monitor
```

Build output (typical): RAM ~8.4 %, Flash ~14.5 %.

### Upload checklist

1. Disconnect OLED SCL (U0T) and the button before flashing — they hold
   strapping/serial lines and cause `Wrong boot mode detected (0x2b)`.
2. Seat the ESP32-CAM firmly on the MB adapter, use a USB data cable.
3. Install the CH340 driver if no COM port appears.
4. If upload stalls, press RST on the MB adapter and retry.
5. Reconnect peripherals after a successful flash.

## Libraries

Declared in `platformio.ini` (auto-installed on first build):

- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit SSD1306`

Framework libraries (no install needed): `Wire`, `FS`, `SD_MMC`,
ESP32 Arduino camera driver (`esp_camera`).

## Host tools

`tools/film_lab/` is a Python test bench that renders every candidate film look
over the sample captures in `test-images/` and builds a contact-sheet gallery,
so a grade can be chosen by eye without flashing anything. It needs `uv` and the
packages in `tools/film_lab/requirements.txt`; see its
[README](tools/film_lab/README.md) for setup and options.

## Project layout

- `src/main.cpp` — firmware: setup, viewfinder loop, SD capture
- `src/preview.h` — `PREVIEW_MODE` selection, aspect-fit downscale, dispatch
- `src/preview_types.h` — `PreviewFrame` shared by all renderers
- `src/preview_dither.h` — ordered-dither renderer (default)
- `src/preview_canny.h` — Canny edge-detection renderer
- `src/jpeg_filter.h` — compile-time JPEG filter pipeline for SD capture
- `platformio.ini` — `esp32cam` board, Arduino framework, 115200 monitor,
  460800 upload
- `tools/film_lab/` — host-side film look lab (Python + `uv`)
- `test-images/` — sample captures used by the lab (git-ignored, local only)
