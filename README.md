# lofi-camera

ESP32-S3-CAM lo-fi viewfinder: live grayscale camera preview on a
128×64 SSD1306 OLED, with button-triggered full-resolution JPEG capture to
microSD.

The host-side [film look lab](tools/film_lab/README.md) prototypes the colour
looks for those JPEGs before any of them are ported to C++.

## Hardware

- ESP32-S3-CAM (ESP32-S3R8: 8 MB flash, 8 MB octal PSRAM, native USB-C)
- SSD1306 128×64 OLED (I2C, address `0x3C` / `0x3D`)
- Momentary push button (shutter) — the onboard BOOT button is used
- FAT32-formatted microSD card

## Wiring

Use the silkscreen labels on the ESP32-S3-CAM header.

| Peripheral | ESP32-S3-CAM pin | Notes |
|---|---|---|
| OLED SDA | IO41 (GPIO41) | `Wire` at 400 kHz (I2C port 0) |
| OLED SCL | IO42 (GPIO42) | right-hand header |
| OLED VCC | 3V3 | **not** 5V (left header) |
| OLED GND | GND | |
| Button | BOOT (GPIO0) → GND | Onboard button, 10k pull-up, active LOW |
| microSD | Onboard slot | SD_MMC 1-bit: CLK 39, CMD 38, DATA0 40 |

`Wire` uses I2C port 0; the camera's SCCB is on I2C port 1 in the Arduino
core's S3 build (`CONFIG_SCCB_HARDWARE_I2C_PORT1`), so the buses don't collide.

### Pin constraints (why these pins)

- Camera uses GPIO 4–13 and 15–18 (SCCB 4/5, VSYNC/HREF 6/7, PCLK 13,
  XCLK 15, data 8–12 and 16–18) — do not reuse. There is no PWDN or sensor
  reset line: `CAM_RST` is only pulled up on the module.
- SD 1-bit mode uses GPIO 39 (CLK), 38 (CMD), 40 (DATA0). D1–D3 are not
  wired and there is no card-detect switch.
- GPIO 26–37 are the in-package flash and octal PSRAM — never use them.
- GPIO19/20 are USB D-/D+; GPIO43/44 are UART0; GPIO45/46 are strapping.
- GPIO 2 and 3 are not free: GPIO2 drives the onboard green LED through 1k,
  GPIO3 drives the two flash LEDs through a transistor (active HIGH).
- GPIO0 is the BOOT strapping pin and the onboard shutter button. It is only
  sampled at reset, so pressing it while the firmware runs cannot reset or
  reboot the board; holding it through a reset enters ROM download mode.
- GPIO41/42 are the JTAG MTDI/MTMS pins, so external JTAG is unavailable.
  The built-in USB-Serial-JTAG port (GPIO19/20) still works.
- No UART pin is shared with a peripheral: `DBG()` goes to the USB-C CDC
  port, and nothing has to be disconnected to flash.

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
  centred on the 128×64 OLED, then rendered by the selected preview mode. The
  panel runs at 400 kHz I2C and the camera at two frame buffers, so the
  viewfinder settles at ~5 fps at SXGA; the sensor frame period, not the
  renderer, is the limit.
- Button press (debounced 50 ms): captures **SXGA (1280×1024) JPEG at
  `jpeg_quality = 10`**, saves as `/img_NNNN.jpg` (next free index, never
  overwritten). Expect roughly 90–120 KB per photo — measured at 0.57 bpp on
  the OV2640 module; an OV3660 has a different bitrate and colour response.
  - Success → OLED shows `Saved to SD` + filename for ~0.9 s.
  - Failure → `Capture FAILED` (no frame from the sensor) or
    `Save FAILED` (SD write error / filter error).
  - Because preview and capture share one camera configuration, a shot needs
    **no deinit/init cycle and no AWB/AEC settle** — the next frame is simply
    written to the card. Capture is ~0.2–0.4 s instead of ~1.5–2.5 s.
  - That also removes the old washed-out / green-cast failure, which came from
    re-initialising the sensor and grabbing before AWB had converged.
- The sensor runs neutral tone (contrast 0) since it feeds both the preview and
  the saved JPEG; preview contrast is applied in the Floyd–Steinberg renderer
  (`DITHER_CONTRAST`, default `130`).
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

## Preview renderer

The OLED viewfinder image processing is chosen at **compile time**. Saving
full-resolution JPEGs to the SD card is a separate path and never changes.

The firmware supports one renderer: `PREVIEW_MODE_FLOYD_STEINBERG`, using
serpentine Floyd–Steinberg error diffusion with a threshold of `128` and an
error-diffusion strength of `0.45`. `DITHER_CONTRAST` (default `130`) applies a
linear contrast stretch around mid-grey before error diffusion. These settings
are compile-time tunables and may be overridden with `-D` build flags:
`FLOYD_STEINBERG_THRESHOLD`, `FLOYD_STEINBERG_STRENGTH`, and
`DITHER_CONTRAST`. Canny edge detection remains available as a host-side
`tools/preview_lab/` experiment; it is not compiled into the firmware.

**Adding a mode:** create `src/preview_<name>.h` with
`void <name>Draw(Adafruit_SSD1306&, const PreviewFrame&)`, add its id and
`#error` guard in `src/preview.h`, and dispatch it from `previewDraw()`. The
frame arrives already aspect-fit, letterboxed and downscaled, so a renderer
only has to map `PreviewFrame::px` onto the panel.

Measured firmware RAM after replacing ordered dither with Floyd–Steinberg:
≈ 60 KB (of 512 KB internal SRAM). The renderer uses two 128-float scanline
error buffers; the unified stream also needs a 160×128 grayscale decode buffer
plus tjgd's work area.

## JPEG filters (SD capture)

A compile-time pipeline in `src/jpeg_filter.h` post-processes the captured
JPEG before it is written to the card. `JPEG_FILTER_NONE` is the default and
is a pure pass-through — the camera's JPEG is written verbatim, with no decode
or re-encode, so it costs nothing.

| `JPEG_FILTER` | Effect | Output |
|---|---|---|
| `JPEG_FILTER_NONE` (default) | Pass-through | SXGA 1280×1024 |
| `JPEG_FILTER_WHITE_BORDER` | Paints a 20 px white band over all four frame edges; edge pixels are lost and the visible image area is 1240×984 | SXGA 1280×1024 (canvas unchanged) |
| `JPEG_FILTER_SUPERIA_USM` | Fuji Superia 400 film look + gaussian unsharp sharpen, ported from the `tools/preview_lab` chain with the lab's default parameters. One decode, one fused colour pass, sharpen as a second pass, one re-encode — no intermediate JPEG round trips. The lab's grain stage is not ported (its default is 0 = off). Measured on the S3 at 240 MHz with `-O3`: about 6.6 s per SXGA shot (decode 0.95, stats 0.65, look 2.09, sharpen 1.24, encode 1.4 s); it was ~14–15 s on the ESP32-CAM. | SXGA 1280×1024 |
| `JPEG_FILTER_KODACHROME_USM` | Kodak Kodachrome 64 film look + vibrance + gaussian unsharp, ported from the `tools/preview_lab` chain “Kodachrome 64 → Vibrance (0.5) → Sharpen (0.4, sigma 1)”. Same one-decode, one fused colour pass, one sharpen, one re-encode design. The lab's grain is reproduced as per-pixel monochrome noise (the default grain size 1.2 is below the lab's own blur threshold). Measured on the S3 at 240 MHz with `-O3`: about 7.3 s per SXGA shot (decode 0.87, stats 0.66, look 2.71, sharpen 1.28, encode 1.35 s). | SXGA 1280×1024 |
| `JPEG_FILTER_TMAX_USM` | Kodak T-Max 400 panchromatic B&W + gaussian unsharp, ported from the `tools/preview_lab` chain “T-Max 400 → Sharpen (0.4, sigma 1)”. Same fused design; the panchromatic mix is 0.30/0.59/0.11, with the node's contrast 0.80, vignette 0.30 and grain 0.030. Measured on the S3 at 240 MHz with `-O3`: about 6.5 s per SXGA shot (decode 0.92, stats 0.65, look 1.71, sharpen 1.28, encode 1.54 s). | SXGA 1280×1024 |

Enable it in the existing `build_flags` block in `platformio.ini`:

```ini
build_flags =
 -DJPEG_FILTER_SWAP565=1
 -DPREVIEW_MODE=PREVIEW_MODE_FLOYD_STEINBERG
 -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
```

Tunables (all `-D` overridable):

| Macro | Default | Effect |
|---|---|---|
| `JPEG_FILTER_BORDER_PX` | `20` | Border thickness in output pixels, each side |
| `JPEG_FILTER_SWAP565` | `0` | WHITE_BORDER only: one RGB565 byte-swap pass before re-encode — `fmt2jpg` reads RGB565 input big-endian while the decoder writes the canvas little-endian, so the swap keeps red/blue true. SUPERIA ignores it (it always swaps pre-encode). |
| `JPEG_QUALITY` | `10` | Camera JPEG quality (esp32-camera scale: lower = better) |
| `JPEG_REENCODE_QUALITY` | `60` | Re-encode quality after a filter runs (fmt2jpg scale: higher = better). The two scales are unrelated — encoder quality 10 means ~5.5× scaled tables plus 4:2:0 subsampling, which visibly solarizes smooth dusk gradients. |

The Superia filter inlines the preview lab's node defaults as `SUPERIA_*`
macros (auto balance 0.55, auto contrast 0.56, white balance 0.98/1.01/1.00,
midtone gamma 0.97, shadow/highlight tint, vignette 0.50, unsharp amount 0.90
at sigma 1.00). Each is overridable with
`-DSUPERIA_<NAME>=<value>`; see the tunables block at the top of the
`JPEG_FILTER_SUPERIA_USM` branch in `src/jpeg_filter.h`. The S3 build uses the
PIE sharpen kernel by default; `-DSUPERIA_USM_SIMD=0` forces the scalar path
(the host test always uses scalar).

The Kodachrome filter inlines its node defaults as `KODACHROME_*` macros
(auto balance 0.62, auto contrast 0.62, saturation 1.24, midtone gamma 0.94,
smoothstep contrast 0.60, warm 3×3 matrix, shadow/highlight tint, vignette
0.10, grain 0.010, vibrance 0.50, unsharp amount 0.40 at sigma 1.00),
overridable with `-DKODACHROME_<NAME>=<value>`. The 3×3 matrix entries are
`KODACHROME_MAT_RR` … `KODACHROME_MAT_BB`. The white balance is identity, so
the linear-light LUT is skipped for this look.

The T-Max filter inlines its node defaults as `TMAX_*` macros (auto balance
0.70, auto contrast 0.55, midtone gamma 0.98, smoothstep contrast 0.80,
panchromatic mix 0.30/0.59/0.11, vignette 0.30, grain 0.030, unsharp amount
0.40 at sigma 1.00). It has no colour matrix, saturation or vibrance stage;
`TMAX_MONO_R/G/B` adjust the B&W channel mix.

**Memory.** The border does not grow the frame: it paints over the outer 20
pixels on each edge (no scaling or resampling), so the canvas stays
1280×1024. Because `esp_jpg_decode`
delivers MCU blocks through a callback, the decode is written straight into
that canvas — there is no second full-frame buffer — and the encoder streams
to the SD file via `fmt2jpg_cb`, so the JPEG is never held in RAM either.

| Item | Bytes |
|---|---|
| RGB565 canvas, 1280×1024 | 2,621,440 |
| Camera buffers (`fb_count = 2`, w×h/5 each) | 524,288 |
| **Peak** | **~3.2 MB** (of 8 MB PSRAM) |

Border width is free: it does not change the memory requirement, only the
painted area. `jpegFilterWorstCaseBytes()` reports an upper bound (0.25
bytes/pixel at encoder quality 10) so the free-space check can run before the
file is opened; with the default `JPEG_REENCODE_QUALITY` of 60 the budget
doubles to 0.5 bytes/pixel.

All three look filters share the canvas and bound above. Their sharpen ring (one
16-byte group per channel/pixel, holding the 2·radius+1 blurred rows as int16
slots, ~61 KB at SXGA) and LUTs (~5 KB) live in internal DRAM, not PSRAM, so
peak PSRAM stays at the border filter's ~3.2 MB envelope. Keep it there: an
earlier version used a ~0.67 MB band buffer in PSRAM, pushed the peak to
~3.6 MB, and the old ESP32-CAM's PSRAM delivered block-garbage into the canvas
at that pressure (psychedelic capture output). The 16-byte groups are also what
let the vertical pass use the S3's 8-lane 16-bit dot product
(`ee.vmulas.s16.accx` in `src/usm_simd.S`); the scalar fallback runs on the
host, on wider kernels, and when the ring comes from the PSRAM heap. Check the
link size after any change to these buffers.

Adding a filter: extend the `#if` chain in `src/jpeg_filter.h` with a new
`JPEG_FILTER_*` id and a `jpegFilterStream()` that pushes bytes to the
`JpegWriteFn` sink, plus a `jpegFilterWorstCaseBytes()` upper bound. An
unknown id fails the build via `#error`.

## Robustness

- Debug logging goes to the USB-C CDC port (`DBG()`), so it owns no GPIO and
  needs no cable changes.
- Camera init failure never halts the unit: it retries forever with an
  attempt counter, so a brownout or slow sensor power-up recovers by itself.
- Capture outcomes are distinct: `Saved to SD`, `SD card full`,
  `SD write error`, `10000 files`, `camera error`.
- Free space is checked before a write starts, and a short write deletes the
  partial file so a truncated JPEG never consumes a name slot.
- `nextPhotoPath()` writes into a stack buffer (no `String` heap churn) and
  starts from the last used index, persisted in `/last_idx.txt` on the card.
  A reboot resumes at the last slot; a blank/new card has no index file and
  starts at `0000`. The scan still skips every occupied name, so a stale or
  hand-edited index can only shift the numbering, never overwrite a photo.
- Pressing the shutter with a just-removed card short-circuits to the
  `NO SD CARD` wait instead of failing part-way through a write.
- Streaming retries are paced, so a slow frame after a still cannot be
  mistaken for a dead camera.

Known limitations (by design, not bugs):

- The shutter is the onboard BOOT button (GPIO0), so holding it through a
  reset puts the ROM in download mode instead of booting the firmware.
- Capture blocks the loop: about 0.2-0.4 s with `JPEG_FILTER_NONE`, and the
  whole ~6.5 s T-Max pass with the default build, so the preview pauses
  while the file is written.
- SXGA stills need PSRAM: the stream reserves two ~256 KB frame buffers. If
  that allocation fails, the boot ladder (`CAM_MODES` in `src/main.cpp`) steps
  down to XGA, SVGA and finally VGA on its own, and the OLED reports which
  mode stuck.
- Filenames are index-only — there is no RTC, so no timestamps.

## Build & upload

Requires [PlatformIO](https://platformio.org/) (`pio` on PATH).

```powershell
pio run                                  # build
pio device list                          # find the USB-Serial-JTAG port
pio run --target upload                  # upload (auto-detect port)
pio run --target upload --upload-port COM6   # upload to explicit port
pio device monitor -b 115200 -p COM6     # USB CDC serial monitor
```

Build output with the checked-in flags (T-Max filter, USB CDC, `-O3`):
RAM 44.4 % (145,564 / 327,680 B), Flash 14.1 % (470,873 / 3,342,336 B).

### Upload checklist

1. Connect the USB-C data cable. No peripheral needs to be disconnected.
2. Run the upload; the board enumerates as a USB-Serial-JTAG device.
3. If no port appears, hold BOOT, tap RST, release BOOT, then retry.

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
- `src/preview_floyd_steinberg.h` — serpentine Floyd–Steinberg renderer
- `src/jpeg_filter.h` — compile-time JPEG filter pipeline for SD capture
- `src/usm_simd.S` — ESP32-S3 PIE dot-product kernel for the Superia sharpen
- `platformio.ini` — `esp32-s3-devkitc-1` (8 MB flash, 8 MB OPI PSRAM),
  Arduino framework, 115200 monitor, 460800 upload
- `tools/film_lab/` — host-side film look lab (Python + `uv`)
- `test-images/` — sample captures used by the lab (git-ignored, local only)
