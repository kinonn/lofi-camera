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
| Shutter | BOOT (GPIO0) → GND | Onboard button, 10k pull-up, active LOW |
| Filter button | IO14 (GPIO14) → GND | Momentary, internal pull-up; short press cycles the look, long press toggles the border |
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
- GPIO1, 14, 47 and 48 are free header pins. GPIO14 is the filter button;
  1, 47 and 48 remain unused.
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
  centred on the 128×64 OLED, then rendered by the selected preview mode. An
  active look is marked with its initial (`S`/`K`/`T`) in the top-right corner,
  and the preserving border draws a 1 px frame around the panel. The panel runs
  at 400 kHz I2C and the camera at two frame buffers, so the
  viewfinder settles at ~5 fps at SXGA; the sensor frame period, not the
  renderer, is the limit.
- Button press (debounced 50 ms): captures **SXGA (1280×1024) JPEG at
  `jpeg_quality = 10`**, saves as `/img_NNNN.jpg` (next free index, never
  overwritten). Expect roughly 90–120 KB per photo — measured at 0.57 bpp on
  the OV2640 module; an OV3660 has a different bitrate and colour response.
  - Success → OLED shows `Saved` + filename for ~0.9 s.
  - Failure → `CAM FAILED` (no frame from the sensor) or
    `SD FAILED` (SD write error / filter error).
  - Because preview and capture share one camera configuration, a shot needs
    **no deinit/init cycle and no AWB/AEC settle** — the next frame is simply
    written to the card. Capture is ~0.2–0.4 s instead of ~1.5–2.5 s.
  - That also removes the old washed-out / green-cast failure, which came from
    re-initialising the sensor and grabbing before AWB had converged.
- Filter button on GPIO14: a short press cycles the capture look
  (`NONE → SUPERIA → KODACHROME → TMAX`), a ≥700 ms hold toggles the 24 px
  preserving white border. The selection is applied to the next shot, stored
  in NVS, and survives a reboot. The OLED shows the new state for ~2 s.
  The button is captured by an ISR because the ~5 fps viewfinder loop would
  miss a normal tap if it polled the pin.
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
  `Offline / retry Ns`, instead of oscillating between messages.

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

The capture pipeline in `src/jpeg_filter.h` post-processes the captured JPEG
before it is written to the card. Look and border are selected at **runtime**
with the GPIO14 button; the choice is stored in NVS and survives a reboot.
`NONE` is a pure pass-through: the camera's JPEG is written verbatim, with no
decode or re-encode, so it costs nothing.

| Look | Effect | Output |
|---|---|---|
| `NONE` | Pass-through | SXGA 1280×1024 |
| `SUPERIA` | Fuji Superia 400 film look + gaussian unsharp sharpen, ported from the `tools/preview_lab` chain with the lab's default parameters. One decode, one fused colour pass, sharpen as a second pass, one re-encode — no intermediate JPEG round trips. The lab's grain stage is not ported (its default is 0 = off). Measured on the S3 at 240 MHz with `-O3`: about 6.5 s per SXGA shot (decode 0.89, stats 0.62, look 2.04, sharpen 1.17, encode 1.36 s); it was ~14–15 s on the ESP32-CAM. | SXGA 1280×1024 |
| `KODACHROME` | Kodak Kodachrome 64 film look + vibrance + gaussian unsharp, ported from the `tools/preview_lab` chain “Kodachrome 64 → Vibrance (0.5) → Sharpen (0.4, sigma 1)”. Same one-decode, one fused colour pass, one sharpen, one re-encode design. The lab's grain is reproduced as per-pixel monochrome noise (the default grain size 1.2 is below the lab's own blur threshold), and the look carries the same S-curve as T-Max (`KODACHROME_TONE_GAIN` 1.12 about mid-grey, then a `KODACHROME_HIGHLIGHT_ROLLOFF` 0.08 shoulder) folded into the tone LUT. Measured on the S3 at 240 MHz with `-O3`: about 7.25 s per SXGA shot (decode 0.92, stats 0.62, look 2.98, sharpen 1.17, encode 1.43 s). | SXGA 1280×1024 |
| `TMAX` | Kodak T-Max 400 panchromatic B&W + gaussian unsharp, ported from the `tools/preview_lab` chain “T-Max 400 → Sharpen (0.4, sigma 1)”. Same fused design; the panchromatic mix is 0.30/0.59/0.11, with the node's contrast 0.80, vignette 0.30 and grain 0.030, plus an S-curve (`TMAX_TONE_GAIN` 1.12 about mid-grey, then a `TMAX_HIGHLIGHT_ROLLOFF` 0.08 shoulder) that lifts the highlights without clipping them while the shadows go down — see the T-Max macro list below. Measured on the S3 at 240 MHz with `-O3`: about 6.5 s per SXGA shot (decode 0.92, stats 0.65, look 1.71, sharpen 1.28, encode 1.54 s). | SXGA 1280×1024 |
| `PORTRA` | Kodak Portra 400 warm golden-pastel film look + gaussian unsharp, ported from the `tools/preview_lab` chain “Portra 400 → Sharpen (0.3, sigma 1)”. Same fused design; auto balance 0.60, auto contrast 0.45, saturation 0.95, white balance 1.04/1.00/0.96, midtone gamma 0.98, smoothstep contrast 0.14, tone gain 1.00 with a 0.03 highlight shoulder, shadow/highlight tint, vignette 0.06, grain 0.010. Not measured unbordered; the bordered variant is 7.02 s single-core / 5.51 s dual-core (timing table below). | SXGA 1280×1024 |

`SUPERIA` is retired from the button cycle. The implementation remains for the
host test, and a saved `SUPERIA` selection falls back to the default on boot.

A long press toggles a 24 px preserving white border on any look. Unlike the
old `WHITE_BORDER`, the frame **grows**: the image is shifted intact into the
inset and the border is filled around it, with no crop or resampling.
`NONE + border` is the filter formerly called `WHITE_BORDER`, now
`WITH_BORDER`.

| State | Output |
|---|---|
| `NONE`, border off | 1280×1024, byte-identical pass-through |
| `WITH_BORDER` (`NONE` + border) | 1328×1072, re-encoded |
| any look, border off | 1280×1024 |
| any look, border on | 1328×1072 |

The T-Max border variant measures 5.88 s per SXGA shot with the 24 px frame on
one core and 4.59 s on both (full breakdown in the dual-core table below). The
first runtime-selectable build was about 0.9 s slower: every look parameter was
read from flash once per pixel, the zero-valued tint block still ran, and the
sharpen blurred all three planes of a monochrome image. The Superia,
Kodachrome and Portra border variants add the same border/encode stage (not
separately measured).

Controls (GPIO14 → GND, internal pull-up):

| Gesture | Action |
|---|---|
| Short press | `NONE` → `KODACHROME` → `TMAX` → `PORTRA` → `NONE` |
| Long press (≥700 ms) | Toggle the 24 px border |

The OLED shows the new look/border for ~2 s. The selection is stored in NVS
(namespace `lofi`, keys `look` and `border`), so it survives a reboot. Defaults
are set by `FILTER_DEFAULT_LOOK` (`FILTER_LOOK_TMAX`) and
`FILTER_DEFAULT_BORDER` (`0`); erase flash and re-upload (`pio run -t erase`
then `pio run -t upload`) to return to them. The button pin and timings are
`FILTER_BUTTON_PIN` (`14`), `FILTER_LONG_PRESS_MS` (`700`),
`FILTER_DEBOUNCE_MS` (`30`) and `FILTER_TOAST_MS` (`2000`) in
`src/main.cpp`, all `-D` overridable.

Tunables (all `-D` overridable):

| Macro | Default | Effect |
|---|---|---|
| `FILTER_DEFAULT_LOOK` | `FILTER_LOOK_TMAX` | Look used when NVS has no saved value. |
| `FILTER_DEFAULT_BORDER` | `0` | Border state used when NVS has no saved value. |
| `JPEG_FILTER_INSET_PX` | `24` | Border width in output pixels, each side. The canvas grows by `2 ×` this; the image is never cropped. Applies to every bordered state, including `WITH_BORDER`. |
| `JPEG_QUALITY` | `10` | Camera JPEG quality (esp32-camera scale: lower = better) |
| `JPEG_REENCODE_QUALITY` | `60` | Re-encode quality after a filter runs (fmt2jpg scale: higher = better). The two scales are unrelated — encoder quality 10 means ~5.5× scaled tables plus 4:2:0 subsampling, which visibly solarizes smooth dusk gradients. |
| `JPEG_FILTER_DUALCORE` | `1` | Split the two statistics passes, the look pass and the sharpen pass across both S3 cores by row. `0` forces the single-core path. Decode and encode stay single-core. |

On the S3 the statistics passes, the look pass and the sharpen pass are split by
row across both LX7 cores (`JPEG_FILTER_DUALCORE`, default `1`). The worker is
pinned to core 0, created on the first filtered capture, and the build falls
back to the single-core path if it cannot be created; the arithmetic is
unchanged, so the host test and existing outputs stay byte-identical. The
canvas is sharpened in place, so the two sharpen bands copy the 2R boundary
rows into a small halo before either starts writing. The per-look stage
breakdowns in the table above are the original single-core runs; the current
single-core and dual-core totals are below.

Measured on the connected S3 at 240 MHz with the 24 px border, three shots per
build:

| Look + border | Stats | Look | Sharpen | Total |
|---|---|---|---|---|
| T-Max 400, single core | 0.57 s | 1.41 s | 0.80 s | 5.88 s |
| T-Max 400, dual core | 0.35 s | 0.85 s | 0.42 s | 4.59 s |
| Portra 400, single core | 0.57 s | 2.51 s | 1.17 s | 7.02 s |
| Portra 400, dual core | 0.35 s | 1.62 s | 0.60 s | 5.51 s |

Stats and look scale about 1.65x (the shared cache/PSRAM path is the likely
ceiling); sharpen scales about 1.9x because it works from the internal-DRAM
ring. Total capture drops ~22 %. The parallel sharpen path needs the static
ring, so a wider `*_USM_SIGMA` (rad > 3) falls back to single-core sharpen; the
default looks all use rad 3. `-DJPEG_FILTER_DUALCORE=0` reproduces the
single-core rows.

The split was checked three ways: the host matrix still compiles the same
per-pixel math, a host harness compares the two-band sharpen with the
single-band result byte-for-byte, and the phase-2 captures were compared with
single-core shots of the same scene — the bright row at the split height is
present in every build, so it is scene content, not a seam. No brownout reset
occurred during the bench runs; the 18650 + 5 V boost supply is the intended
field supply and still needs its own soak test.

The Superia look inlines the preview lab's node defaults as `SUPERIA_*`
macros (auto balance 0.55, auto contrast 0.56, white balance 0.98/1.01/1.00,
midtone gamma 0.97, shadow/highlight tint, vignette 0.50, unsharp amount 0.90
at sigma 1.00). Each is overridable with
`-DSUPERIA_<NAME>=<value>`; see the `SUPERIA_*` block in
`src/jpeg_filter.h`. The S3 build uses the PIE sharpen kernel by default;
`-DSUPERIA_USM_SIMD=0` forces the scalar path (the host test always uses
scalar).

The Kodachrome filter inlines its node defaults as `KODACHROME_*` macros
(auto balance 0.62, auto contrast 0.62, saturation 1.24, midtone gamma 0.94,
smoothstep contrast 0.40, S-curve gain 1.12 with a 0.08 highlight shoulder,
warm 3×3 matrix, shadow/highlight tint, vignette 0.10, grain 0.010, vibrance
0.50, unsharp amount 0.40 at sigma 1.00),
overridable with `-DKODACHROME_<NAME>=<value>`. The 3×3 matrix entries are
`KODACHROME_MAT_RR` … `KODACHROME_MAT_BB`. The white balance is identity, so
the linear-light LUT is skipped for this look.

The T-Max look inlines its node defaults as `TMAX_*` macros (auto balance
0.70, auto contrast 0.55, midtone gamma 0.98, smoothstep contrast 0.80,
panchromatic mix 0.30/0.59/0.11, vignette 0.30, grain 0.030, unsharp amount
0.40 at sigma 1.00). It has no colour matrix, saturation or vibrance stage;
`TMAX_MONO_R/G/B` adjust the B&W channel mix.

Its tone stage is the only one with the lab's full curve. After the smoothstep
blend it applies `TMAX_TONE_GAIN` (default 1.12), a slope about mid-grey that
darkens the shadows and lifts the highlights, and then
`TMAX_HIGHLIGHT_ROLLOFF` (default 0.08), a tanh shoulder that compresses
everything above `1 - rolloff` so lifted highlights saturate towards white
instead of clipping — the same `contrast_gain` → `_highlight_rolloff` pair, in
the same order, as the lab's `tone()`. Both are folded into the existing
256-entry tonal LUT, so the per-pixel cost is unchanged; set either to its
identity (`1.0` / `0.0`) to get the pre-S-curve look back. The shoulder matters
because the unsharp mask runs *after* the tone stage and clamps at 255: without
it, a steeper S eats highlight detail that the sharpen pass then cannot recover.
`TMAX_HIGHLIGHT_ROLLOFF` is range-checked at compile time (0 ≤ r < 1).

The Portra filter inlines its node defaults as `PORTRA_*` macros (auto balance
0.60, auto contrast 0.45, saturation 0.95, white balance 1.04/1.00/0.96,
midtone gamma 0.98, smoothstep contrast 0.14, tone gain 1.00 with a 0.03
highlight shoulder, shadow/highlight tint, vignette 0.06, grain 0.010, unsharp
amount 0.30 at sigma 1.00), overridable with
`-DPORTRA_<NAME>=<value>`. It has no matrix, vibrance or monochrome stage;
its grain size 1.2 is below the lab's blur threshold, so only the amount is
reproduced.

**Memory.** `NONE` with border off does not allocate a canvas at all. Every
re-encoded state decodes into one RGB565 canvas. Bordered states grow the
frame: the canvas is `(w+2b) × (h+2b)` (1328×1072 at 24 px), while the
look/sharpen passes still run on the leading 1280×1024 image. `jpg2rgb565()`
writes straight into that one canvas and `fmt2jpg_cb` streams the encoded JPEG
to the SD file, so there is no second full-frame buffer and the JPEG is never
held in RAM either.

| Item | Bytes |
|---|---|
| RGB565 canvas, 1280×1024 (unbordered re-encode) | 2,621,440 |
| RGB565 canvas, 1328×1072 (any bordered state) | 2,847,232 |
| Camera buffers (`fb_count = 2`, w×h/5 each) | 524,288 |
| **Peak, unbordered looks** | **~3.2 MB** (of 8 MB PSRAM) |
| **Peak, bordered looks / `WITH_BORDER`** | **~3.4 MB** (of 8 MB PSRAM) |

The border canvas grows with `JPEG_FILTER_INSET_PX`, and
`jpegFilterWorstCaseBytes()` includes the extra rows/columns. The free-space
bound is 0.25 bytes/pixel at encoder quality 10; with the default
`JPEG_REENCODE_QUALITY` of 60 the budget doubles to 0.5 bytes/pixel.

All five looks share one runtime-parameterized pipeline, one LUT set and the
same ring/LUT working set. The sharpen ring (one 16-byte group per
channel/pixel, holding the 2·radius+1 blurred rows as int16 slots, ~61 KB at
SXGA) and LUTs (~5 KB) live in internal DRAM, not PSRAM, so peak PSRAM stays
at ~3.2 MB for unbordered looks and ~3.4 MB for bordered states. Keep it there:
an earlier version used a ~0.67 MB band buffer in PSRAM, pushed the peak to
~3.6 MB, and the old ESP32-CAM's PSRAM delivered block-garbage into the canvas
at that pressure (psychedelic capture output). The 16-byte groups are also what
let the vertical pass use the S3's 8-lane 16-bit dot product
(`ee.vmulas.s16.accx` in `src/usm_simd.S`); the scalar fallback runs on the
host, on wider kernels, and when the ring comes from the PSRAM heap. Check the
link size after any change to these buffers.

The dual-core build adds, in internal DRAM, an 8 KB second histogram table, a
second 61 KB sharpen ring, a second 3.8 KB 8-bit row buffer, a 15 KB sharpen
halo, and a 4 KB worker stack plus two semaphores (allocated once on the first
filtered capture). PSRAM peak is unchanged; the second core only splits
existing rows.

Adding a look: add a `FilterLook` value, a `FilmLookParams` entry in
`FILM_LOOKS`, a name in `jpegFilterLookName()`, an entry in `LOOK_STAGES`, and
a dispatch case in `jpegFilterStream`; the shared pipeline and the border stage
pick it up automatically. All values are ordinary `-D` overridable macros, and
no extra `#elif` branch is needed.

## Robustness

- Debug logging goes to the USB-C CDC port (`DBG()`), so it owns no GPIO and
  needs no cable changes.
- Camera init failure never halts the unit: it retries forever with an
  attempt counter, so a brownout or slow sensor power-up recovers by itself.
- Capture outcomes are distinct: `Saved`, `SD card full`,
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
- Capture blocks the loop: about 0.2-0.4 s with `NONE`; the filtered looks
  block for the whole shot (measured with the 24 px border: ~4.6 s T-Max and
  ~5.5 s Portra on the dual-core build, vs ~5.9 s and ~7.0 s single-core), so
  the preview pauses while the file is written.
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

Build output with the checked-in flags (runtime filters, USB CDC, `-O3`,
dual-core stats/look/sharpen):
RAM 72.1 % (236,236 / 327,680 B), Flash 15.7 % (523,597 / 3,342,336 B).

### Upload checklist

1. Connect the USB-C data cable. No peripheral needs to be disconnected.
2. Run the upload; the board enumerates as a USB-Serial-JTAG device.
3. If no port appears, hold BOOT, tap RST, release BOOT, then retry.

## Libraries

Declared in `platformio.ini` (auto-installed on first build):

- `adafruit/Adafruit GFX Library`
- `adafruit/Adafruit SSD1306`

Framework libraries (no install needed): `Wire`, `FS`, `SD_MMC`,
`Preferences`, ESP32 Arduino camera driver (`esp_camera`).

## Host tools

`tools/film_lab/` is a Python test bench that renders every candidate film look
over the sample captures in `test-images/` and builds a contact-sheet gallery,
so a grade can be chosen by eye without flashing anything. It needs `uv` and the
packages in `tools/film_lab/requirements.txt`; see its
[README](tools/film_lab/README.md) for setup and options.

`tools/jpeg_filter_test/` compiles the real `src/jpeg_filter.h` for the host
(under WSL) and runs the full host test matrix — `NONE`, `WITH_BORDER`, the
four looks, the four bordered looks, and the Superia/Kodachrome/T-Max
diagnostics — over the sample captures. It writes an HTML gallery to
`tools/jpeg_filter_test/out/gallery/index.html`. This is the check that
`NONE` is byte-identical pass-through and that every bordered output is
1328×1072 with the image intact.

## Project layout

- `src/main.cpp` — firmware: setup, viewfinder loop, SD capture
- `src/preview.h` — `PREVIEW_MODE` selection, aspect-fit downscale, dispatch
- `src/preview_types.h` — `PreviewFrame` shared by all renderers
- `src/preview_floyd_steinberg.h` — serpentine Floyd–Steinberg renderer
- `src/jpeg_filter.h` — runtime look/border JPEG pipeline for SD capture
- `src/usm_simd.S` — ESP32-S3 PIE dot-product kernel for the look sharpen
- `platformio.ini` — `esp32-s3-devkitc-1` (8 MB flash, 8 MB OPI PSRAM),
  Arduino framework, 115200 monitor, 460800 upload
- `tools/film_lab/` — host-side film look lab (Python + `uv`)
- `tools/jpeg_filter_test/` — host compile/run matrix for `src/jpeg_filter.h`
- `test-images/` — sample captures used by the lab (git-ignored, local only)
