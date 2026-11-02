# Preview lab

Host-side simulator for the lofi-camera **OLED viewfinder** — the dither and
Canny preview modes in `src/preview.h`, `src/preview_dither.h` and
`src/preview_canny.h`. It renders both modes over the sample captures in
`../../test-images/` so the 1-bit look can be judged **before** flashing
anything. No firmware file is touched by this tool.

## What it simulates

Per image, mirroring the device pipeline:

```
RGB -> Rec.709 luma -> aspect-fit letterbox (nearest neighbour, 128x64)
-> dither variant / Canny edge detection
```

`dither` is the exact firmware 4x4 Bayer renderer. The extra dither modes
(`bayer2`, `bayer8`, `floyd`, `atkinson`, `threshold`, `clahe-floyd`,
`blur-atkinson`, `photo`) are lab-only alternatives for judging what a
different 1-bit
mapping would look like on the same panel.

`blur-atkinson` runs after grayscale as slight Gaussian blur -> Atkinson,
sharing the `--clahe-blur` sigma with the CLAHE chain (`0` = off).

`clahe-floyd` runs after grayscale as CLAHE -> slight Gaussian blur ->
Floyd-Steinberg, all on the letterboxed tile. Tune it with `--clahe-clip`
(local punch), `--clahe-tiles ROWS COLS` (adaptation grid),
`--clahe-blur` (pre-diffusion softening, `0` = off), and
`--clahe-contrast` (firmware-style stretch before CLAHE).

The Canny path follows the firmware step for step: optional binomial blur
passes, Sobel `|gx| + |gy|` gradients with quantised direction, one-sided
non-maximum suppression, double threshold, and hysteresis.

## Setup

The lab uses `uv` for its virtual environment:

```powershell
uv venv .venv --python 3.10
uv pip install --python .venv\Scripts\python.exe -r tools/preview_lab/requirements.txt
```

## Run

```powershell
# both modes over every sample image
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py

# what are the modes?
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --list

# only one mode, or a single image
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --modes dither
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --input test-images/img_0027.jpg

# tune the firmware knobs
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --dither-contrast 130 --blur-passes 2 --low 80 --high 160
```

Then open `tools/preview_lab/out/index.html` — it shows the original next to
both 1-bit previews per sample photo.

### Output layout

```
tools/preview_lab/out/
  dither/<image>__dither.png            128x64 1-bit dither result
  dither/<image>__dither_x4.png        magnified for viewing
  canny/<image>__canny.png             128x64 1-bit Canny result
  canny/<image>__canny_x4.png          magnified for viewing
  contact_sheets/<image>__contact_sheet.jpg   original + both previews
  index.html                           gallery
```

Every output filename carries the preview mode, so a result is never ambiguous.

---

## Tunables (mirror the `-D` firmware flags)

| Flag | Default | Effect |
|---|---|---|
| `--dither-contrast` | `130` | `DITHER_CONTRAST`: linear stretch around mid-grey (`100` = unchanged) |
| `--blur-passes` | `2` | `CANNY_BLUR_PASSES`: 3x3 binomial blur repetitions (`0` = off) |
| `--low` | `80` | `CANNY_LOW`: weak-edge threshold |
| `--high` | `160` | `CANNY_HIGH`: strong-edge threshold (the density knob) |
| `--preview-scale` | `4` | Magnify 128x64 previews for viewing (nearest neighbour) |
| `--modes` | all ten | Which preview modes to render (`dither bayer2 bayer8 floyd atkinson threshold clahe-floyd blur-atkinson photo canny`) |
| `--photo-contrast` / `--photo-brightness` / `--photo-gamma` | `1.15` / `0` / `1.0` | Photo tone: gain around mid-grey, flat offset, power curve |
| `--photo-alpha` | `0.80` | Photo temporal weight on past frames (`0` = no memory) |
| `--photo-strength` | `0.90` | Photo diffused error fraction (`1.0` = classic Floyd-Steinberg) |
| `--photo-threshold` | `128` | Photo black/white decision point |
| `--photo-sharpen` | `0` | Photo unsharp-mask amount (`0` = off) |
| `--photo-blur` | `0` | Photo Gaussian sigma before diffusion (`0` = off) |
| `--photo-resize` | `area` | Photo downsampler: `area` averaging, or `nearest` only if already 128x64 |

## Notes

- The simulator uses the same integer weights and thresholds as the firmware
  (`54/183/19` luma, `*16 + 8` Bayer thresholds, one-sided NMS), so the
  outputs should match the panel pixel-for-pixel for the same input luma.
- Input images are converted with Rec.709 luma; the device decodes its SXGA
  JPEG stream at 1/8 scale first, so expect slightly softer edges on-device
  than from these full-resolution samples.
