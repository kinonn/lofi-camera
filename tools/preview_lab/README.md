# Preview lab

Host-side simulator for the lofi-camera **OLED viewfinder**. It renders the
firmware's Floyd–Steinberg mode and lab-only Bayer/Canny alternatives over
sample captures in `../../test-images/` or evenly sampled video frames so the
1-bit look can be judged **before** flashing anything. No firmware file is
touched by this tool.

## What it simulates

Per image or sampled video frame, mirroring the device pipeline:

```
RGB -> Rec.709 luma -> aspect-fit letterbox (nearest neighbour, 128x64)
-> Floyd–Steinberg or a lab-only dither variant / Canny edge detection
```

`floyd` uses the firmware's serpentine Floyd–Steinberg renderer (threshold
128, strength 0.45). `dither` is a lab-only 4×4 Bayer alternative. The other
lab-only modes (`bayer2`, `bayer8`, `atkinson`, `threshold`, `clahe-floyd`,
`blur-atkinson`, `photo`, and `canny`) are alternatives for judging different
1-bit mappings on the same panel.

`blur-atkinson` runs after grayscale as slight Gaussian blur -> Atkinson,
sharing the `--clahe-blur` sigma with the CLAHE chain (`0` = off).

The interactive `Resize` block defaults to maintaining the input aspect ratio
within its width/height bounds (128x64 by default); clear the checkbox to
stretch to the exact dimensions instead.

After running a pipeline, each Output block enables **Copy last-run config**.
The clipboard contains versioned JSON with ordered nodes and explicit edges,
stable operation identifiers (for example, `gaussian_blur`,
`floyd_steinberg_dithering`, and `canny_edge_detection`), descriptive
parameter values, typed ports (including accepted input types and each edge's
effective data type), algorithm contracts, pixel conventions, and output
dimensions. The last-run config records the resolved input source and decoded
dimensions. Uploaded image bytes are not embedded; the config records the file
name and metadata so an implementation can be supplied with that source asset
separately. If an upload is unavailable, its fallback order is included too.

`clahe-floyd` runs after grayscale as CLAHE -> slight Gaussian blur ->
Floyd-Steinberg, all on the letterboxed tile. Tune it with `--clahe-clip`
(local punch), `--clahe-tiles ROWS COLS` (adaptation grid),
`--clahe-blur` (pre-diffusion softening, `0` = off), and
`--clahe-contrast` (firmware-style stretch before CLAHE).

The host-only Canny path uses optional binomial blur passes, Sobel
`|gx| + |gy|` gradients with quantised direction, one-sided non-maximum
suppression, double threshold, and hysteresis. Canny is not included in the
firmware; the device's only preview renderer is Floyd–Steinberg.

## Setup

The lab uses `uv` for its virtual environment:

```powershell
uv venv .venv --python 3.10
uv pip install --python .venv\Scripts\python.exe -r tools/preview_lab/requirements.txt
```

## Run

```powershell
# all configured lab modes over every sample image
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py

# what are the modes?
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --list

# only one mode, or a single image
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --modes dither
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --input test-images/img_0027.jpg

# sample 12 chronological frames from a video; photo alpha blends each with
# temporal state from its previous sampled frame
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --input clip.mp4 --modes photo --video-frames 12 --photo-alpha 0.8

# tune preview contrast and lab-only Canny parameters
.venv\Scripts\python.exe tools\preview_lab\run_preview_lab.py --dither-contrast 130 --blur-passes 2 --low 80 --high 160
```

Then open `tools/preview_lab/out/index.html` — it shows the original next to
the selected 1-bit previews for each input image or sampled video frame.

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

## Tunables

| Flag | Default | Effect |
|---|---|---|
| `--dither-contrast` | `130` | `DITHER_CONTRAST`: linear stretch around mid-grey before dithering (`100` = unchanged) |
| `--blur-passes` | `2` | Canny 3x3 binomial blur repetitions (`0` = off; lab-only) |
| `--low` | `80` | Canny weak-edge threshold (lab-only) |
| `--high` | `160` | Canny strong-edge threshold (lab-only density knob) |
| `--preview-scale` | `4` | Magnify 128x64 previews for viewing (nearest neighbour) |
| `--modes` | all ten | Which preview modes to render (`dither bayer2 bayer8 floyd atkinson threshold clahe-floyd blur-atkinson photo canny`) |
| `--photo-contrast` / `--photo-brightness` / `--photo-gamma` | `1.15` / `0` / `1.0` | Photo tone: gain around mid-grey, flat offset, power curve |
| `--photo-alpha` | `0.80` | Photo temporal weight on the previous frame (`0` = no memory); single still images have no prior frame to blend |
| `--photo-strength` | `0.90` | Photo diffused error fraction (`1.0` = classic Floyd-Steinberg) |
| `--photo-threshold` | `128` | Photo black/white decision point |
| `--photo-sharpen` | `0` | Photo unsharp-mask amount (`0` = off) |
| `--photo-blur` | `0` | Photo Gaussian sigma before diffusion (`0` = off) |
| `--photo-resize` | `area` | Photo downsampler: `area` averaging or the preview's `nearest`-neighbour resize |
| `--video-frames` | `12` | Number of evenly spaced source frames to sample from each supported video; sampled frames are processed in order |

## Notes

- When `photo` is selected for video input, sampled frames are processed in
  chronological order, carrying temporal state from each frame to the next
  within that clip. Separate clips and still images do not share prior-frame
  state.
- Videos are decoded with OpenCV. The lab requirements use
  `opencv-python-headless`, avoiding GUI libraries; supported containers and
  codecs depend on the OpenCV build.
- The `floyd` mode uses the same grayscale input and Floyd–Steinberg threshold
  and strength as the firmware, but the host simulator's image-source and
  letterboxing path can differ from the device. Canny's one-sided NMS is part
  of the lab-only implementation, not firmware.
- Input images are quantised to RGB565, expanded, then converted with the
  firmware's integer Rec.709 luma weights; the device decodes its SXGA JPEG
  stream at 1/8 scale first, so fine detail can differ from these full-resolution
  samples.
