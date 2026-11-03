"""OLED preview test bench for lofi-camera.

Simulates the firmware Floyd-Steinberg path and lab-only alternatives,
including Bayer and Canny, over sample images or evenly sampled video frames
without touching firmware.

Pipeline per image or sampled video frame (mirrors the device)::

    RGB -> Rec.709 luma -> aspect-fit letterbox (nearest neighbour)
    -> Floyd-Steinberg or a lab-only alternative

Examples:
    python tools/preview_lab/run_preview_lab.py
    python tools/preview_lab/run_preview_lab.py --list
    python tools/preview_lab/run_preview_lab.py --modes dither
    python tools/preview_lab/run_preview_lab.py --input test-images/img_0027.jpg
    python tools/preview_lab/run_preview_lab.py --input clip.mp4 --modes photo --photo-alpha 0.8
    python tools/preview_lab/run_preview_lab.py --dither-contrast 130 --blur-passes 2 --low 80 --high 160
"""

from __future__ import annotations

import argparse
import html as html_lib
import sys
import tempfile
from pathlib import Path
from urllib.parse import quote

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))

from preview_sim import (  # noqa: E402
    SCREEN_WIDTH,
    PreviewFrame,
    area_resize,
    atkinson_render,
    blur_atkinson_render,
    canny_render,
    clahe_floyd_render,
    dither_bayer2_render,
    dither_bayer8_render,
    dither_render,
    floyd_steinberg_render,
    photo_floyd_render,
    preview_prepare,
    rgb_to_luma,
    threshold_render,
)

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
VIDEO_SUFFIXES = {".avi", ".m4v", ".mkv", ".mov", ".mp4", ".mpeg", ".mpg", ".webm"}

BG = (26, 27, 30)
LABEL_BG = (38, 40, 45)
TEXT = (232, 232, 236)
ACCENT = (255, 196, 96)

MODES = ("dither", "bayer2", "bayer8", "floyd", "atkinson", "threshold",
         "clahe-floyd", "blur-atkinson", "photo", "canny")
MODE_HELP = {
    "dither": "lab-only 4x4 Bayer ordered dither",
    "bayer2": "lab-only 2x2 Bayer (tight dots, only 4 gray steps)",
    "bayer8": "lab-only 8x8 Bayer (finer tones, coarser dots)",
    "floyd": "firmware serpentine Floyd-Steinberg (threshold=128, strength=0.45)",
    "atkinson": "lab-only Atkinson error diffusion (punchy, posterised)",
    "threshold": "lab-only plain threshold (stark black/white)",
    "clahe-floyd": "lab-only CLAHE -> slight Gaussian blur -> Floyd-Steinberg",
    "blur-atkinson": "lab-only slight Gaussian blur -> Atkinson",
    "photo": "lab-only area resize + tone + Floyd-Steinberg",
    "canny": "lab-only Canny edge detection (blur passes=2, low=80, high=160)",
}


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def find_images(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(
        p for p in path.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES | VIDEO_SUFFIXES
    )


def sample_video(path: Path, frame_count: int, output_dir: Path) -> list[tuple[Path, int, int]]:
    """Extract evenly spaced video frames to temporary PNGs.

    Returns each temporary frame path with its zero-based source frame index
    and the video's reported total frame count.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        capture.release()
        raise ValueError(f"could not open video: {path}")

    try:
        total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if total_frames <= 0:
            raise ValueError(f"video has no readable frames: {path}")
        sample_count = min(frame_count, total_frames)
        indices = np.linspace(0, total_frames - 1, num=sample_count, dtype=np.int64)
        samples: list[tuple[Path, int, int]] = []
        for sample_number, frame_index in enumerate(indices):
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(frame_index))
            ok, bgr = capture.read()
            if not ok:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            frame_path = output_dir / f"frame_{sample_number:04d}.png"
            Image.fromarray(rgb).save(frame_path)
            samples.append((frame_path, int(frame_index), total_frames))
        if not samples:
            raise ValueError(f"could not decode sampled frames from video: {path}")
        return samples
    finally:
        capture.release()


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in ("arial.ttf", "segoeui.ttf", "calibri.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def upscale_nearest(img: Image.Image, scale: int) -> Image.Image:
    """Magnify a 128x64 1-bit preview so dots/edges are visible."""
    scale = max(1, int(scale))
    if scale == 1:
        return img
    return img.resize(
        (img.width * scale, img.height * scale), Image.Resampling.NEAREST
    )


def positive_int(value: str) -> int:
    """Argparse type that rejects values unusable as dimensions/counts."""
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def unique_stem(path: Path, seen: set[str]) -> str:
    """Build a filesystem-safe unique output key for an input image."""
    base = path.stem
    candidate = base
    if candidate.casefold() in seen:
        candidate = f"{base}__{path.suffix.lstrip('.') or 'image'}"
    serial = 2
    while candidate.casefold() in seen:
        candidate = f"{base}__{path.suffix.lstrip('.') or 'image'}_{serial}"
        serial += 1
    seen.add(candidate.casefold())
    return candidate


def save_preview(out_dir: Path, slug: str, stem: str, mono: np.ndarray,
                 scale: int) -> tuple[str, Image.Image, Image.Image]:
    """Save 1x + scaled PNGs, return (href, small, big)."""
    small = Image.fromarray(mono, mode="L")
    big = upscale_nearest(small, scale)
    folder = out_dir / slug
    folder.mkdir(parents=True, exist_ok=True)
    small.save(folder / f"{stem}__{slug}.png")
    big.save(folder / f"{stem}__{slug}_x{scale}.png")
    return f"{slug}/{stem}__{slug}_x{scale}.png", small, big


def build_contact_sheet(
    original: Image.Image,
    rendered: list[tuple[str, str, Image.Image]],
    cols: int = 3,
    thumb_w: int = 512,
) -> Image.Image:
    """Grid of ``original`` first, then every (number, label, image) entry."""
    label_h, gap = 30, 10
    cells: list[tuple[str, Image.Image, bool]] = [("00  ORIGINAL", original, True)]
    for number, label, img in rendered:
        cells.append((f"{number}  {label}", img, False))

    def _thumb_h(img: Image.Image) -> int:
        return max(1, round(thumb_w * img.height / img.width))

    prepared = [
        (label,
         img.resize((thumb_w, _thumb_h(img)),
                    Image.Resampling.NEAREST
                    if img.width <= SCREEN_WIDTH * 2
                    else Image.Resampling.LANCZOS),
         is_original)
        for label, img, is_original in cells
    ]
    cell_h = max(pic.height for _, pic, _ in prepared) + label_h
    rows = (len(prepared) + cols - 1) // cols

    sheet_w = gap + cols * (thumb_w + gap)
    sheet_h = gap + rows * (cell_h + gap)
    sheet = Image.new("RGB", (sheet_w, sheet_h), BG)

    draw = ImageDraw.Draw(sheet)
    small = load_font(max(12, label_h - 13))
    title = load_font(max(13, label_h - 12))

    for i, (label, pic, is_original) in enumerate(prepared):
        r, c = divmod(i, cols)
        x = gap + c * (thumb_w + gap)
        y = gap + r * (cell_h + gap)

        sheet.paste(pic, (x, y))

        draw.rectangle(
            [x, y + pic.height, x + thumb_w, y + cell_h],
            fill=LABEL_BG,
        )
        colour = ACCENT if is_original else TEXT
        draw.text(
            (x + 6, y + pic.height + 7),
            label,
            fill=colour,
            font=title if is_original else small,
        )

    return sheet


def write_gallery(
    out_dir: Path,
    groups: list[tuple[str, list[tuple[str, str, str]], str]],
    sheet_paths: dict[str, str],
) -> Path:
    """Plain static HTML so every preview mode can be flipped through."""
    rows = []
    for source_label, items, output_key in groups:
        cards = [
            f'<figure class="card"><a href="{html_lib.escape(quote(href, safe="/"), quote=True)}">'
            f'<img src="{html_lib.escape(quote(href, safe="/"), quote=True)}" loading="lazy" alt="{html_lib.escape(label, quote=True)}" '
            f'style="image-rendering: pixelated;"></a>'
            f'<figcaption><b>{html_lib.escape(number)}</b> {html_lib.escape(label)}</figcaption></figure>'
            for number, label, href in items
        ]
        sheet = sheet_paths.get(output_key)
        sheet_link = (
            f'<a class="sheet" href="{html_lib.escape(quote(sheet, safe="/"), quote=True)}">contact sheet &rsaquo;</a>'
            if sheet else ""
        )
        rows.append(
            f'<section><h2>{html_lib.escape(source_label)} {sheet_link}</h2>'
            f'<div class="grid">{"".join(cards)}</div></section>'
        )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>lofi-cam preview bench</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ margin: 0; padding: 24px 28px 64px; background: #141518;
         color: #e8e8ec; font: 14px/1.5 "Segoe UI", system-ui, sans-serif; }}
  h1 {{ font-size: 20px; margin: 0 0 4px; letter-spacing: .01em; }}
  .hint {{ color: #9a9aa4; margin: 0 0 28px; }}
  h2 {{ font-size: 15px; font-weight: 600; margin: 36px 0 12px;
       padding-bottom: 8px; border-bottom: 1px solid #2c2e34; }}
  .sheet {{ font-weight: 400; font-size: 12px; color: #ffc460;
           text-decoration: none; margin-left: 10px; }}
  .grid {{ display: grid; gap: 14px;
          grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); }}
  .card {{ margin: 0; background: #1c1e22; border-radius: 6px; overflow: hidden; }}
  .card img {{ display: block; width: 100%; height: auto; image-rendering: pixelated; }}
  .card figcaption {{ padding: 7px 9px; font-size: 12px; color: #c9c9d2; }}
  .card figcaption b {{ color: #ffc460; font-variant-numeric: tabular-nums; }}
</style>
</head>
<body>
<h1>lofi-cam &mdash; preview bench</h1>
<p class="hint">Firmware Floyd-Steinberg and lab-only Bayer/Canny previews over sample captures. Click any
thumbnail for full size. The <code>floyd</code> mode uses firmware threshold and strength defaults; Canny is lab-only.</p>
{"".join(rows)}
</body>
</html>
"""
    index = out_dir / "index.html"
    index.write_text(html, encoding="utf-8")
    return index


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Simulate lofi-cam OLED preview modes over sample images and videos.",
    )
    parser.add_argument("--input", default=str(repo_root / "test-images"),
                        help="image/video file or folder (default: test-images/)")
    parser.add_argument("--video-frames", type=positive_int, default=12,
                        help="evenly spaced frames to sample from each video (default: 12)")
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent / "out"),
                        help="output folder (default: tools/preview_lab/out)")
    parser.add_argument("--modes", nargs="+", default=list(MODES),
                        choices=list(MODES),
                        help="preview modes to render (default: all)")
    parser.add_argument("--list", action="store_true",
                        help="list the available preview modes and exit")
    parser.add_argument("--dither-contrast", type=int, default=130,
                        help="DITHER_CONTRAST (default: 130, 100 = unchanged)")
    parser.add_argument("--photo-contrast", type=float, default=1.15,
                        help="photo mode contrast gain around mid-grey (default: 1.15)")
    parser.add_argument("--photo-brightness", type=float, default=0.0,
                        help="photo mode brightness offset in gray levels (default: 0)")
    parser.add_argument("--photo-gamma", type=float, default=1.0,
                        help="photo mode gamma power curve (default: 1.0 = off)")
    parser.add_argument("--photo-alpha", type=float, default=0.8,
                        help="photo mode temporal weight on previous frame (default: 0.80)")
    parser.add_argument("--photo-strength", type=float, default=0.9,
                        help="photo mode diffused error fraction (default: 0.90)")
    parser.add_argument("--photo-threshold", type=float, default=128.0,
                        help="photo mode black/white decision point (default: 128)")
    parser.add_argument("--photo-sharpen", type=float, default=0.0,
                        help="photo mode unsharp-mask amount (default: 0 = off)")
    parser.add_argument("--photo-blur", type=float, default=0.0,
                        help="photo mode Gaussian sigma before diffusion (default: 0 = off)")
    parser.add_argument("--photo-resize", choices=["area", "nearest"], default="area",
                        help="photo mode downsampler (default: area)")
    parser.add_argument("--clahe-clip", type=float, default=2.0,
                        help="CLAHE clip limit for clahe-floyd (default: 2.0)")
    parser.add_argument("--clahe-tiles", type=int, nargs=2, default=[8, 8],
                        metavar=("ROWS", "COLS"),
                        help="CLAHE tile grid for clahe-floyd (default: 8 8)")
    parser.add_argument("--clahe-blur", type=float, default=0.8,
                        help="Gaussian sigma before Floyd-Steinberg (default: 0.8, 0 = off)")
    parser.add_argument("--clahe-contrast", type=int, default=100,
                        help="contrast stretch before CLAHE (default: 100 = unchanged)")
    parser.add_argument("--blur-passes", type=int, default=2,
                        help="lab-only Canny blur passes (default: 2, 0 = off)")
    parser.add_argument("--low", type=int, default=80,
                        help="lab-only Canny weak-edge threshold (default: 80)")
    parser.add_argument("--high", type=int, default=160,
                        help="lab-only Canny strong-edge threshold (default: 160)")
    parser.add_argument("--preview-scale", type=positive_int, default=4,
                        help="magnify 128x64 previews for viewing (default: 4)")
    parser.add_argument("--cols", type=positive_int, default=3,
                        help="contact sheet columns (default: 3)")
    parser.add_argument("--thumb-width", type=positive_int, default=512,
                        help="contact sheet thumbnail width (default: 512)")
    parser.add_argument("--no-sheet", action="store_true",
                        help="skip contact sheets and the HTML gallery")
    args = parser.parse_args(argv)

    if args.list:
        print("preview modes:\n")
        for m in MODES:
            print(f"  {m:13s} {MODE_HELP[m]}")
        return 0

    src_dir = Path(args.input)
    if not src_dir.exists():
        print(f"error: input not found: {src_dir}", file=sys.stderr)
        return 2

    images = find_images(src_dir)
    if not images:
        print(f"error: no images in {src_dir}", file=sys.stderr)
        return 2

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    temp_frames = tempfile.TemporaryDirectory(prefix="lofi-preview-video-")
    inputs: list[tuple[Path, str, str, str | None, int | None, int | None]] = []
    try:
        for image_path in images:
            if image_path.suffix.lower() not in VIDEO_SUFFIXES:
                inputs.append((image_path, image_path.name, image_path.stem,
                               None, None, None))
                continue
            video_key = str(image_path.resolve())
            video_dir = Path(temp_frames.name) / str(len(inputs))
            samples = sample_video(image_path, args.video_frames, video_dir)
            for frame_path, frame_index, total_frames in samples:
                frame_stem = f"{image_path.stem}__frame_{frame_index:06d}"
                frame_label = (f"{image_path.name} — frame {frame_index + 1}"
                               f"/{total_frames}")
                inputs.append((frame_path, frame_label, frame_stem,
                               video_key, frame_index, total_frames))
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        temp_frames.cleanup()
        return 2

    print(f"input   : {src_dir}")
    print(f"output  : {out_dir}")
    print(f"sources : {len(images)}   sampled frames: {len(inputs)}   modes: {', '.join(args.modes)}")
    print(f"video   : up to {args.video_frames} evenly spaced frames per clip")
    print(f"preview : contrast={args.dither_contrast}")
    print(f"photo   : contrast={args.photo_contrast} brightness={args.photo_brightness} "
          f"gamma={args.photo_gamma} alpha={args.photo_alpha} "
          f"strength={args.photo_strength} resize={args.photo_resize} "
          f"threshold={args.photo_threshold} sharpen={args.photo_sharpen} blur={args.photo_blur}")
    print(f"clahe   : contrast={args.clahe_contrast} clip={args.clahe_clip} "
          f"tiles={args.clahe_tiles[0]}x{args.clahe_tiles[1]} blur={args.clahe_blur}")
    print(f"canny   : blur_passes={args.blur_passes} low={args.low} high={args.high}\n")

    groups: list[tuple[str, list[tuple[str, str, str]], str]] = []
    sheet_paths: dict[str, str] = {}
    used_output_keys: set[str] = set()
    photo_state: dict[str, np.ndarray] = {}

    try:
        for image_path, source_label, output_stem, video_key, _, _ in inputs:
            with Image.open(image_path) as handle:
                rgb = handle.convert("RGB")

            luma = rgb_to_luma(np.asarray(rgb))
            frame = preview_prepare(luma)

            stem = unique_stem(Path(output_stem), used_output_keys)
            items: list[tuple[str, str, str]] = []
            rendered: list[tuple[str, str, Image.Image]] = []

            def render(mode: str) -> tuple[str, np.ndarray, str]:
                """Return (slug, mono, label) for one mode."""
                dc = args.dither_contrast
                if mode == "dither":
                    return "dither", dither_render(frame, contrast=dc), f"dither (contrast {dc})"
                if mode == "bayer2":
                    return "bayer2", dither_bayer2_render(frame, contrast=dc), f"bayer2 (contrast {dc})"
                if mode == "bayer8":
                    return "bayer8", dither_bayer8_render(frame, contrast=dc), f"bayer8 (contrast {dc})"
                if mode == "floyd":
                    mono = floyd_steinberg_render(
                        frame, contrast=dc, threshold=128, strength=0.45)
                    return "floyd-steinberg", mono, (
                        f"floyd-steinberg (contrast {dc}, threshold 128, strength 0.45)")
                if mode == "atkinson":
                    return "atkinson", atkinson_render(frame, contrast=dc), f"atkinson (contrast {dc})"
                if mode == "threshold":
                    return "threshold", threshold_render(frame, contrast=dc), f"threshold (contrast {dc})"
                if mode == "clahe-floyd":
                    mono = clahe_floyd_render(
                        frame, contrast=args.clahe_contrast, clip_limit=args.clahe_clip,
                        tiles=(args.clahe_tiles[0], args.clahe_tiles[1]), sigma=args.clahe_blur)
                    return "clahe-floyd", mono, (f"clahe-floyd (contrast {args.clahe_contrast}, clip {args.clahe_clip}, "
                        f"tiles {args.clahe_tiles[0]}x{args.clahe_tiles[1]}, blur {args.clahe_blur})")
                if mode == "blur-atkinson":
                    mono = blur_atkinson_render(frame, contrast=dc, sigma=args.clahe_blur)
                    return "blur-atkinson", mono, f"blur-atkinson (contrast {dc}, blur {args.clahe_blur})"
                if mode == "photo":
                    if args.photo_resize == "area":
                        small_gray = area_resize(luma, frame.w, frame.h)
                        photo_frame = PreviewFrame(px=small_gray, w=frame.w,
                                                   h=frame.h, x=frame.x, y=frame.y)
                    else:
                        photo_frame = frame
                    mono, stabilised = photo_floyd_render(
                        photo_frame, contrast=args.photo_contrast,
                        brightness=args.photo_brightness, gamma=args.photo_gamma,
                        sharpen_amount=args.photo_sharpen, blur_sigma=args.photo_blur,
                        threshold=args.photo_threshold, strength=args.photo_strength,
                        previous=photo_state.get(video_key) if video_key else None,
                        alpha=args.photo_alpha)
                    if video_key:
                        photo_state[video_key] = stabilised
                    return "photo", mono, (f"photo (contrast {args.photo_contrast}, bright {args.photo_brightness}, "
                        f"gamma {args.photo_gamma}, alpha {args.photo_alpha}, strength {args.photo_strength}, "
                        f"thr {args.photo_threshold}, sharpen {args.photo_sharpen}, "
                        f"blur {args.photo_blur}, {args.photo_resize})")
                raise ValueError(mode)

            number = 0
            for mode in MODES:
                if mode == "canny" or mode not in args.modes:
                    continue
                number += 1
                slug, mono, label = render(mode)
                href, _, big = save_preview(out_dir, slug, stem, mono, args.preview_scale)
                num = f"{number:02d}"
                items.append((num, label, href))
                print(f"  {stem}  <- {num} {label}")
                if not args.no_sheet:
                    rendered.append((num, label, big.convert("RGB")))

            if "canny" in args.modes:
                mono = canny_render(
                    frame,
                    blur_passes=args.blur_passes,
                    low=args.low,
                    high=args.high,
                )
                href, _, big = save_preview(out_dir, "canny", stem, mono, args.preview_scale)

                label = (f"canny (blur {args.blur_passes}, "
                         f"low {args.low}, high {args.high})")
                num = f"{len(items) + 1:02d}"
                items.append((num, label, href))
                print(f"  {stem}  <- {num} {label}")
                if not args.no_sheet:
                    rendered.append((num, label, big.convert("RGB")))

            groups.append((source_label, items, stem))

            if not args.no_sheet:
                sheet = build_contact_sheet(
                    rgb, rendered, cols=args.cols, thumb_w=args.thumb_width,
                )
                sheet_dir = out_dir / "contact_sheets"
                sheet_dir.mkdir(parents=True, exist_ok=True)
                sheet_path = sheet_dir / f"{stem}__contact_sheet.jpg"
                sheet.save(sheet_path, quality=92, subsampling=0)
                sheet_paths[stem] = f"contact_sheets/{sheet_path.name}"
                print(f"  sheet   -> {sheet_path.relative_to(out_dir)}")

            print()
    finally:
        temp_frames.cleanup()

    if not args.no_sheet:
        index = write_gallery(out_dir, groups, sheet_paths)
        print(f"gallery -> {index}")

        print("\nDone. Open the gallery to compare the preview modes; tune via "
              "--dither-contrast / --blur-passes / --low / --high.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
