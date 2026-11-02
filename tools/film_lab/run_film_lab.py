"""
Film look test bench for lofi-cam.

Renders every filter in ``film_emulation.py`` over the sample JPEGs in
``test-images/``, writes one full-size result per (image, filter) pair, builds a
labelled contact sheet per source image, and an HTML gallery for A/B comparing.

Nothing here touches the firmware; it is a pure host-side experiment.

Examples:
    python tools/film_lab/run_film_lab.py                # everything
    python tools/film_lab/run_film_lab.py --list         # show every look
    python tools/film_lab/run_film_lab.py --filters 3 12 kodak_gold_200
    python tools/film_lab/run_film_lab.py --input test-images/img_0027.jpg
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))

from film_emulation import (  # noqa: E402
    FILTERS,
    FILTER_ORDER,
    apply_filter,
    get_filter,
)

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}

BG = (26, 27, 30)
LABEL_BG = (38, 40, 45)
TEXT = (232, 232, 236)
ACCENT = (255, 196, 96)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def find_images(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(
        p for p in path.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
    )


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for name in ("arial.ttf", "segoeui.ttf", "calibri.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def thumb(img: Image.Image, width: int) -> Image.Image:
    height = max(1, round(width * img.height / img.width))
    return img.resize((width, height), Image.Resampling.LANCZOS)


def build_contact_sheet(
    original: Image.Image,
    rendered: list[tuple[str, str, Image.Image]],
    cols: int = 4,
    thumb_w: int = 340,
    label_h: int = 30,
    gap: int = 10,
) -> Image.Image:
    """Grid of ``original`` first, then every (number, label, image) entry."""
    cells: list[tuple[str, Image.Image, bool]] = [("00  ORIGINAL", original, True)]
    for number, label, img in rendered:
        cells.append((f"{number}  {label}", img, False))

    prepared = [(label, thumb(img, thumb_w), is_original)
                for label, img, is_original in cells]
    # Cells follow the tallest thumbnail, so mixed aspect ratios (e.g. a
    # bordered Polaroid render) cannot overflow into the label bar.
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
        draw.text((x + 6, y + pic.height + 7), label, fill=colour, font=title if is_original else small)

    return sheet


def write_gallery(
    out_dir: Path,
    groups: list[tuple[Path, list[tuple[str, str, str]]]],
    sheet_paths: dict[str, str],
) -> Path:
    """Plain static HTML so every look can be flipped through in a browser."""
    rows = []
    for src, items in groups:
        cards = [
            f'<figure class="card"><a href="{href}">'
            f'<img src="{href}" loading="lazy" alt="{label}"></a>'
            f'<figcaption><b>{number}</b> {label}</figcaption></figure>'
            for number, label, href in items
        ]
        sheet = sheet_paths.get(src.stem)
        sheet_link = (
            f'<a class="sheet" href="{sheet}">contact sheet &rsaquo;</a>' if sheet else ""
        )
        rows.append(
            f'<section><h2>{src.name} {sheet_link}</h2>'
            f'<div class="grid">{"".join(cards)}</div></section>'
        )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>lofi-cam film look bench</title>
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
  .card img {{ display: block; width: 100%; height: auto; }}
  .card figcaption {{ padding: 7px 9px; font-size: 12px; color: #c9c9d2; }}
  .card figcaption b {{ color: #ffc460; font-variant-numeric: tabular-nums; }}
  .orig {{ outline: 2px solid #ffc460; }}
</style>
</head>
<body>
<h1>lofi-cam &mdash; film look bench</h1>
<p class="hint">{len(FILTER_ORDER)} emulations over the sample captures. Click any thumbnail for
full size. Pick numbers, then port the winner to the firmware.</p>
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
        description="Render film-emulation looks over lofi-cam sample images.",
    )
    parser.add_argument("--input", default=str(repo_root / "test-images"),
                        help="image file or folder (default: test-images/)")
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent / "out"),
                        help="output folder (default: tools/film_lab/out)")
    parser.add_argument("--filters", nargs="+", metavar="FILTER",
                        help="slugs or 1-based numbers; omit for all")
    parser.add_argument("--family", metavar="FAMILY",
                        help="render only one family, e.g. CF, Kodak, Fujifilm")
    parser.add_argument("--list", action="store_true",
                        help="list the available filters and exit")
    parser.add_argument("--cols", type=int, default=4,
                        help="contact sheet columns (default: 4)")
    parser.add_argument("--thumb-width", type=int, default=340,
                        help="contact sheet thumbnail width (default: 340)")
    parser.add_argument("--quality", type=int, default=95, help="JPEG quality")
    parser.add_argument("--no-sheet", action="store_true",
                        help="skip contact sheets and the HTML gallery")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="resize output by this factor (default: 1.0)")
    args = parser.parse_args(argv)

    if args.list:
        print(f"{len(FILTER_ORDER)} film looks:\n")
        for name in FILTER_ORDER:
            p = FILTERS[name]
            print(f"  {p.order:2d}. {p.name:<24s} {p.label:<28s} [{p.family}]")
            print(f"      {p.description}")
        return 0

    src_dir = Path(args.input)
    if not src_dir.exists():
        print(f"error: input not found: {src_dir}", file=sys.stderr)
        return 2

    images = find_images(src_dir)
    if not images:
        print(f"error: no images in {src_dir}", file=sys.stderr)
        return 2

    if args.filters:
        try:
            presets = [get_filter(f) for f in args.filters]
        except KeyError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    elif args.family:
        presets = [FILTERS[n] for n in FILTER_ORDER
                   if FILTERS[n].family.lower() == args.family.lower()]
        if not presets:
            print(f"error: no filters in family {args.family!r}", file=sys.stderr)
            return 2
    else:
        presets = [FILTERS[n] for n in FILTER_ORDER]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"input   : {src_dir}")
    print(f"output  : {out_dir}")
    print(f"images  : {len(images)}   filters: {len(presets)}\n")

    groups: list[tuple[Path, list[tuple[str, str, str]]]] = []
    sheet_paths: dict[str, str] = {}

    for image_path in images:
        with Image.open(image_path) as handle:
            original = handle.convert("RGB")
        if args.scale != 1.0:
            original = original.resize(
                (max(1, round(original.width * args.scale)),
                 max(1, round(original.height * args.scale))),
                Image.Resampling.LANCZOS,
            )

        stem = image_path.stem
        items: list[tuple[str, str, str]] = []

        for preset in presets:
            result = apply_filter(original, preset)

            folder = out_dir / preset.name
            folder.mkdir(parents=True, exist_ok=True)
            filename = f"{stem}__{preset.name}.jpg"
            result.save(folder / filename, quality=args.quality, subsampling=0)

            items.append((f"{preset.order:02d}", preset.label,
                          f"{preset.name}/{filename}"))
            print(f"  {stem}  <- {preset.order:02d} {preset.label}")

        groups.append((image_path, items))

        if not args.no_sheet:
            rendered = []
            for preset in presets:
                path = out_dir / preset.name / f"{stem}__{preset.name}.jpg"
                with Image.open(path) as handle:
                    rendered.append((f"{preset.order:02d}", preset.label,
                                     handle.convert("RGB")))
            sheet = build_contact_sheet(
                original, rendered, cols=args.cols, thumb_w=args.thumb_width,
            )
            sheet_dir = out_dir / "contact_sheets"
            sheet_dir.mkdir(parents=True, exist_ok=True)
            sheet_path = sheet_dir / f"{stem}__contact_sheet.jpg"
            sheet.save(sheet_path, quality=92, subsampling=0)
            sheet_paths[stem] = f"contact_sheets/{sheet_path.name}"
            print(f"  sheet   -> {sheet_path.relative_to(out_dir)}")

        print()

    if not args.no_sheet:
        index = write_gallery(out_dir, groups, sheet_paths)
        print(f"gallery -> {index}")

    print("\nDone. Open the gallery, note the numbers you like, then ask to port them.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
