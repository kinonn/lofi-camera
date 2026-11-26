#!/usr/bin/env python3
"""
Firmware capture-filter test bench.

Builds the REAL src/jpeg_filter.h as host binaries under WSL and runs it
over the sample JPEGs, producing the runtime matrix: None (pass-through),
None + border (WITH_BORDER), the Superia/Kodachrome/T-Max/Portra looks, the
Superia look alone, the USM alone, and the four looks with the image
inset into a 24 px white border, plus an HTML gallery and contact sheets
for side-by-side inspection.

What this reproduces vs. hardware:
  reproduced  all filter math from src/jpeg_filter.h as compiled (LUTs,
              RGB565 canvas, 5/6/5-bit truncation/replication, the native
              little-endian decoder canvas and the pre-encode swap that the
              encoder's big-endian read expects, tone chain order,
              row-ring USM, JPEG_REENCODE_QUALITY handling); src/exif.h's
              APP1 layout/first-write/patch-offset contract, with the result
              reopened by Pillow to confirm ImageDescription parses
  different   JPEG decoder/encoder codec: the device uses esp_new_jpeg +
              jpge; the host byte-exact stubs use stb. Expect comparable but
              not bit-identical JPEG compression behaviour around blocks.
              Memory-failure paths (4 MB PSRAM ceiling) are not exercised:
              the host stub allocates freely.

Usage:
  .venv/Scripts/python.exe tools/jpeg_filter_test/run_filter_test.py
  ...  --input test-images/img_0134.jpg  --quality 80
Output: tools/jpeg_filter_test/out/gallery/index.html (git-ignored).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL = Path(__file__).resolve().parent

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "film_lab"))
from run_film_lab import build_contact_sheet, find_images  # noqa: E402

# mode -> (filter mode, filter kind, sheet label)
MODES = {
    "none": (0, 0, "None (pass-through)"),
    "with_border": (0, 1, "None + 24 px border (WITH_BORDER)"),
    "superia": (1, 2, "Superia 400 (firmware look only)"),
    "usm": (2, 2, "USM (firmware sharpen only)"),
    "full": (0, 2, "Superia 400 + USM (firmware)"),
    "kodachrome": (0, 3, "Kodachrome 64 + Vibrance + USM (firmware)"),
    "kodachrome_flat": (4, 3, "Kodachrome 64 + Vibrance + USM, S-curve off (before)"),
    "tmax": (0, 4, "T-Max 400 + USM (firmware)"),
    "tmax_flat": (3, 4, "T-Max 400 + USM, S-curve off (before)"),
    "portra": (0, 8, "Portra 400 + USM (firmware)"),
    "superia_border": (0, 5, "Superia 400 + USM + 24 px border"),
    "kodachrome_border": (0, 6, "Kodachrome 64 + Vibrance + USM + 24 px border"),
    "tmax_border": (0, 7, "T-Max 400 + USM + 24 px border"),
    "portra_border": (0, 9, "Portra 400 + USM + 24 px border"),
}


def as_wsl(p: Path) -> str:
    pos = p.resolve().as_posix()
    return "/mnt/" + pos[0].lower() + pos[2:]


def wsl(cmd: str) -> subprocess.CompletedProcess:
    return subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True)


def build(wsl_repo: str) -> None:
    if wsl("command -v g++").returncode != 0:
        print("error: g++ not found in WSL. Install with: sudo apt install g++",
              file=sys.stderr)
        raise SystemExit(2)
    for exe, (mode, kind, _) in MODES.items():
        cmd = (f'cd {wsl_repo}/tools/jpeg_filter_test && '
               f"g++ -O2 -Wall -DFILTER_MODE={mode} -DFILTER_KIND={kind} "
               f"-I {wsl_repo}/src -I stubs host_test.cpp -o out/{exe}")
        r = subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True,
                           capture_output=True)
        if r.returncode != 0:
            print(f"compile {exe} failed:\n{r.stdout}{r.stderr}", file=sys.stderr)
            raise SystemExit(2)
        print(f"built out/{exe} (kind {kind}, mode {mode})")


def check_exif(wsl_repo: str, sample_in: Path, sample_out: Path) -> None:
    """Compile/run the APP1 contract test plus a real-JPEG Pillow read."""
    cmd = (f'cd {wsl_repo}/tools/jpeg_filter_test && '
           f'g++ -O2 -Wall -I {wsl_repo}/src exif_test.cpp -o out/exif_test && '
           f'./out/exif_test {as_wsl(sample_in)} {as_wsl(sample_out)}')
    r = subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True,
                       capture_output=True)
    if r.returncode != 0:
        print(f"exif check failed:\n{r.stdout}{r.stderr}", file=sys.stderr)
        raise SystemExit(2)

    with Image.open(sample_out) as im:
        im.load()
        desc = im.getexif().get(0x010E)
    if isinstance(desc, bytes):
        desc = desc.decode("ascii", "replace")
    if not desc or "look=T-Max 400" not in desc:
        print(f"exif check failed: ImageDescription={desc!r}", file=sys.stderr)
        raise SystemExit(2)
    print(r.stdout.strip())


def write_gallery(out_dir: Path, groups) -> Path:
    rows = []
    for src, items in groups:
        cards = "".join(
            f'<figure class="card"><a href="{href}">'
            f'<img src="{href}" loading="lazy" alt="{label}"></a>'
            f'<figcaption>{label}</figcaption></figure>'
            for label, href in items
        )
        rows.append(f'<section><h2>{src.name}</h2><div class="grid">{cards}</div></section>')
    index = out_dir / "gallery" / "index.html"
    index.parent.mkdir(parents=True, exist_ok=True)
    index.write_text(f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>firmware JPEG filter test -- runtime looks and borders</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ margin: 0; padding: 24px 28px 64px; background: #141518;
         color: #e8e8ec; font: 14px/1.5 "Segoe UI", system-ui, sans-serif; }}
  h1 {{ font-size: 20px; margin: 0 0 4px; }}
  .hint {{ color: #9a9aa4; margin: 0 0 28px; max-width: 90ch; }}
  h2 {{ font-size: 15px; font-weight: 600; margin: 36px 0 12px;
       padding-bottom: 8px; border-bottom: 1px solid #2c2e34; }}
  .grid {{ display: grid; gap: 14px;
          grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); }}
  .card {{ margin: 0; background: #1c1e22; border-radius: 6px; overflow: hidden; }}
  .card img {{ display: block; width: 100%; height: auto; }}
  .card figcaption {{ padding: 7px 9px; font-size: 12px; color: #c9c9d2; }}
</style>
</head>
<body>
<h1>firmware JPEG filter test &mdash; runtime looks and borders</h1>
<p class="hint">Output of the real src/jpeg_filter.h compiled for the host
(WSL), over the test captures: None (pass-through), None + 24 px border
(WITH_BORDER), the Superia 400 look alone, the USM alone, Superia 400 +
USM, Kodachrome 64 + Vibrance + USM, T-Max 400 B&amp;W + USM, Portra 400 +
USM, and the same four looks with the image inset into a 24 px white
border. Click any card for full size; contact sheet links in each filename.</p>
{"".join(rows)}
</body>
</html>""", encoding="utf-8")
    return index


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", default=str(REPO_ROOT / "test-images"))
    parser.add_argument("--out", default=str(TOOL / "out"))
    parser.add_argument("--quality", type=int, default=80,
                        help="re-encode quality; default = firmware JPEG_REENCODE_QUALITY")
    parser.add_argument("--thumb-width", type=int, default=420)
    args = parser.parse_args(argv)

    src = Path(args.input)
    images = find_images(src)
    if not images:
        print(f"error: no images under {src}", file=sys.stderr)
        return 2

    out_dir = Path(args.out)
    files = out_dir / "files"
    sheets = out_dir / "sheets"
    for d in (out_dir, files, sheets):
        d.mkdir(parents=True, exist_ok=True)

    build(as_wsl(REPO_ROOT))
    check_exif(as_wsl(REPO_ROOT), images[0], out_dir / "exif_sample.jpg")

    groups, sheet_links = [], {}
    for image_path in images:
        try:
            original = Image.open(image_path).convert("RGB")
        except Exception as e:  # keep going on one bad input
            print(f"  skip {image_path.name}: {e}", file=sys.stderr)
            continue

        stem = image_path.stem
        rendered = []
        for exe, (mode, _kind, label) in MODES.items():
            out_name = f"{stem}__{exe}.jpg"
            cmd = (f"cd {as_wsl(TOOL)} && ./out/{exe} {as_wsl(image_path)} "
                   f"{as_wsl(files / out_name)} {args.quality}")
            r = subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True,
                               capture_output=True)
            print(r.stdout.strip())
            if r.returncode != 0:
                print(f"  filter failed for {label}:\n{r.stdout}{r.stderr}",
                      file=sys.stderr)
                continue
            rendered.append((f"{len(rendered):02d}", label, files / out_name))

        sheet = build_contact_sheet(
            original,
            [(num, label, Image.open(p).convert("RGB"))
             for num, label, p in rendered],
            cols=2, thumb_w=args.thumb_width)
        sheet_name = f"{stem}__sheet.jpg"
        sheet.save(sheets / sheet_name, quality=92)
        sheet_links[stem] = f"sheets/{sheet_name}"

        groups.append((image_path,
                       [(label, f"../files/{p.name}") for _, label, p in rendered]))
        print()

    index = write_gallery(out_dir, groups)
    print(f"gallery -> {index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
