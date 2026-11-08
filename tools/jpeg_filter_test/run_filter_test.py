#!/usr/bin/env python3
"""
Firmware capture-filter test bench.

Builds the REAL src/jpeg_filter.h (JPEG_FILTER_SUPERIA_USM) as a host binary
under WSL and runs it over the sample JPEGs, producing three filtered
outputs per image -- the shipped Superia 400 + USM combination, the Superia
look alone, and the USM alone -- plus an HTML gallery and contact sheets
for side-by-side inspection.

What this reproduces vs. hardware:
  reproduced  all filter math from src/jpeg_filter.h as compiled (LUTs,
              RGB565 canvas, 5/6/5-bit truncation/replication, the big-endian
              canvas byte order of decoder + encoder, tone chain order,
              row-ring USM, JPEG_QUALITY=10 handling)
  different   JPEG decoder/encoder codec: the device uses esp_new_jpeg +
              jpge; the host byte-exact stubs use stb. Expect comparable but
              not bit-identical JPEG compression behaviour around blocks.
              Memory-failure paths (4 MB PSRAM ceiling) are not exercised:
              the host stub allocates freely.

Usage:
  .venv/Scripts/python.exe tools/jpeg_filter_test/run_filter_test.py
  ...  --input test-images/img_0037.jpg  --quality 10
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

# mode -> (binary base name, sheet label)
MODES = {
    "superia": (1, "Superia 400 (firmware look only)"),
    "usm": (2, "USM (firmware sharpen only)"),
    "full": (0, "Superia 400 + USM (shipped firmware)"),
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
    for exe, (mode, _) in MODES.items():
        cmd = (f'cd {wsl_repo}/tools/jpeg_filter_test && '
               f"g++ -O2 -Wall -DFILTER_MODE={mode} -DJPEG_FILTER_SWAP565=1 "
               f"-I {wsl_repo}/src -I stubs host_test.cpp -o out/{exe}")
        r = subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True,
                           capture_output=True)
        if r.returncode != 0:
            print(f"compile {exe} failed:\n{r.stdout}{r.stderr}", file=sys.stderr)
            raise SystemExit(2)
        print(f"built out/{exe} (mode {mode})")


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
<title>firmware JPEG filter test -- Superia 400 / USM</title>
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
<h1>firmware JPEG filter test &mdash; Superia 400 / USM</h1>
<p class="hint">Output of the real src/jpeg_filter.h compiled for the host
(WSL), over the test captures. left to right: firmware Superia 400 look
alone, firmware USM alone, shipped Superia 400 + USM. Click any card for
full size; contact sheet links in each filename.</p>
{"".join(rows)}
</body>
</html>""", encoding="utf-8")
    return index


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", default=str(REPO_ROOT / "test-images"))
    parser.add_argument("--out", default=str(TOOL / "out"))
    parser.add_argument("--quality", type=int, default=10,
                        help="re-encode quality; 10 = firmware JPEG_QUALITY")
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

    groups, sheet_links = [], {}
    for image_path in images:
        try:
            original = Image.open(image_path).convert("RGB")
        except Exception as e:  # keep going on one bad input
            print(f"  skip {image_path.name}: {e}", file=sys.stderr)
            continue

        stem = image_path.stem
        rendered = []
        for exe, (mode, label) in MODES.items():
            out_name = f"{stem}__{exe}.jpg"
            cmd = (f"cd {as_wsl(TOOL)} && ./out/{exe} {as_wsl(image_path)} "
                   f"out/files/{out_name} {args.quality}")
            r = subprocess.run(["wsl", "-e", "bash", "-c", cmd], text=True,
                               capture_output=True)
            print(r.stdout.strip())
            if r.returncode != 0:
                print(f"  filter failed for {label}:\n{r.stdout}{r.stderr}",
                      file=sys.stderr)
                continue
            rendered.append((f"{mode:02d}", label, files / out_name))

        sheet = build_contact_sheet(
            original,
            [(num, label, Image.open(p).convert("RGB"))
             for num, label, p in rendered],
            cols=2, thumb_w=args.thumb_width)
        sheet_name = f"{stem}__sheet.jpg"
        sheet.save(sheets / sheet_name, quality=92)
        sheet_links[stem] = f"sheets/{sheet_name}"

        groups.append((image_path,
                       [(label, f"files/{p.name}") for _, label, p in rendered]))
        print()

    index = write_gallery(out_dir, groups)
    print(f"gallery -> {index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
