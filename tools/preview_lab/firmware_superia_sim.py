#!/usr/bin/env python3
"""Host replica of the JPEG_FILTER_SUPERIA_USM firmware filter -- and of the
RGB565 byte-order story around it.

Proven from the linked framework (esp32-camera in Arduino-ESP32 2.0.17,
source v2.0.16 -- same as-built layout, checked in the shipped archive):

  decode  jpg2rgb565() stores the canvas little-endian standard R5G6B5:
          out[0] = v & 0xFF, out[1] = v >> 8 with v = R5<<11 | G6<<5 | B5.
          A native u16 read therefore holds the true fields -- no swap.
  encode  fmt2jpg()'s convert_image() reads RGB565 input unconditionally
          big-endian: byte 0 is the high byte, expanded to RGB888 with zero
          extension -- R = b0 & 0xF8, G = (b0 & 0x07) << 5 | (b1 & 0xE0) >> 3,
          B = (b1 & 0x1F) << 3.

So the canvas needs exactly one byte-swap pass before encode. Variants:

  asbuilt  the old firmware: swap-first unpack/repack (it processed a bit
           salad) and no pre-encode swap; the big-endian encoder then
           rendered that salad. Reproduces the rainbow captures.
  fixed    the current firmware: native unpack/pack plus one swap pass
           before encode; the encoder reads true fields (RGB565 quantised
           and zero-extended -- the accepted encoder coarseness ceiling,
           same in every re-encoding filter path, not compensated).
  lab_ref  the pure 8-bit float lab chain, no RGB565 step at all.

Usage:
  python firmware_superia_sim.py [input.jpg] [--stages]
Output goes to out/superia_sim/ (git-ignored).
"""
import numpy as np
from PIL import Image
from pathlib import Path

OUT = Path(__file__).parent / "out" / "superia_sim"
SCALE = 2  # render at half resolution for quick visual comparison


def srgb_to_lin(x):
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def lin_to_srgb(x):
    x = np.maximum(x, 0)
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * x ** (1 / 2.4) - 0.055)

AUTO_BALANCE = 0.55
AUTO_CONTRAST = 0.56
WB = (0.98, 1.01, 1.00)
MIDTONE_GAMMA = 0.97
SHADOW = (-0.010, 0.010, 0.010)
HIGHLIGHT = (0.020, 0.014, -0.006)
VIGNETTE = 0.50
USM_AMOUNT = 0.90
USM_SIGMA = 1.00
USM_MAX_RADIUS = 16


# ---------------------------------------------------------------- bit helpers

def swap16(v):
    """Byte swap a u16 array."""
    v = v.astype(np.uint32)
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def _fields(v):
    """Standard R5G6B5 field values of a u16 array."""
    v = v.astype(np.uint32)
    return (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F


def _replicate(r5, g6, b5):
    """Firmware unpack expansion: 5/6-bit fields -> 8 bits by replication."""
    r8 = ((r5 << 3) | (r5 >> 2)) & 0xFF
    g8 = ((g6 << 2) | (g6 >> 4)) & 0xFF
    b8 = ((b5 << 3) | (b5 >> 2)) & 0xFF
    return r8, g8, b8


def _zero_extend(r5, g6, b5):
    """fmt2jpg expansion: fields -> RGB888 by zero extension (coarse)."""
    return r5 << 3, g6 << 2, b5 << 3


def pack565_std(r8, g8, b8):
    """Standard field layout u16: R5<<11 | G6<<5 | B5."""
    r8, g8, b8 = (x.astype(np.uint32) for x in (r8, g8, b8))
    return ((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b8 >> 3)


# The decode canvas (jpg2rgb565 output) holds the photo as LE standard fields:
def unpack_native(v):
    """Current firmware unpack: read the canvas natively (true fields)."""
    return _replicate(*_fields(v))


def unpack_asbuilt(v):
    """Old firmware unpack: byte swap FIRST, then standard fields (salad)."""
    return _replicate(*_fields(swap16(v)))


def encoder_read565(w_stored):
    """What fmt2jpg() extracts from a canvas whose u16 (native) value is
    w_stored: it reads the memory bytes big-endian and zero-extends."""
    return _zero_extend(*_fields(swap16(w_stored)))


def _self_check():
    """One bounded algebra check for the whole unpack/pack/swap story."""
    v = np.arange(65536, dtype=np.uint32)
    assert np.array_equal(pack565_std(*unpack_native(v)), v), "native round trip"
    # the old firmware's salad repacks into the swapped u16 (symmetric salad)
    assert np.array_equal(pack565_std(*unpack_asbuilt(v)), swap16(v)), "salad rt"
    # zero-extension expansion keeps channels inside the 565 palette
    r, g, b = _zero_extend((v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F)
    assert (r % 8 == 0).all() and (g % 4 == 0).all() and (b % 8 == 0).all()


# ---------------------------------------------------------------- look chain

def apply_look(r8, g8, b8, w, h, dumps=None):
    """The fused look chain in float straight from u8 canvas planes.
    Dumps per-stage copies of f (float, [0..~1.5]) when diagnostics run."""
    f = np.stack([r8, g8, b8], axis=-1).astype(np.float64) / 255.0
    if dumps is not None:
        stages = dumps.setdefault("look", [])

        def snap(name):
            stages.append((name, f.copy(), dumps))
    else:
        def snap(name):
            pass
    snap("balance:in")
    luma = 0.2126 * f[..., 0] + 0.7152 * f[..., 1] + 0.0722 * f[..., 2]

    mask = (luma > 0.10) & (luma < 0.90)
    if mask.any() and mask.sum() >= 200:
        means = f[mask].mean(axis=0)
        target = means.mean()
        gains = 1.0 + (target / np.maximum(means, 1e-4) - 1.0) * AUTO_BALANCE
        for c in range(3):
            f[..., c] *= gains[c]
    snap("balance:out")

    luma = 0.2126 * f[..., 0] + 0.7152 * f[..., 1] + 0.0722 * f[..., 2]
    bins = np.clip((luma * (2048 / 1.5)).astype(np.int32), 0, 2047)
    hist = np.bincount(bins.ravel(), minlength=2048)
    cum = np.cumsum(hist)
    n = w * h
    lo_i = np.searchsorted(cum, int(0.01 * (n - 1)) + 1, side="left")
    hi_i = np.searchsorted(cum, int(0.99 * (n - 1)) + 1, side="left")
    lo = (lo_i + 0.5) / 2048 * 1.5
    hi = (hi_i + 0.5) / 2048 * 1.5
    if hi - lo >= 0.02:
        t = np.clip((luma - lo) / (hi - lo), 0, 1)
        s = 0.015 + t * 0.945
        sc = (luma + (s - luma) * AUTO_CONTRAST) / np.maximum(luma, 1e-4)
        for c in range(3):
            f[..., c] *= sc
    snap("contrast:out")

    for c in range(3):
        f[..., c] = lin_to_srgb(srgb_to_lin(np.clip(f[..., c], 0, 1)) * WB[c])
    snap("wb:out")

    luma = 0.2126 * f[..., 0] + 0.7152 * f[..., 1] + 0.0722 * f[..., 2]
    if MIDTONE_GAMMA != 1.0:
        v = np.clip(luma, 0, 1) ** MIDTONE_GAMMA
        sc = v / np.maximum(luma, 1e-4)
        for c in range(3):
            f[..., c] *= sc
    snap("gamma:out")

    Lc = np.clip(luma, 0, 1)
    shw = (1 - Lc) ** 2
    hiw = Lc ** 2
    for c in range(3):
        f[..., c] += SHADOW[c] * shw + HIGHLIGHT[c] * hiw
    snap("tint:out")

    cx, cy = (w - 1) / 2, (h - 1) / 2
    xx = (np.arange(w) - cx) / max(cx, 1)
    yy = (np.arange(h) - cy) / max(cy, 1)
    d = (yy[:, None] ** 2 + xx[None, :] ** 2) * 0.5
    vs = np.clip(1 - VIGNETTE * np.clip(d, 0, 1) ** 1.2, 0, 1)
    for c in range(3):
        f[..., c] *= vs
    snap("vignette:out")
    return f


def gauss_usm(plane_u8, w, h, amount, sigma):
    """Full-frame separable gaussian + unsharp with edge replication -- the
    firmware's row-ring sharpen computes exactly this (the ring holds the
    2*rad+1-row vertical kernel window; no band edges cut into the window)."""
    rad = int(np.ceil(3 * sigma))
    kern = np.exp(-0.5 * (np.arange(-rad, rad + 1) ** 2) / sigma ** 2)
    kern /= kern.sum()
    hb = np.zeros_like(plane_u8, dtype=np.float64)
    for d in range(-rad, rad + 1):
        idx = np.clip(np.arange(w) + d, 0, w - 1)
        hb += kern[d + rad] * plane_u8[:, idx].astype(np.float64)
    b = np.zeros_like(hb)
    for d in range(-rad, rad + 1):
        idx = np.clip(np.arange(h) + d, 0, h - 1)
        b += kern[d + rad] * hb[idx, :]
    g = plane_u8.astype(np.float64)
    return np.clip(np.round(g + amount * (g - b)), 0, 255)


def save_rgb(name, r8, g8, b8):
    img = Image.fromarray(np.stack([r8, g8, b8], axis=-1).astype(np.uint8), "RGB")
    if SCALE > 1:
        img = img.resize((img.width // SCALE, img.height // SCALE), Image.NEAREST)
    img.save(OUT / name)
    print("wrote", name)


def to_u8(f):
    return np.clip(np.round(f * 255), 0, 255).astype(np.uint8)


def chain(img, unpack, dumps=None, usm=True):
    """Decode canvas -> look -> USM -> firmware pack, then the pre-encode
    byte swap -> the encoder's view (what the file shows). Both builds leave
    the canvas in the encoder's byte order; the old one swapped inside
    unpack/pack, the new one in one pass right before encode."""
    w, h = img.width, img.height
    a = np.asarray(img, dtype=np.uint8)
    stored = pack565_std(a[..., 0], a[..., 1], a[..., 2])  # jpg2rgb565 canvas
    r8, g8, b8 = unpack(stored)
    if dumps is not None:
        dumps["unpack"] = np.stack([r8, g8, b8], axis=-1)
        f = apply_look(r8, g8, b8, w, h, dumps)
    else:
        f = apply_look(r8, g8, b8, w, h)
    u = to_u8(f)
    if usm:
        for c in range(3):
            u[..., c] = gauss_usm(u[..., c], w, h, USM_AMOUNT, USM_SIGMA)
    if dumps is not None and usm:
        dumps["usm"] = u.copy()
    stored2 = swap16(pack565_std(u[..., 0], u[..., 1], u[..., 2]))
    return tuple(x.astype(np.uint8) for x in encoder_read565(stored2)), u


def main():
    import argparse
    ap = argparse.ArgumentParser()
    default_img = Path(__file__).parents[2] / "test-images" / "img_0037.jpg"
    ap.add_argument("input", nargs="?", default=str(default_img))
    ap.add_argument("--stages", action="store_true",
                    help="dump per-stage images of the fixed story")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    _self_check()

    img = Image.open(args.input).convert("RGB")
    w, h = img.size
    stem = Path(args.input).stem

    a = np.asarray(img, dtype=np.uint8)
    # lab reference: full chain in 8-bit float, incl. USM
    f = apply_look(a[..., 0], a[..., 1], a[..., 2], w, h)
    u = to_u8(f)
    for c in range(3):
        u[..., c] = gauss_usm(u[..., c], w, h, USM_AMOUNT, USM_SIGMA)
    save_rgb(f"{stem}__lab_ref.png", u[..., 0], u[..., 1], u[..., 2])

    # the two firmware stories through the real encoder
    fixed, _ = chain(img, unpack_native)
    save_rgb(f"{stem}__fixed.png", *fixed)
    asbuilt, _ = chain(img, unpack_asbuilt)
    save_rgb(f"{stem}__asbuilt.png", *asbuilt)

    if args.stages:
        d = {}
        chain(img, unpack_native, d)
        save_rgb(f"{stem}__st0_unpack.png", d["unpack"][..., 0],
                 d["unpack"][..., 1], d["unpack"][..., 2])
        for i, (name, f, _) in enumerate(d.get("look", [])):
            save_rgb(f"{stem}__st{i + 1}_{name.replace(':', '_')}.png",
                     to_u8(f)[..., 0], to_u8(f)[..., 1], to_u8(f)[..., 2])
        if "usm" in d:
            save_rgb(f"{stem}__st_usm.png", d["usm"][..., 0],
                     d["usm"][..., 1], d["usm"][..., 2])


if __name__ == "__main__":
    main()
