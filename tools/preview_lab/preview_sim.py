"""Host-side simulation of the lofi-camera OLED preview pipeline.

Mirrors the firmware Floyd-Steinberg path and provides lab-only alternatives,
including Bayer and Canny, without touching firmware:

    RGB -> Rec.709 luma -> aspect-fit letterbox (nearest neighbour)
    -> Floyd-Steinberg, another dither variant, or Canny edge detection

``floyd_steinberg_render`` uses the firmware's error-diffusion defaults. The
other ``*_render`` helpers, including Bayer and Canny, are lab-only alternatives
for judging different 1-bit mappings on the same 128x64 panel. Canny uses
optional binomial blur passes, Sobel ``|gx| + |gy|`` gradients with quantised
direction, one-sided non-maximum suppression, double threshold, and hysteresis.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageFilter

SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64

BAYER_2X2 = np.array(
    [
        [0, 2],
        [3, 1],
    ],
    dtype=np.int32,
)

BAYER_4X4 = np.array(
    [
        [0, 8, 2, 10],
        [12, 4, 14, 6],
        [3, 11, 1, 9],
        [15, 7, 13, 5],
    ],
    dtype=np.int32,
)


@dataclass
class PreviewFrame:
    """Aspect-fit letterboxed grayscale frame, like the firmware struct."""

    px: np.ndarray  # 2D uint8 grayscale, shape (h, w)
    w: int
    h: int
    x: int  # left offset on the 128x64 display
    y: int  # top offset on the display


def rgb_to_luma(rgb: np.ndarray) -> np.ndarray:
    """Rec.709 luma after the firmware's RGB565 quantise/expand step."""
    rgb = np.asarray(rgb, dtype=np.uint32)
    r5, g6, b5 = rgb[..., 0] >> 3, rgb[..., 1] >> 2, rgb[..., 2] >> 3
    r = (r5 << 3) | (r5 >> 2)
    g = (g6 << 2) | (g6 >> 4)
    b = (b5 << 3) | (b5 >> 2)
    return ((54 * r + 183 * g + 19 * b) >> 8).astype(np.uint8)


def preview_prepare(gray: np.ndarray) -> PreviewFrame:
    """Aspect-fit + letterbox with nearest neighbour, like ``previewPrepare``."""
    gray = np.asarray(gray, dtype=np.uint8)
    if gray.ndim != 2:
        raise ValueError("preview_prepare expects 2D grayscale input")
    src_h, src_w = gray.shape
    if src_w <= 0 or src_h <= 0:
        empty = np.zeros((0, 0), dtype=np.uint8)
        return PreviewFrame(px=empty, w=0, h=0, x=0, y=0)

    scale = min(SCREEN_WIDTH / src_w, SCREEN_HEIGHT / src_h)
    dw = int(src_w * scale)
    dh = int(src_h * scale)
    dw = min(max(dw, 1), SCREEN_WIDTH)
    dh = min(max(dh, 1), SCREEN_HEIGHT)

    # Firmware mapping: dst[x] = row[x * srcW / dw] with integer division.
    ys = (np.arange(dh, dtype=np.int64) * src_h) // dh
    xs = (np.arange(dw, dtype=np.int64) * src_w) // dw
    px = gray[ys[:, None], xs[None, :]].astype(np.uint8, copy=True)
    return PreviewFrame(
        px=px,
        w=dw,
        h=dh,
        x=(SCREEN_WIDTH - dw) // 2,
        y=(SCREEN_HEIGHT - dh) // 2,
    )


def _stretch_contrast(gray: np.ndarray, contrast: int) -> np.ndarray:
    """Firmware contrast stretch around mid-grey (100 = unchanged)."""
    product = (gray.astype(np.int64) - 128) * int(contrast)
    # C++ signed integer division truncates toward zero; Python // floors.
    quotient = np.where(product >= 0, product // 100, -((-product) // 100))
    stretched = 128 + quotient
    return np.clip(stretched, 0, 255).astype(np.uint8)


def _blit(out: np.ndarray, frame: PreviewFrame, white: np.ndarray) -> np.ndarray:
    """Paste a boolean white mask into the letterboxed display buffer."""
    out[frame.y : frame.y + frame.h, frame.x : frame.x + frame.w] = np.where(
        white, 255, 0
    ).astype(np.uint8)
    return out


def _bayer_render(frame: PreviewFrame, matrix: np.ndarray, contrast: int = 130) -> np.ndarray:
    """Ordered dither with any square Bayer matrix (levels = n*n)."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out
    n = matrix.shape[0]
    step = 256 // (n * n)
    stretched = _stretch_contrast(frame.px, contrast).astype(np.int32)
    yy, xx = np.mgrid[0 : frame.h, 0 : frame.w]
    threshold = matrix[yy % n, xx % n] * step + step // 2
    return _blit(out, frame, stretched > threshold)


def dither_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only 4x4 Bayer ordered dither with a contrast stretch."""
    return _bayer_render(frame, BAYER_4X4, contrast)


def dither_bayer2_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only 2x2 Bayer ordered dither (tight dots, only 4 gray steps)."""
    return _bayer_render(frame, BAYER_2X2, contrast)


# 8x8 Bayer matrix (values 0-63). Spreads quantisation over a wider area
# than the 4x4 alternative, so smooth gradients band less at the cost of a
# coarser visible dot pattern.
BAYER_8X8 = np.array(
    [
        [0, 32, 8, 40, 2, 34, 10, 42],
        [48, 16, 56, 24, 50, 18, 58, 26],
        [12, 44, 4, 36, 14, 46, 6, 38],
        [60, 28, 52, 20, 62, 30, 54, 22],
        [3, 35, 11, 43, 1, 33, 9, 41],
        [51, 19, 59, 27, 49, 17, 57, 25],
        [15, 47, 7, 39, 13, 45, 5, 37],
        [63, 31, 55, 23, 61, 29, 53, 21],
    ],
    dtype=np.int32,
)


def dither_bayer8_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only 8x8 Bayer ordered dither (finer tones, coarser dots)."""
    return _bayer_render(frame, BAYER_8X8, contrast)


def threshold_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only plain threshold (no dither): stark black/white posterisation."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    return _blit(out, frame, stretched > 128)


def _error_diffuse(
    gray: np.ndarray, threshold: float, taps: list[tuple[int, int, float]]
) -> np.ndarray:
    """Serpentine error diffusion with mirrored taps on odd rows."""
    img = gray.astype(np.float64)
    h, w = img.shape
    for y in range(h):
        ltr = (y & 1) == 0
        xs = range(w) if ltr else range(w - 1, -1, -1)
        for x in xs:
            old = img[y, x]
            new = 255.0 if old >= threshold else 0.0
            img[y, x] = new
            err = old - new
            for dx, dy, f in taps:
                if not ltr:
                    dx = -dx
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    img[ny, nx] += err * f
    return (img >= threshold).astype(bool)


def _floyd_steinberg_gray(
    gray: np.ndarray, threshold: float = 128.0, strength: float = 1.0
) -> np.ndarray:
    """Serpentine Floyd-Steinberg error diffusion on a grayscale tile.

    ``threshold`` sets the black/white decision point; ``strength`` scales how
    much quantisation error is diffused (1.0 = classic Floyd-Steinberg, lower
    values calm worming at the cost of flatter mid-tones).
    """
    strength = float(np.clip(strength, 0.0, 1.0))
    taps = [(1, 0, 7 / 16 * strength), (-1, 1, 3 / 16 * strength),
            (0, 1, 5 / 16 * strength), (1, 1, 1 / 16 * strength)]
    return _error_diffuse(gray, threshold, taps)


def area_resize(gray: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
    """Area-averaging downscale via Pillow's box filter."""
    img = np.asarray(gray, dtype=np.uint8)
    if img.ndim != 2:
        raise ValueError("area_resize expects 2D grayscale input")
    dst_w = max(1, int(dst_w))
    dst_h = max(1, int(dst_h))
    if img.shape == (dst_h, dst_w):
        return img.copy()
    # ponytail: Pillow BOX, custom loop if Pillow falls short on edge handling
    return np.asarray(
        Image.fromarray(img, mode="L").resize((dst_w, dst_h), Image.BOX),
        dtype=np.uint8,
    )


def tone_adjust(
    gray: np.ndarray,
    contrast: float = 1.15,
    brightness: float = 0.0,
    gamma: float = 1.0,
) -> np.ndarray:
    """Photo-style tone controls: gain, offset, then gamma.

    ``contrast`` multiplies around mid-grey (1.0 = unchanged),
    ``brightness`` adds a flat offset in gray levels, and ``gamma`` applies a
    power curve (1.0 = off, >1 darkens mid-tones, <1 lifts them).
    """
    img = np.asarray(gray, dtype=np.float64)
    out = (img - 128.0) * float(contrast) + 128.0 + float(brightness) * 255.0
    out = np.clip(out, 0.0, 255.0) / 255.0
    gamma = float(gamma)
    if gamma != 1.0 and gamma > 0:
        out = np.power(np.clip(out, 0.0, 1.0), gamma)
    return np.clip(np.round(out * 255.0), 0, 255).astype(np.uint8)


def temporal_stabilise(
    current: np.ndarray, previous: np.ndarray | None, alpha: float = 0.8
) -> np.ndarray:
    """Blend a frame with the previous stabilised tile (alpha weights the past)."""
    cur = np.asarray(current, dtype=np.float64)
    if previous is None:
        return np.clip(np.round(cur), 0, 255).astype(np.uint8)
    prev = np.asarray(previous, dtype=np.float64)
    if prev.shape != cur.shape:
        return np.clip(np.round(cur), 0, 255).astype(np.uint8)
    alpha = float(np.clip(alpha, 0.0, 1.0))
    out = alpha * prev + (1.0 - alpha) * cur
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def unsharp_mask(
    gray: np.ndarray, amount: float = 0.0, sigma: float = 1.0
) -> np.ndarray:
    """Optional sharpening: original + amount * (original - gaussian blur)."""
    img = np.asarray(gray, dtype=np.float64)
    amount = float(amount)
    if amount == 0:
        return np.clip(np.round(img), 0, 255).astype(np.uint8)
    blurred = gaussian_blur(img, sigma=sigma).astype(np.float64)
    out = img + amount * (img - blurred)
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def photo_floyd_render(
    frame: PreviewFrame,
    contrast: float = 1.15,
    brightness: float = 0.0,
    gamma: float = 1.0,
    sharpen_amount: float = 0.0,
    blur_sigma: float = 0.0,
    threshold: float = 128.0,
    strength: float = 0.9,
    previous: np.ndarray | None = None,
    alpha: float = 0.8,
) -> tuple[np.ndarray, np.ndarray]:
    """Lab-only photo chain on the letterboxed tile.

    Grayscale -> tone adjust -> optional unsharp mask -> optional blur
    -> temporal stabilisation -> Floyd-Steinberg.

    Returns the display buffer and stabilised grayscale tile. Feed the tile
    back as ``previous`` when processing successive frames.
    """
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out, np.zeros((0, 0), dtype=np.uint8)

    toned = tone_adjust(frame.px, contrast=contrast, brightness=brightness, gamma=gamma)
    sharpened = unsharp_mask(toned, amount=sharpen_amount, sigma=1.0)
    softened = gaussian_blur(sharpened, sigma=blur_sigma)
    stabilised = temporal_stabilise(softened, previous, alpha=alpha)
    mask = _floyd_steinberg_gray(stabilised, threshold=threshold, strength=strength)
    return _blit(out, frame, mask), stabilised


def floyd_steinberg_render(
    frame: PreviewFrame, contrast: int = 130, threshold: float = 128.0,
    strength: float = 0.45,
) -> np.ndarray:
    """Floyd-Steinberg error diffusion matching the firmware defaults."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    return _blit(out, frame, _floyd_steinberg_gray(
        stretched, threshold=threshold, strength=strength))


def _atkinson_gray(gray: np.ndarray) -> np.ndarray:
    """Serpentine Atkinson error diffusion (1/8 spread, higher contrast)."""
    taps = [(1, 0, 1 / 8), (2, 0, 1 / 8), (-1, 1, 1 / 8),
            (0, 1, 1 / 8), (1, 1, 1 / 8), (0, 2, 1 / 8)]
    return _error_diffuse(gray, 128.0, taps)


def atkinson_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only Atkinson error diffusion (punchy, posterised highlights)."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    return _blit(out, frame, _atkinson_gray(stretched))


def clahe(
    gray: np.ndarray, clip_limit: float = 2.0, tiles: tuple[int, int] = (8, 8)
) -> np.ndarray:
    """Contrast Limited Adaptive Histogram Equalisation on grayscale.

    Lab-only tone-mapping step: each tile gets its own clipped histogram, then
    tiles blend with bilinear interpolation so boundaries stay invisible.
    ``clip_limit`` caps how far any histogram bin can spike (higher = more
    local punch); ``tiles`` sets the grid as (rows, cols).
    """
    img = np.asarray(gray, dtype=np.uint8)
    if img.ndim != 2:
        raise ValueError("clahe expects 2D grayscale input")
    h, w = img.shape
    rows, cols = int(tiles[0]), int(tiles[1])
    rows = min(max(rows, 1), h)
    cols = min(max(cols, 1), w)
    clip = max(float(clip_limit), 1.0)

    # Tile boundaries: evenly split, last tile absorbs any remainder.
    row_edges = np.linspace(0, h, rows + 1).astype(int)
    col_edges = np.linspace(0, w, cols + 1).astype(int)

    # Per-tile clipped CDFs mapping 0..255 -> 0..255.
    maps = np.zeros((rows, cols, 256), dtype=np.float64)
    for ty in range(rows):
        for tx in range(cols):
            tile = img[row_edges[ty] : row_edges[ty + 1], col_edges[tx] : col_edges[tx + 1]]
            hist = np.bincount(tile.ravel(), minlength=256).astype(np.float64)
            excess = np.maximum(hist - clip * tile.size / 256.0, 0.0).sum()
            hist = np.minimum(hist, clip * tile.size / 256.0)
            hist += excess / 256.0
            cdf = hist.cumsum()
            total = cdf[-1]
            if total > 0:
                cdf = cdf / total
            maps[ty, tx] = cdf * 255.0

    # Bilinear blend between the four surrounding tile maps per pixel.
    row_centres = (row_edges[:-1] + row_edges[1:]) / 2.0
    col_centres = (col_edges[:-1] + col_edges[1:]) / 2.0
    ys = np.arange(h)
    xs = np.arange(w)
    ty0 = np.clip(np.searchsorted(row_centres, ys, side="right") - 1, 0, rows - 1)
    ty1 = np.clip(ty0 + 1, 0, rows - 1)
    tx0 = np.clip(np.searchsorted(col_centres, xs, side="right") - 1, 0, cols - 1)
    tx1 = np.clip(tx0 + 1, 0, cols - 1)

    wy1 = np.zeros(h)
    wx1 = np.zeros(w)
    same_row = ty0 == ty1
    same_col = tx0 == tx1
    denom_y = row_centres[np.clip(ty1, 0, rows - 1)] - row_centres[ty0]
    denom_x = col_centres[np.clip(tx1, 0, cols - 1)] - col_centres[tx0]
    wy1[~same_row] = (ys[~same_row] - row_centres[ty0[~same_row]]) / denom_y[~same_row]
    wx1[~same_col] = (xs[~same_col] - col_centres[tx0[~same_col]]) / denom_x[~same_col]
    wy1 = np.clip(wy1, 0.0, 1.0)
    wx1 = np.clip(wx1, 0.0, 1.0)
    wy0 = 1.0 - wy1
    wx0 = 1.0 - wx1

    out = np.zeros((h, w), dtype=np.float64)
    for dy, wy in ((0, wy0[:, None]), (1, wy1[:, None])):
        tyy = np.where(dy == 0, ty0[:, None], ty1[:, None])
        tyy = np.broadcast_to(tyy, (h, w))
        for dx, wx in ((0, wx0[None, :]), (1, wx1[None, :])):
            txx = np.where(dx == 0, tx0[None, :], tx1[None, :])
            txx = np.broadcast_to(txx, (h, w))
            out += wx * wy * maps[tyy, txx, img]
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def gaussian_blur(gray: np.ndarray, sigma: float = 0.8) -> np.ndarray:
    """Lab-only slight Gaussian blur. ``sigma <= 0`` returns input unchanged."""
    img = np.asarray(gray, dtype=np.uint8)
    sigma = float(sigma)
    if sigma <= 0:
        return img.copy()
    # ponytail: Pillow radius≈sigma, custom separable loop if edge handling matters
    return np.asarray(
        Image.fromarray(img, mode="L").filter(ImageFilter.GaussianBlur(radius=sigma)),
        dtype=np.uint8,
    )


def clahe_floyd_render(
    frame: PreviewFrame,
    contrast: int = 100,
    clip_limit: float = 2.0,
    tiles: tuple[int, int] = (8, 8),
    sigma: float = 0.8,
) -> np.ndarray:
    """Lab-only chain: CLAHE -> slight Gaussian blur -> Floyd-Steinberg.

    Runs on the letterboxed grayscale tile (after luma), so the panel mapping
    stays identical to the other modes. ``contrast`` applies the firmware
    stretch first; ``100`` leaves the CLAHE input untouched.
    """
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    equalised = clahe(stretched, clip_limit=clip_limit, tiles=tiles)
    softened = gaussian_blur(equalised, sigma=sigma)
    return _blit(out, frame, _floyd_steinberg_gray(softened))


def blur_atkinson_render(
    frame: PreviewFrame,
    contrast: int = 130,
    sigma: float = 0.8,
) -> np.ndarray:
    """Lab-only chain: grayscale -> slight Gaussian blur -> Atkinson.

    Runs on the letterboxed grayscale tile (after luma), so the panel mapping
    stays identical to the other modes. ``contrast`` applies the firmware
    stretch first; ``sigma <= 0`` skips the blur.
    """
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    softened = gaussian_blur(stretched, sigma=sigma)
    return _blit(out, frame, _atkinson_gray(softened))


def _edge_clamped(arr: np.ndarray) -> np.ndarray:
    """Pad one pixel by edge replication, like the firmware's ``at()``."""
    return np.pad(arr, 1, mode="edge")


def binomial_blur(gray: np.ndarray, passes: int) -> np.ndarray:
    """Repeated 3x3 binomial blur with integer arithmetic (``acc // 16``)."""
    img = np.asarray(gray, dtype=np.int32)
    for _ in range(max(0, int(passes))):
        padded = _edge_clamped(img)
        acc = (
            padded[:-2, :-2]
            + 2 * padded[:-2, 1:-1]
            + padded[:-2, 2:]
            + 2 * padded[1:-1, :-2]
            + 4 * padded[1:-1, 1:-1]
            + 2 * padded[1:-1, 2:]
            + padded[2:, :-2]
            + 2 * padded[2:, 1:-1]
            + padded[2:, 2:]
        )
        img = acc // 16
    return img.astype(np.uint8)


def sobel_magnitude_direction(gray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sobel ``|gx| + |gy|`` magnitudes and firmware direction codes."""
    img = np.asarray(gray, dtype=np.int32)
    padded = _edge_clamped(img)
    tl = padded[:-2, :-2]
    tc = padded[:-2, 1:-1]
    tr = padded[:-2, 2:]
    ml = padded[1:-1, :-2]
    mr = padded[1:-1, 2:]
    bl = padded[2:, :-2]
    bc = padded[2:, 1:-1]
    br = padded[2:, 2:]

    gx = -tl + tr - 2 * ml + 2 * mr - bl + br
    gy = -tl - 2 * tc - tr + bl + 2 * bc + br
    mag = np.abs(gx) + np.abs(gy)

    ax = np.abs(gx)
    ay = np.abs(gy)
    direction = np.full(mag.shape, 3, dtype=np.uint8)
    horizontal = ay * 5 < ax
    vertical = (~horizontal) & (ax * 5 < ay)
    same_sign = (~horizontal) & (~vertical) & ((gx ^ gy) >= 0)
    direction[horizontal] = 0
    direction[vertical] = 2
    direction[same_sign] = 1
    return mag.astype(np.int32), direction


def non_maximum_suppression(
    mag: np.ndarray, direction: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """One-sided NMS matching the firmware plateau behaviour."""
    h, w = mag.shape
    padded = np.pad(mag, 1, mode="constant", constant_values=0)

    # Neighbour magnitudes along each quantised gradient direction.
    left = mag[:, [0, *range(w - 1)]]
    right = mag[:, [*range(1, w), w - 1]]
    up = mag[[0, *range(h - 1)], :]
    down = mag[[*range(1, h), h - 1], :]
    diag_nw = padded[:-2, :-2]
    diag_se = padded[2:, 2:]
    diag_ne = padded[:-2, 2:]
    diag_sw = padded[2:, :-2]

    a = np.zeros_like(mag)
    b = np.zeros_like(mag)
    horizontal = direction == 0
    diag_45 = direction == 1
    vertical = direction == 2
    diag_135 = direction == 3
    a[horizontal] = left[horizontal]
    b[horizontal] = right[horizontal]
    a[diag_45] = diag_nw[diag_45]
    b[diag_45] = diag_se[diag_45]
    a[vertical] = up[vertical]
    b[vertical] = down[vertical]
    a[diag_135] = diag_ne[diag_135]
    b[diag_135] = diag_sw[diag_135]

    # Firmware keeps a pixel only when ``m > a`` on one side and ``m >= b`` on
    # the other, which thins flat-topped ridges consistently to one side.
    ridge = (mag > a) & (mag >= b)
    return mag, ridge


def double_threshold_hysteresis(
    mag: np.ndarray, ridge: np.ndarray, low: int, high: int
) -> np.ndarray:
    """Double threshold plus 8-connected hysteresis from strong pixels."""
    strong = ridge & (mag >= high)
    weak = ridge & (mag >= low) & (mag < high)
    edges = np.zeros_like(mag, dtype=bool)
    if not np.any(strong):
        return edges

    # Flood outward from every strong pixel through adjacent weak pixels.
    stack = [tuple(p) for p in np.argwhere(strong)]
    strong_set = {tuple(p) for p in stack}
    weak_set = {tuple(p) for p in np.argwhere(weak)}
    h, w = mag.shape
    while stack:
        y, x = stack.pop()
        if (y, x) in strong_set:
            edges[y, x] = True
        for ny in range(max(0, y - 1), min(h, y + 2)):
            for nx in range(max(0, x - 1), min(w, x + 2)):
                if (ny, nx) in weak_set:
                    weak_set.remove((ny, nx))
                    strong_set.add((ny, nx))
                    edges[ny, nx] = True
                    stack.append((ny, nx))
    return edges


def canny_render(
    frame: PreviewFrame,
    blur_passes: int = 2,
    low: int = 80,
    high: int = 160,
) -> np.ndarray:
    """Lab-only Canny chain rendered onto a 128x64 1-bit image."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    src = binomial_blur(frame.px, blur_passes)
    mag, direction = sobel_magnitude_direction(src)
    mag, ridge = non_maximum_suppression(mag, direction)
    edges = double_threshold_hysteresis(mag, ridge, low, high)
    out[frame.y : frame.y + frame.h, frame.x : frame.x + frame.w] = np.where(
        edges, 255, 0
    ).astype(np.uint8)
    return out
