"""Host-side simulation of the lofi-camera OLED preview pipeline.

Mirrors ``src/preview.h`` + ``src/preview_dither.h`` + ``src/preview_canny.h``
without touching the firmware:

    RGB -> Rec.709 luma -> aspect-fit letterbox (nearest neighbour)
    -> dither variant or Canny edge detection

``dither_render`` is the exact firmware 4x4 Bayer renderer. The extra
``*_render`` helpers below are lab-only alternatives for judging what a
different 1-bit mapping would look like on the same 128x64 panel.

The Canny path follows the firmware step for step: optional binomial blur
passes, Sobel ``|gx| + |gy|`` gradients with quantised direction, one-sided
non-maximum suppression, double threshold, and hysteresis.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64
PREVIEW_MAX_W = 128
PREVIEW_MAX_H = 64

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
    """Rec.709 luma, matching the firmware's integer weights (54/183/19)."""
    rgb = np.asarray(rgb, dtype=np.float64)
    luma = (54.0 * rgb[..., 0] + 183.0 * rgb[..., 1] + 19.0 * rgb[..., 2]) / 256.0
    return np.clip(np.round(luma), 0, 255).astype(np.uint8)


def preview_prepare(
    gray: np.ndarray,
    screen_w: int = SCREEN_WIDTH,
    screen_h: int = SCREEN_HEIGHT,
) -> PreviewFrame:
    """Aspect-fit + letterbox with nearest neighbour, like ``previewPrepare``."""
    gray = np.asarray(gray, dtype=np.uint8)
    if gray.ndim != 2:
        raise ValueError("preview_prepare expects 2D grayscale input")
    src_h, src_w = gray.shape
    if src_w <= 0 or src_h <= 0:
        empty = np.zeros((0, 0), dtype=np.uint8)
        return PreviewFrame(px=empty, w=0, h=0, x=0, y=0)

    scale = min(screen_w / src_w, screen_h / src_h)
    dw = int(src_w * scale)
    dh = int(src_h * scale)
    dw = min(max(dw, 1), PREVIEW_MAX_W)
    dh = min(max(dh, 1), PREVIEW_MAX_H)

    # Firmware mapping: dst[x] = row[x * srcW / dw] with integer division.
    ys = (np.arange(dh, dtype=np.int64) * src_h) // dh
    xs = (np.arange(dw, dtype=np.int64) * src_w) // dw
    px = gray[ys[:, None], xs[None, :]].astype(np.uint8, copy=True)
    return PreviewFrame(
        px=px,
        w=dw,
        h=dh,
        x=(screen_w - dw) // 2,
        y=(screen_h - dh) // 2,
    )


def _stretch_contrast(gray: np.ndarray, contrast: int) -> np.ndarray:
    """Firmware contrast stretch around mid-grey (100 = unchanged)."""
    stretched = 128 + (gray.astype(np.int32) - 128) * int(contrast) // 100
    return np.clip(stretched, 0, 255).astype(np.uint8)


def _blit(out: np.ndarray, frame: PreviewFrame, white: np.ndarray) -> np.ndarray:
    """Paste a boolean white mask into the letterboxed display buffer."""
    out[frame.y : frame.y + frame.h, frame.x : frame.x + frame.w] = np.where(
        white, 255, 0
    ).astype(np.uint8)
    return out


def dither_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """4x4 Bayer ordered dither with the firmware's contrast stretch."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast).astype(np.int32)
    yy, xx = np.mgrid[0 : frame.h, 0 : frame.w]
    threshold = BAYER_4X4[yy & 3, xx & 3] * 16 + 8
    return _blit(out, frame, stretched > threshold)


def dither_bayer2_render(frame: PreviewFrame, contrast: int = 130) -> np.ndarray:
    """Lab-only 2x2 Bayer ordered dither (tight dots, only 4 gray steps)."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast).astype(np.int32)
    yy, xx = np.mgrid[0 : frame.h, 0 : frame.w]
    threshold = BAYER_2X2[yy & 1, xx & 1] * 64 + 32
    return _blit(out, frame, stretched > threshold)


# 8x8 Bayer matrix (values 0-63). Spreads the quantisation error over a wider
# area than the firmware 4x4, so smooth gradients band less at the cost of a
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
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast).astype(np.int32)
    yy, xx = np.mgrid[0 : frame.h, 0 : frame.w]
    threshold = BAYER_8X8[yy & 7, xx & 7] * 4 + 2
    return _blit(out, frame, stretched > threshold)


def threshold_render(
    frame: PreviewFrame, contrast: int = 130, level: int = 128
) -> np.ndarray:
    """Lab-only plain threshold (no dither): stark black/white posterisation."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    return _blit(out, frame, stretched > int(level))


def _floyd_steinberg_gray(
    gray: np.ndarray, threshold: float = 128.0, strength: float = 1.0
) -> np.ndarray:
    """Serpentine Floyd-Steinberg error diffusion on a grayscale tile.

    ``threshold`` sets the black/white decision point; ``strength`` scales how
    much quantisation error is diffused (1.0 = classic Floyd-Steinberg, lower
    values calm worming at the cost of flatter mid-tones).
    """
    img = gray.astype(np.float64)
    strength = float(np.clip(strength, 0.0, 1.0))
    h, w = img.shape
    for y in range(h):
        left_to_right = (y & 1) == 0
        xs = range(w) if left_to_right else range(w - 1, -1, -1)
        for x in xs:
            old = img[y, x]
            new = 255.0 if old >= threshold else 0.0
            img[y, x] = new
            err = (old - new) * strength
            if left_to_right:
                if x + 1 < w:
                    img[y, x + 1] += err * 7 / 16
                if y + 1 < h:
                    if x > 0:
                        img[y + 1, x - 1] += err * 3 / 16
                    img[y + 1, x] += err * 5 / 16
                    if x + 1 < w:
                        img[y + 1, x + 1] += err * 1 / 16
            else:
                if x > 0:
                    img[y, x - 1] += err * 7 / 16
                if y + 1 < h:
                    if x + 1 < w:
                        img[y + 1, x + 1] += err * 3 / 16
                    img[y + 1, x] += err * 5 / 16
                    if x > 0:
                        img[y + 1, x - 1] += err * 1 / 16
    return (img >= threshold).astype(bool)


def area_resize(gray: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
    """Area-averaging downscale (box filter), edge-inclusive.

    Each destination pixel averages the source pixels whose centres fall in
    its box. Unlike nearest-neighbour this preserves photographic energy when
    downsampling (no aliasing shimmer); use nearest only when the source is
    already at panel size.
    """
    img = np.asarray(gray, dtype=np.float64)
    if img.ndim != 2:
        raise ValueError("area_resize expects 2D grayscale input")
    src_h, src_w = img.shape
    dst_w = max(1, int(dst_w))
    dst_h = max(1, int(dst_h))
    if (src_h, src_w) == (dst_h, dst_w):
        return np.clip(np.round(img), 0, 255).astype(np.uint8)

    row_edges = np.linspace(0, src_h, dst_h + 1)
    col_edges = np.linspace(0, src_w, dst_w + 1)
    out = np.zeros((dst_h, dst_w), dtype=np.float64)
    for dy in range(dst_h):
        y0, y1 = row_edges[dy], row_edges[dy + 1]
        iy0, iy1 = int(np.floor(y0)), int(np.ceil(y1))
        for dx in range(dst_w):
            x0, x1 = col_edges[dx], col_edges[dx + 1]
            ix0, ix1 = int(np.floor(x0)), int(np.ceil(x1))
            acc = 0.0
            area = 0.0
            for iy in range(max(iy0, 0), min(iy1, src_h)):
                oy0, oy1 = max(y0, iy), min(y1, iy + 1)
                wy = max(0.0, oy1 - oy0)
                if wy <= 0:
                    continue
                for ix in range(max(ix0, 0), min(ix1, src_w)):
                    ox0, ox1 = max(x0, ix), min(x1, ix + 1)
                    wx = max(0.0, ox1 - ox0)
                    if wx <= 0:
                        continue
                    acc += img[iy, ix] * wy * wx
                    area += wy * wx
            out[dy, dx] = acc / area if area > 0 else img[min(iy0, src_h - 1), min(ix0, src_w - 1)]
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


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
    """First-order IIR blend towards the previous stabilised frame.

    ``alpha`` weights the past (0 = no memory, 1 = frozen). ``previous=None``
    returns ``current`` unchanged (first frame). Both inputs are uint8 tiles.
    """
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
    sharpen_sigma: float = 1.0,
    blur_sigma: float = 0.0,
    threshold: float = 128.0,
    strength: float = 0.9,
    previous: np.ndarray | None = None,
    alpha: float = 0.8,
) -> tuple[np.ndarray, np.ndarray]:
    """Lab-only photo chain on the letterboxed tile.

    Grayscale -> tone adjust (contrast/brightness/gamma) -> optional unsharp
    mask -> optional Gaussian blur -> temporal stabilise -> Floyd-Steinberg
    with adjustable threshold and diffusion strength.

    Returns ``(display_buffer, stabilised_tile)``; feed the stabilised tile
    back as ``previous`` for the next frame to get temporal smoothing.
    """
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        empty = np.zeros((0, 0), dtype=np.uint8)
        return out, empty

    toned = tone_adjust(frame.px, contrast=contrast, brightness=brightness, gamma=gamma)
    sharpened = unsharp_mask(toned, amount=sharpen_amount, sigma=sharpen_sigma)
    softened = gaussian_blur(sharpened, sigma=blur_sigma)
    stabilised = temporal_stabilise(softened, previous, alpha=alpha)
    mask = _floyd_steinberg_gray(stabilised, threshold=threshold, strength=strength)
    return _blit(out, frame, mask), stabilised


def floyd_steinberg_render(
    frame: PreviewFrame, contrast: int = 130
) -> np.ndarray:
    """Lab-only Floyd-Steinberg error diffusion (smooth tones, some worming)."""
    out = np.zeros((SCREEN_HEIGHT, SCREEN_WIDTH), dtype=np.uint8)
    if frame.w <= 0 or frame.h <= 0:
        return out

    stretched = _stretch_contrast(frame.px, contrast)
    return _blit(out, frame, _floyd_steinberg_gray(stretched))


def _atkinson_gray(gray: np.ndarray) -> np.ndarray:
    """Serpentine Atkinson error diffusion (1/8 spread, higher contrast)."""
    img = gray.astype(np.float64)
    h, w = img.shape
    for y in range(h):
        left_to_right = (y & 1) == 0
        xs = range(w) if left_to_right else range(w - 1, -1, -1)
        step = 1 if left_to_right else -1
        for x in xs:
            old = img[y, x]
            new = 255.0 if old >= 128.0 else 0.0
            img[y, x] = new
            err = (old - new) / 8.0
            if left_to_right:
                neighbours = [(y, x + 1), (y, x + 2), (y + 1, x - 1),
                              (y + 1, x), (y + 1, x + 1), (y + 2, x)]
            else:
                neighbours = [(y, x - 1), (y, x - 2), (y + 1, x + 1),
                              (y + 1, x), (y + 1, x - 1), (y + 2, x)]
            _ = step
            for ny, nx in neighbours:
                if 0 <= ny < h and 0 <= nx < w:
                    img[ny, nx] += err
    return (img >= 128).astype(bool)


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
    """Lab-only slight Gaussian blur (separable, edge-replicated).

    ``sigma`` is in pixels; radius is ``ceil(3 * sigma)``. ``sigma <= 0``
    returns the input unchanged.
    """
    img = np.asarray(gray, dtype=np.float64)
    sigma = float(sigma)
    if sigma <= 0:
        return np.clip(np.round(img), 0, 255).astype(np.uint8)
    radius = max(1, int(np.ceil(3 * sigma)))
    xs = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (xs / sigma) ** 2)
    kernel = kernel / kernel.sum()

    padded = np.pad(img, ((0, 0), (radius, radius)), mode="edge")
    tmp = np.zeros_like(img)
    for i in range(2 * radius + 1):
        tmp += kernel[i] * padded[:, i : i + img.shape[1]]
    padded = np.pad(tmp, ((radius, radius), (0, 0)), mode="edge")
    out = np.zeros_like(img)
    for i in range(2 * radius + 1):
        out += kernel[i] * padded[i : i + img.shape[0], :]
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


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
    """Full firmware Canny chain rendered onto a 128x64 1-bit image."""
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
