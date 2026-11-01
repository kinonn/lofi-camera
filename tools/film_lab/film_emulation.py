"""
Film emulation filters for lofi-cam stills.

Host-side only: this module never touches the ESP32 firmware. It exists so the
look of the saved JPEGs can be evaluated on real camera output before any of it
is ported to C++ for the device.

Every filter is a small, explicit preset for one shared pipeline:

    auto-balance -> auto-contrast -> exposure/white balance (linear light)
      -> colour matrix -> saturation -> tone curve (gamma/contrast/rolloff/fade)
      -> shadow + highlight tint -> [B&W] -> halation -> softness
      -> vignette -> film grain

Colour is carried in gamma-encoded sRGB (the way film looks are normally
tuned); exposure and white balance happen in linear light where they belong.

Usage:
    from film_emulation import FILTERS, apply_filter
    out = apply_filter(pil_image, FILTERS["kodak_portra_400"])
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field, replace

import numpy as np
from PIL import Image

# Rec.709 weights, applied to gamma-encoded values (matches image editors).
LUMA = np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)


# --------------------------------------------------------------------------
# colour helpers
# --------------------------------------------------------------------------


def _to_float(img: Image.Image) -> np.ndarray:
    if img.mode != "RGB":
        img = img.convert("RGB")
    return np.asarray(img, dtype=np.float32) / 255.0


def _to_pil(arr: np.ndarray) -> Image.Image:
    return Image.fromarray(np.clip(arr * 255.0 + 0.5, 0, 255).astype(np.uint8))


def _luma(rgb: np.ndarray) -> np.ndarray:
    return rgb @ LUMA


def _srgb_to_linear(x: np.ndarray) -> np.ndarray:
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def _linear_to_srgb(x: np.ndarray) -> np.ndarray:
    x = np.maximum(x, 0.0)
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * np.power(x, 1.0 / 2.4) - 0.055)


# --------------------------------------------------------------------------
# fast separable gaussian (pure numpy box-blur triple pass, no scipy)
# --------------------------------------------------------------------------


def _box_blur(a: np.ndarray, radius: int, axis: int) -> np.ndarray:
    if radius < 1:
        return a
    n = a.shape[axis]
    widths = [(0, 0)] * a.ndim
    widths[axis] = (radius + 1, radius)
    ap = np.pad(a, np.asarray(widths, dtype=np.intp), mode="edge")
    cs = np.cumsum(ap, axis=axis, dtype=np.float64)
    hi = [slice(None)] * a.ndim
    lo = [slice(None)] * a.ndim
    hi[axis] = slice(2 * radius + 1, 2 * radius + 1 + n)
    lo[axis] = slice(0, n)
    return ((cs[tuple(hi)] - cs[tuple(lo)]) / (2 * radius + 1)).astype(np.float32)


def _gaussian(a: np.ndarray, sigma: float) -> np.ndarray:
    """Approximate a gaussian blur with three box passes."""
    if sigma < 0.3:
        return a
    radius = max(1, round((np.sqrt(4.0 * sigma * sigma + 1.0) - 1.0) / 2.0))
    out = a
    for _ in range(3):
        out = _box_blur(out, radius, 0)
        out = _box_blur(out, radius, 1)
    return out


# --------------------------------------------------------------------------
# pipeline stages
# --------------------------------------------------------------------------


def auto_balance(rgb: np.ndarray, strength: float) -> np.ndarray:
    """Gently neutralise a colour cast measured over mid-tones only.

    The ESP32-CAM JPEGs come out with a green/yellow cast. Correcting that
    *before* the film grade means each look still applies its own colour idea
    on top of a sane starting point.
    """
    if strength <= 0.0:
        return rgb
    luma = _luma(rgb)
    mask = (luma > 0.10) & (luma < 0.90)
    if mask.sum() < 200:
        return rgb
    means = rgb[mask].mean(axis=0)
    target = float(means.mean())
    gains = target / np.maximum(means, 1e-4)
    gains = 1.0 + (gains - 1.0) * strength
    return rgb * gains


def auto_contrast(rgb: np.ndarray, strength: float) -> np.ndarray:
    """Stretch the luma histogram between the 1st and 99th percentiles."""
    if strength <= 0.0:
        return rgb
    luma = _luma(rgb)
    flat = np.asarray(luma, dtype=np.float64).ravel()
    lo = float(np.percentile(flat, 1.0))
    hi = float(np.percentile(flat, 99.0))
    if hi - lo < 0.02:
        return rgb
    # Land the histogram on 0.015..0.96 instead of 0..1. Leaving a little
    # headroom stops the already near-clipped sky from going pure white, which
    # is what made every look read as the same blown-out highlight.
    target_lo, target_hi = 0.015, 0.960
    stretched = np.clip((luma - lo) / (hi - lo), 0.0, 1.0)
    stretched = target_lo + stretched * (target_hi - target_lo)
    new_luma = luma + (stretched - luma) * strength
    scale = new_luma / np.maximum(luma, 1e-4)
    return rgb * scale[..., None]


def exposure_wb(rgb: np.ndarray, exposure: float, wb: tuple) -> np.ndarray:
    if exposure == 0.0 and tuple(wb) == (1.0, 1.0, 1.0):
        return rgb
    lin = _srgb_to_linear(rgb)
    lin = lin * (2.0 ** exposure) * np.asarray(wb, dtype=np.float32)
    return _linear_to_srgb(lin)


def apply_matrix(rgb: np.ndarray, matrix: tuple) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float32)
    return rgb @ m.T


def saturate(rgb: np.ndarray, amount: float) -> np.ndarray:
    if amount == 1.0:
        return rgb
    luma = _luma(rgb)
    return luma[..., None] + (rgb - luma[..., None]) * amount


def _smoothstep_contrast(x: np.ndarray, amount: float) -> np.ndarray:
    if amount == 0.0:
        return x
    s = x * x * (3.0 - 2.0 * x)
    return x + amount * (s - x)


def _highlight_rolloff(x: np.ndarray, strength: float) -> np.ndarray:
    """Compress the top end the way film's shoulder does, never hard-clipping."""
    if strength <= 0.0:
        return x
    knee = 1.0 - strength
    span = max(1.0 - knee, 1e-3)
    over = np.maximum(x - knee, 0.0)
    rolled = knee + span * np.tanh(over / span)
    return np.where(x <= knee, x, rolled)


def tone(rgb: np.ndarray, preset: FilmPreset) -> np.ndarray:
    luma = _luma(rgb)
    new = luma

    if preset.midtone_gamma != 1.0:
        new = np.power(np.clip(new, 0.0, 1.0), preset.midtone_gamma)

    new = _smoothstep_contrast(new, preset.contrast)
    if preset.contrast_gain != 1.0:
        # Slope about mid-grey. Values pushed past 0 or 1 clip, which is what a
        # genuine contrast increase does to shadow and highlight detail.
        new = 0.5 + (new - 0.5) * preset.contrast_gain
    new = np.clip(new, 0.0, 1.0)
    new = _highlight_rolloff(new, preset.highlight_rolloff)

    if preset.fade > 0.0:
        new = preset.fade + new * (1.0 - preset.fade)

    scale = new / np.maximum(luma, 1e-4)
    return rgb * scale[..., None]


def tints(rgb: np.ndarray, preset: FilmPreset) -> np.ndarray:
    shadow = np.asarray(preset.shadow_tint, dtype=np.float32)
    mid = np.asarray(preset.midtone_tint, dtype=np.float32)
    high = np.asarray(preset.highlight_tint, dtype=np.float32)
    if not shadow.any() and not mid.any() and not high.any():
        return rgb
    luma = np.clip(_luma(rgb), 0.0, 1.0)
    rgb = rgb + shadow * ((1.0 - luma) ** 2)[..., None] + high * (luma ** 2)[..., None]
    if mid.any():
        # Weight peaks at mid-grey and falls to zero at black and white.
        rgb = rgb + mid * (4.0 * luma * (1.0 - luma))[..., None]
    return rgb


def monochrome(rgb: np.ndarray, preset: FilmPreset) -> np.ndarray:
    mix = np.asarray(preset.mono_mix, dtype=np.float32)
    grey = (rgb @ mix)[..., None]
    grey = np.repeat(grey, 3, axis=2)
    tint = np.asarray(preset.mono_tint, dtype=np.float32)
    if tint.any():
        grey = grey + tint
    return grey


def halation(rgb: np.ndarray, preset: FilmPreset, rng: np.random.Generator) -> np.ndarray:
    """Red-orange bloom bleeding off the highlights (the Cinestill signature)."""
    amount = preset.halation
    if amount <= 0.0:
        return rgb
    luma = np.clip(_luma(rgb), 0.0, 1.0)
    thresh = preset.halation_threshold
    over = np.clip((luma - thresh) / max(1.0 - thresh, 1e-3), 0.0, 1.0)
    # Deliberately not peak-normalised: a large bright region (an overcast sky)
    # must not be forced to full bloom just because it is the brightest thing
    # in frame. Real halation follows actual specular highlights only.
    bloom = _gaussian(over, preset.halation_radius)
    colour = np.asarray(preset.halation_colour, dtype=np.float32)
    return rgb + (bloom[..., None] * colour) * amount


def soften(rgb: np.ndarray, amount: float) -> np.ndarray:
    if amount <= 0.0:
        return rgb
    blurred = _gaussian(rgb, 1.6)
    return rgb + (blurred - rgb) * amount


def sharpen(rgb: np.ndarray, amount: float, radius: float) -> np.ndarray:
    """Unsharp mask: add back a scaled copy of the high-pass residual."""
    if amount <= 0.0:
        return rgb
    blurred = _gaussian(rgb, radius)
    return rgb + (rgb - blurred) * amount


def _radius(rgb: np.ndarray) -> np.ndarray:
    """Normalised radius map: 0 at the frame centre, 1.0 at the corners."""
    h, w = rgb.shape[:2]
    yy = (np.arange(h, dtype=np.float32) - (h - 1) / 2.0) / max((h - 1) / 2.0, 1.0)
    xx = (np.arange(w, dtype=np.float32) - (w - 1) / 2.0) / max((w - 1) / 2.0, 1.0)
    return np.sqrt((yy[:, None] ** 2 + xx[None, :] ** 2) / 2.0)


def _sample(channel: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Bilinear resample of a single channel at fractional coordinates."""
    h, w = channel.shape
    x0 = np.floor(xs).astype(np.int32)
    y0 = np.floor(ys).astype(np.int32)
    fx = (xs - x0).astype(np.float32)
    fy = (ys - y0).astype(np.float32)
    x0c = np.clip(x0, 0, w - 1)
    x1c = np.clip(x0 + 1, 0, w - 1)
    y0c = np.clip(y0, 0, h - 1)
    y1c = np.clip(y0 + 1, 0, h - 1)
    top = channel[y0c, x0c] * (1.0 - fx) + channel[y0c, x1c] * fx
    bottom = channel[y1c, x0c] * (1.0 - fx) + channel[y1c, x1c] * fx
    return top * (1.0 - fy) + bottom * fy


def vignette(rgb: np.ndarray, amount: float, power: float = 2.4) -> np.ndarray:
    """Radial falloff. ``power`` sets how far the darkening reaches inwards:
    low values spread it into the mid-frame, high values keep it in the corners."""
    if amount <= 0.0:
        return rgb
    mask = 1.0 - amount * np.clip(_radius(rgb), 0.0, 1.0) ** power
    return rgb * np.maximum(mask, 0.0)[..., None]


def edge_softness(rgb: np.ndarray, amount: float) -> np.ndarray:
    """Blur that grows towards the frame edge, like a plastic meniscus lens."""
    if amount <= 0.0:
        return rgb
    sigma = max(1.0, 3.0 * rgb.shape[0] / 600.0)
    blurred = _gaussian(rgb, sigma)
    weight = (amount * np.clip(_radius(rgb), 0.0, 1.0) ** 1.6)[..., None]
    return rgb + (blurred - rgb) * weight


def chromatic_aberration(rgb: np.ndarray, corner_pixels: float) -> np.ndarray:
    """Lateral colour fringing: red and blue are scaled slightly apart."""
    if corner_pixels <= 0.0:
        return rgb
    h, w = rgb.shape[:2]
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    k = corner_pixels / (max(w, h) / 2.0)
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    out = rgb.copy()
    for index, scale in ((0, 1.0 + k), (2, 1.0 - k)):
        out[..., index] = _sample(rgb[..., index],
                                  cx + (xs - cx) * scale,
                                  cy + (ys - cy) * scale)
    return out


def barrel_distort(rgb: np.ndarray, amount: float) -> np.ndarray:
    """Bulge the centre outwards the way a cheap single-element lens does."""
    if amount <= 0.0:
        return rgb
    h, w = rgb.shape[:2]
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    half = max(w, h) / 2.0
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    dx = (xs - cx) / half
    dy = (ys - cy) / half
    scale = 1.0 - amount * (dx * dx + dy * dy)
    sx = cx + dx * scale * half
    sy = cy + dy * scale * half
    return np.stack([_sample(rgb[..., c], sx, sy) for c in range(3)], axis=2)


def light_leak(rgb: np.ndarray, amount: float, colour: tuple) -> np.ndarray:
    """Warm flare bleeding in from one corner, as on a leaky Diana back."""
    if amount <= 0.0:
        return rgb
    h, w = rgb.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = -0.15 * w, -0.10 * h
    distance = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2) / (max(w, h) * 0.85)
    grad = _gaussian(np.exp(-distance * 2.6).astype(np.float32),
                     max(w, h) * 0.02)
    return rgb + np.asarray(colour, dtype=np.float32) * grad[..., None] * amount


def add_grain(rgb: np.ndarray, preset: FilmPreset, rng: np.random.Generator) -> np.ndarray:
    amount = preset.grain
    if amount <= 0.0:
        return rgb
    h, w = rgb.shape[:2]
    size = max(preset.grain_size, 0.0)

    noise = rng.standard_normal((h, w)).astype(np.float32)
    if size > 1.0:
        noise = _gaussian(noise, (size - 1.0) * 0.9)
        noise /= max(float(noise.std()), 1e-4)

    # Film grain is most apparent in the mid-tones.
    luma = np.clip(_luma(rgb), 0.0, 1.0)
    weight = 0.35 + 0.65 * (1.0 - np.abs(2.0 * luma - 1.0))

    chroma = rng.standard_normal((h, w, 3)).astype(np.float32)
    if size > 1.0:
        chroma = _gaussian(chroma, (size - 1.0) * 1.2)
        chroma /= max(float(chroma.std()), 1e-4)

    mono_part = noise[..., None] * 0.85
    chroma_part = chroma * 0.12
    return rgb + (mono_part + chroma_part) * (weight[..., None] * amount)


def add_frame(img: Image.Image, border_px: int, colour: tuple) -> Image.Image:
    """Pad with an instant-film border of equal width on all four sides."""
    border = max(1, int(border_px))
    fill = tuple(round(min(max(c, 0.0), 1.0) * 255) for c in colour)
    canvas = Image.new("RGB", (img.width + 2 * border, img.height + 2 * border), fill)
    canvas.paste(img, (border, border))
    return canvas


# --------------------------------------------------------------------------
# preset definition
# --------------------------------------------------------------------------


@dataclass
class FilmPreset:
    name: str                      # slug used in filenames
    label: str                     # human readable name
    family: str                    # Kodak / Fujifilm / Creative / Mono
    description: str

    # sensor prep (fixes the washed-out ESP32-CAM source)
    auto_balance: float = 0.0      # 0..1 colour-cast removal
    auto_contrast: float = 0.0     # 0..1 histogram stretch

    # base grade
    exposure: float = 0.0          # EV
    wb: tuple = (1.0, 1.0, 1.0)    # linear R,G,B gains
    matrix: tuple | None = None    # 3x3 channel mixer
    saturation: float = 1.0

    # tone
    midtone_gamma: float = 1.0
    contrast: float = 0.0          # -1..1, smoothstep blend (subtle shaping)
    contrast_gain: float = 1.0     # linear slope about mid-grey; 1.10 = +10%
    highlight_rolloff: float = 0.0
    fade: float = 0.0              # black lift

    # grading
    shadow_tint: tuple = (0.0, 0.0, 0.0)
    midtone_tint: tuple = (0.0, 0.0, 0.0)
    highlight_tint: tuple = (0.0, 0.0, 0.0)

    # B&W
    monochrome: bool = False
    mono_mix: tuple = (0.30, 0.59, 0.11)
    mono_tint: tuple = (0.0, 0.0, 0.0)

    # optics
    softness: float = 0.0
    sharpen: float = 0.0            # unsharp-mask amount, 0.10 = +10%
    sharpen_radius: float = 1.0
    vignette: float = 0.0
    vignette_power: float = 2.4     # lower = falloff reaches further inwards
    edge_softness: float = 0.0      # extra blur towards the frame edge
    chromatic_aberration: float = 0.0   # lateral fringing, px at the corner
    barrel: float = 0.0             # barrel distortion strength
    light_leak: float = 0.0
    leak_colour: tuple = (1.0, 0.55, 0.20)

    # emulsion
    grain: float = 0.0
    grain_size: float = 1.0
    halation: float = 0.0
    halation_radius: float = 14.0
    halation_threshold: float = 0.72
    halation_colour: tuple = (1.0, 0.32, 0.12)

    # presentation
    frame_px: int = 0              # border width in pixels, 0 = no border
    frame_colour: tuple = (0.98, 0.975, 0.965)

    order: int = field(default=0)


def apply_filter(img: Image.Image, preset: FilmPreset, seed: int | None = None) -> Image.Image:
    """Run one preset over a PIL image and return the processed image."""
    if seed is None:
        seed = zlib.crc32(preset.name.encode("utf-8"))
    rng = np.random.default_rng(seed ^ zlib.crc32(str(img.size).encode("utf-8")))

    rgb = _to_float(img)

    rgb = auto_balance(rgb, preset.auto_balance)
    rgb = auto_contrast(rgb, preset.auto_contrast)
    rgb = exposure_wb(rgb, preset.exposure, preset.wb)

    if preset.matrix is not None:
        rgb = apply_matrix(rgb, preset.matrix)

    rgb = saturate(rgb, preset.saturation)
    rgb = tone(rgb, preset)
    rgb = tints(rgb, preset)

    if preset.monochrome:
        rgb = monochrome(rgb, preset)

    rgb = barrel_distort(rgb, preset.barrel)
    rgb = chromatic_aberration(rgb, preset.chromatic_aberration)
    rgb = halation(rgb, preset, rng)
    rgb = soften(rgb, preset.softness)
    rgb = edge_softness(rgb, preset.edge_softness)
    rgb = sharpen(rgb, preset.sharpen, preset.sharpen_radius)
    rgb = light_leak(rgb, preset.light_leak, preset.leak_colour)
    rgb = vignette(rgb, preset.vignette, preset.vignette_power)
    rgb = add_grain(rgb, preset, rng)

    out = _to_pil(rgb)
    if preset.frame_px > 0:
        out = add_frame(out, preset.frame_px, preset.frame_colour)
    return out


# --------------------------------------------------------------------------
# the 20 looks
# --------------------------------------------------------------------------

_PRESETS = [
    # ---------------- Kodak ----------------
    FilmPreset(
        name="kodak_portra_400",
        label="Kodak Portra 400",
        family="Kodak",
        description="Warm pastel skin tones, soft contrast, creamy highlights.",
        auto_balance=0.60, auto_contrast=0.45,
        exposure=0.05, wb=(1.02, 1.00, 0.98),
        saturation=0.92, midtone_gamma=0.98, contrast=0.14,
        shadow_tint=(0.004, 0.002, 0.004),
        highlight_tint=(0.035, 0.018, -0.012),
        softness=0.10, vignette=0.06,
        grain=0.010, grain_size=1.2,
    ),
    FilmPreset(
        name="kodak_portra_160",
        label="Kodak Portra 160",
        family="Kodak",
        description="Cleaner and slightly cooler than 400, fine grain, gentle skin.",
        auto_balance=0.65, auto_contrast=0.48,
        exposure=0.03, wb=(1.01, 1.00, 0.99),
        saturation=0.95, midtone_gamma=0.99, contrast=0.20,
        shadow_tint=(0.000, 0.002, 0.006),
        highlight_tint=(0.022, 0.014, -0.006),
        softness=0.07, vignette=0.06,
        grain=0.006, grain_size=1.1,
    ),
    FilmPreset(
        name="kodak_gold_200",
        label="Kodak Gold 200",
        family="Kodak",
        description="Golden hour warmth, saturated yellows, punchy consumer colour.",
        auto_balance=0.60, auto_contrast=0.52,
        exposure=0.06, wb=(1.06, 1.00, 0.93),
        saturation=1.12, midtone_gamma=0.97, contrast=0.32,
        shadow_tint=(0.008, 0.004, 0.000),
        highlight_tint=(0.050, 0.026, -0.034),
        vignette=0.07,
        grain=0.012, grain_size=1.2,
    ),
    FilmPreset(
        name="kodak_ektar_100",
        label="Kodak Ektar 100",
        family="Kodak",
        description="Vivid reds, cool blue shadows, crisp high-saturation negative.",
        auto_balance=0.70, auto_contrast=0.58,
        exposure=0.02, wb=(1.03, 1.00, 0.99),
        saturation=1.42, midtone_gamma=0.98, contrast=0.42,
        shadow_tint=(-0.004, 0.000, 0.014),
        highlight_tint=(0.020, 0.010, -0.006),
        softness=0.03, vignette=0.06,
        grain=0.006, grain_size=1.0,
    ),
    FilmPreset(
        name="kodak_ultramax_400",
        label="Kodak UltraMax 400",
        family="Kodak",
        description="Consumer-grade punch, warm-yellow cast, coarse visible grain.",
        auto_balance=0.50, auto_contrast=0.52,
        saturation=1.15, midtone_gamma=0.97, contrast=0.34,
        shadow_tint=(0.004, 0.006, -0.002),
        highlight_tint=(0.038, 0.020, -0.026),
        vignette=0.08,
        grain=0.016, grain_size=1.3,
    ),
    FilmPreset(
        name="kodak_colorplus_200",
        label="Kodak ColorPlus 200",
        family="Kodak",
        description="Warm, slightly faded, brown shadows - cheap-film nostalgia.",
        auto_balance=0.55, auto_contrast=0.38,
        saturation=0.98, midtone_gamma=1.01, contrast=0.18, fade=0.05,
        shadow_tint=(0.020, 0.012, 0.004),
        highlight_tint=(0.030, 0.018, -0.014),
        softness=0.10, vignette=0.09,
        grain=0.014, grain_size=1.3,
    ),
    FilmPreset(
        name="kodak_ektachrome_e100",
        label="Kodak Ektachrome E100",
        family="Kodak",
        description="Slide film: deep blacks, clean cool blues, high contrast.",
        auto_balance=0.70, auto_contrast=0.62,
        saturation=1.20, midtone_gamma=0.94, contrast=0.55,
        highlight_rolloff=0.10,
        shadow_tint=(-0.004, -0.002, 0.010),
        highlight_tint=(0.004, 0.004, 0.002),
        softness=0.02, vignette=0.05,
        grain=0.006, grain_size=1.0,
    ),
    FilmPreset(
        name="kodak_kodachrome_64",
        label="Kodak Kodachrome 64",
        family="Kodak",
        description="Deep rich reds, dense shadows, muted greens - the National Geographic look.",
        auto_balance=0.65, auto_contrast=0.60,
        matrix=(
            (1.06, -0.04, -0.02),
            (-0.02, 0.98, 0.03),
            (-0.01, 0.02, 0.99),
        ),
        saturation=1.05, midtone_gamma=0.95, contrast=0.48,
        shadow_tint=(0.000, 0.002, 0.008),
        highlight_tint=(0.018, 0.008, -0.004),
        vignette=0.06,
        grain=0.008, grain_size=1.1,
    ),

    # ---------------- Fujifilm ----------------
    FilmPreset(
        name="fuji_pro_400h",
        label="Fujifilm Pro 400H",
        family="Fujifilm",
        description="Airy pastel, minty green shadows, magenta highlights, very soft.",
        auto_balance=0.60, auto_contrast=0.38,
        exposure=0.05, wb=(0.99, 1.00, 1.01),
        saturation=0.86, midtone_gamma=1.02, contrast=0.08, fade=0.04,
        shadow_tint=(-0.008, 0.008, 0.012),
        highlight_tint=(0.014, 0.006, 0.014),
        softness=0.12, vignette=0.05,
        grain=0.010, grain_size=1.2,
    ),
    FilmPreset(
        name="fuji_superia_400",
        label="Fujifilm Superia 400",
        family="Fujifilm",
        description="Punchy greens, cyan-green shadows, contrasty everyday Fuji look.",
        auto_balance=0.55, auto_contrast=0.56,
        wb=(0.98, 1.01, 1.00),
        saturation=1.28, midtone_gamma=0.97, contrast=0.36,
        shadow_tint=(-0.010, 0.010, 0.010),
        highlight_tint=(0.020, 0.014, -0.006),
        vignette=0.08,
        grain=0.017, grain_size=1.3,
    ),
    FilmPreset(
        name="fuji_c200",
        label="Fujifilm C200",
        family="Fujifilm",
        description="Cool neutral, mild contrast, subtle green shadows.",
        auto_balance=0.65, auto_contrast=0.48,
        wb=(0.99, 1.00, 1.02),
        saturation=1.05, midtone_gamma=0.99, contrast=0.26,
        shadow_tint=(-0.006, 0.004, 0.010),
        highlight_tint=(0.008, 0.006, 0.004),
        vignette=0.06,
        grain=0.010, grain_size=1.2,
    ),
    FilmPreset(
        name="fuji_velvia_50",
        label="Fujifilm Velvia 50",
        family="Fujifilm",
        description="Extreme saturation, magenta-red, deep blacks, landscape slide film.",
        auto_balance=0.70, auto_contrast=0.66,
        matrix=(
            (1.10, -0.06, -0.02),
            (-0.03, 1.09, -0.05),
            (-0.03, -0.04, 1.08),
        ),
        saturation=1.55, midtone_gamma=0.92, contrast=0.65,
        highlight_rolloff=0.10,
        shadow_tint=(-0.010, -0.004, 0.016),
        highlight_tint=(0.024, 0.004, 0.004),
        vignette=0.07,
        grain=0.005, grain_size=1.0,
    ),
    FilmPreset(
        name="fuji_provia_100f",
        label="Fujifilm Provia 100F",
        family="Fujifilm",
        description="Neutral professional slide: accurate colour, high contrast, clean.",
        auto_balance=0.70, auto_contrast=0.62,
        saturation=1.16, midtone_gamma=0.95, contrast=0.50,
        highlight_rolloff=0.08,
        shadow_tint=(-0.004, 0.002, 0.008),
        highlight_tint=(0.006, 0.006, 0.002),
        vignette=0.05,
        grain=0.006, grain_size=1.0,
    ),
    FilmPreset(
        name="fuji_astia_100f",
        label="Fujifilm Astia 100F",
        family="Fujifilm",
        description="Soft muted slide, gentle contrast - flattering for portraits.",
        auto_balance=0.65, auto_contrast=0.46,
        saturation=0.95, midtone_gamma=1.00, contrast=0.26,
        shadow_tint=(-0.002, 0.004, 0.006),
        highlight_tint=(0.010, 0.008, 0.002),
        softness=0.08, vignette=0.05,
        grain=0.008, grain_size=1.1,
    ),
    FilmPreset(
        name="fuji_classic_chrome",
        label="Fujifilm Classic Chrome",
        family="Fujifilm",
        description="Desaturated documentary look, muted colours, cyan shadows.",
        auto_balance=0.60, auto_contrast=0.52,
        saturation=0.72, midtone_gamma=0.96, contrast=0.34, fade=0.03,
        shadow_tint=(-0.008, 0.004, 0.018),
        highlight_tint=(0.012, 0.014, 0.012),
        vignette=0.07,
        grain=0.009, grain_size=1.1,
    ),
    FilmPreset(
        name="fuji_eterna",
        label="Fujifilm Eterna (cinema)",
        family="Fujifilm",
        description="Motion-picture negative: milky blacks, green shadows, muted colour.",
        auto_balance=0.55, auto_contrast=0.38,
        saturation=0.85, midtone_gamma=1.02, contrast=0.16, fade=0.05,
        shadow_tint=(-0.008, 0.014, 0.008),
        highlight_tint=(0.010, 0.014, 0.000),
        softness=0.10, vignette=0.08,
        grain=0.012, grain_size=1.2,
    ),

    # ---------------- Creative ----------------
    FilmPreset(
        name="cinestill_800t",
        label="Cinestill 800T",
        family="Creative",
        description="Tungsten-balanced Kodak Vision3: cool daylight cast, heavy red halation.",
        auto_balance=0.60, auto_contrast=0.48,
        wb=(0.94, 1.00, 1.08),
        saturation=1.12, midtone_gamma=1.00, contrast=0.30, fade=0.05,
        shadow_tint=(-0.010, 0.002, 0.018),
        highlight_tint=(0.012, 0.008, 0.006),
        softness=0.06, vignette=0.09,
        grain=0.018, grain_size=1.4,
        halation=0.28, halation_radius=10.0, halation_threshold=0.86,
    ),
    FilmPreset(
        name="cross_process",
        label="Cross Process (E6 in C-41)",
        family="Creative",
        description="Wild slide-film-in-negative-chemistry shift: cyan shadows, magenta highlights.",
        auto_balance=0.50, auto_contrast=0.55,
        matrix=(
            (1.15, -0.10, -0.02),
            (-0.05, 1.02, 0.05),
            (0.02, -0.06, 1.05),
        ),
        saturation=1.30, midtone_gamma=0.90, contrast=0.50,
        shadow_tint=(-0.020, 0.010, 0.030),
        highlight_tint=(0.040, 0.000, 0.020),
        vignette=0.11,
        grain=0.012, grain_size=1.2,
    ),
    FilmPreset(
        name="expired_faded",
        label="Expired Film (faded)",
        family="Creative",
        description="Lifted blacks, low contrast, warm haze - found-film aesthetic.",
        auto_balance=0.40, auto_contrast=0.28,
        saturation=0.85, midtone_gamma=1.04, contrast=0.02, fade=0.07,
        shadow_tint=(0.030, 0.020, 0.010),
        highlight_tint=(0.030, 0.020, -0.010),
        softness=0.10, vignette=0.11,
        grain=0.014, grain_size=1.4,
    ),
    FilmPreset(
        name="ilford_hp5_bw",
        label="Ilford HP5 Plus (B&W)",
        family="Mono",
        description="Panchromatic black & white: deep blacks, smooth mid-tones, coarse grain.",
        auto_balance=0.70, auto_contrast=0.56,
        monochrome=True,
        mono_mix=(0.30, 0.59, 0.11),
        mono_tint=(0.004, 0.002, -0.004),
        saturation=1.0, midtone_gamma=0.98, contrast=0.40,
        vignette=0.09,
        grain=0.024, grain_size=1.5,
    ),

    # ---------------- Toy camera ----------------
    FilmPreset(
        name="diana_f",
        label="Diana F+ (toy camera)",
        family="Toy",
        description="Plastic meniscus lens: heavy corner vignette, dreamy soft "
                    "edges, low contrast, warm cast, barrel bulge and a light leak.",
        auto_balance=0.45, auto_contrast=0.30,
        exposure=0.10, wb=(1.04, 1.00, 0.95),
        saturation=1.15, midtone_gamma=0.98, contrast=0.02, fade=0.05,
        shadow_tint=(0.014, 0.026, -0.004),
        highlight_tint=(0.022, 0.018, -0.014),
        softness=0.05,
        vignette=0.82, vignette_power=1.7,
        edge_softness=0.90,
        chromatic_aberration=1.8,
        barrel=0.10,
        light_leak=0.22,
        grain=0.020, grain_size=1.3,
        # Unsharp plastic lens scatters light: a broad warm veiling glow.
        halation=0.22, halation_radius=20.0,
        halation_threshold=0.55, halation_colour=(0.60, 0.45, 0.32),
    ),

    # ---------------- Utility ----------------
    FilmPreset(
        name="cf01",
        label="CF01 (+5% sat, +10% sharpen, light vignette)",
        family="CF",
        description="Neutral enhancement only: saturation +5%, unsharp mask "
                    "+10%, and a very light corner vignette. No film grade.",
        saturation=1.05,
        sharpen=0.10,
        vignette=0.04,
    ),
]

# --------------------------------------------------------------------------
# CF family - neutral enhancement variations on the CF01 baseline
# --------------------------------------------------------------------------


def _cf_variant(slug: str, what: str, description: str, **overrides) -> FilmPreset:
    """A CF01-style look with only the named knobs changed."""
    settings = {"family": "CF", "saturation": 1.05, "sharpen": 0.10, "vignette": 0.04}
    settings.update(overrides)
    return FilmPreset(
        name=slug,
        label=f"{slug.upper()} ({what})",
        description=description,
        **settings,
    )


_PRESETS.extend([
    # --- contrast ladder (linear gain about mid-grey) ---
    _cf_variant("cf02", "contrast +10%",
                "Gentlest step up from CF01.", contrast_gain=1.10),
    _cf_variant("cf03", "contrast +20%",
                "Clear punch without touching colour or exposure.", contrast_gain=1.20),
    _cf_variant("cf04", "contrast +35%",
                "Snappy mid-tones, still natural.", contrast_gain=1.35),
    _cf_variant("cf05", "contrast +50%",
                "Hard contrast; highlights start to crowd together.", contrast_gain=1.50),
    _cf_variant("cf06", "contrast +75%",
                "Near slide-film bite, shadows go dense.", contrast_gain=1.75),

    # --- contrast plus mid-tone shaping ---
    _cf_variant("cf07", "contrast +35%, midtones up",
                "CF04 with mid-tones lifted, so it punches without going muddy.",
                contrast_gain=1.35, midtone_gamma=0.88),
    _cf_variant("cf08", "contrast +50%, midtones up",
                "CF05 with the mid-tones raised back up.",
                contrast_gain=1.50, midtone_gamma=0.88),
    _cf_variant("cf09", "contrast +20%, midtones down",
                "CF03 with mid-tones pulled down for a moodier frame.",
                contrast_gain=1.20, midtone_gamma=1.12),

    # --- saturation ladder ---
    _cf_variant("cf10", "saturation 1.10",
                "Colour +10%.", saturation=1.10),
    _cf_variant("cf11", "saturation 1.21",
                "Colour +20%.", saturation=1.21),
    _cf_variant("cf12", "saturation 1.35",
                "Colour +30%, vivid.", saturation=1.35),
    _cf_variant("cf13", "saturation 0.95",
                "Colour muted 10% - a calmer frame.", saturation=0.95),

    # --- sharpening ladder ---
    _cf_variant("cf14", "sharpen 0.20",
                "Double the CF01 sharpening.", sharpen=0.20),
    _cf_variant("cf15", "sharpen 0.35",
                "Aggressive; crisp but starting to look digital.", sharpen=0.35),

    # --- vignette ladder ---
    _cf_variant("cf16", "vignette 0.12",
                "Triple the corner falloff of CF01.", vignette=0.12),
    _cf_variant("cf17", "vignette 0.22",
                "Obvious corner darkening.", vignette=0.22),

    # --- exposure ---
    _cf_variant("cf18", "exposure +0.3 EV",
                "Half a stop brighter, everything else unchanged.", exposure=0.30),
    _cf_variant("cf19", "exposure -0.3 EV",
                "Half a stop darker.", exposure=-0.30),

    # --- combined looks ---
    _cf_variant("cf20", "punchy",
                "Contrast +30%, colour +15%, extra sharpening - the 'make it pop' preset.",
                contrast_gain=1.30, saturation=1.20, sharpen=0.18),
    _cf_variant("cf21", "vivid landscape",
                "Contrast +25% and colour +30%, minimal vignette: skies and foliage.",
                contrast_gain=1.25, saturation=1.35, vignette=0.02),
])

# --------------------------------------------------------------------------
# Instant film
# --------------------------------------------------------------------------

_PRESETS.append(
    FilmPreset(
        name="polaroid_sx70",
        label="Polaroid SX-70",
        family="Instant",
        description="Integral instant film: olive-lifted blacks, muted ochre "
                    "mid-tones, cyan shadows, creamy rolled-off highlights.",
        auto_balance=0.55, auto_contrast=0.45,
        exposure=0.02, wb=(1.04, 1.00, 0.95),
        matrix=(
            (1.02, 0.00, -0.01),
            (0.00, 1.00, 0.00),
            (-0.01, -0.02, 1.00),
        ),
        saturation=0.80, midtone_gamma=0.96, contrast=0.05, contrast_gain=1.12,
        highlight_rolloff=0.15, fade=0.055,
        shadow_tint=(-0.008, 0.004, 0.020),
        midtone_tint=(0.010, 0.004, -0.012),
        highlight_tint=(0.016, 0.004, 0.002),
        softness=0.06,
        vignette=0.12, vignette_power=2.4,
        grain=0.012, grain_size=1.2,
        halation=0.10, halation_radius=12.0,
        halation_threshold=0.86, halation_colour=(0.95, 0.70, 0.50),
    )
)

# The same chemistry shown inside an instant-film border of equal width on all
# four sides. Built with dataclasses.replace so the two presets cannot drift.
_SX70 = next(_p for _p in _PRESETS if _p.name == "polaroid_sx70")
_PRESETS.append(
    replace(
        _SX70,
        name="polaroid_sx70_border",
        label="Polaroid SX-70 (white border)",
        description="Polaroid SX-70 chemistry inside a white instant-film "
                    "border of equal width on all four sides.",
        frame_px=20,
    )
)

# Registry in presentation order, 1-based numbering used by the contact sheets.
FILTERS: dict[str, FilmPreset] = {}
for _i, _p in enumerate(_PRESETS, start=1):
    _p.order = _i
    FILTERS[_p.name] = _p

FILTER_ORDER: list[str] = [p.name for p in _PRESETS]


def get_filter(name_or_number) -> FilmPreset:
    """Resolve a filter by slug, label, or 1-based number."""
    key = str(name_or_number).strip()
    if key in FILTERS:
        return FILTERS[key]
    if key.isdigit():
        idx = int(key)
        if 1 <= idx <= len(_PRESETS):
            return _PRESETS[idx - 1]
    lowered = key.lower()
    for preset in _PRESETS:
        if preset.name == lowered or preset.label.lower() == lowered:
            return preset
    raise KeyError(f"unknown filter: {name_or_number!r}")
