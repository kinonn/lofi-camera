# Film look lab

Host-side test bench for choosing a **film-camera colour look** for the JPEGs
that `lofi-camera` saves to the SD card.

It runs every look defined in `film_emulation.py` — Kodak / Fujifilm /
creative film stocks, a Diana F+ toy camera, and a neutral `cf01` enhancement
pass — over the sample captures in `../../test-images/` so a look can be picked
by eye **before** anything is written in C++ for the ESP32. No firmware file is
touched by this tool.

---

## Setup

The lab uses `uv` for its virtual environment:

```powershell
uv venv .venv --python 3.10
uv pip install --python .venv\Scripts\python.exe -r tools\film_lab\requirements.txt
```

## Run

```powershell
# every look over every sample image
.venv\Scripts\python.exe tools\film_lab\run_film_lab.py

# what are the looks?
.venv\Scripts\python.exe tools\film_lab\run_film_lab.py --list

# only some of them, by number or slug
.venv\Scripts\python.exe tools\film_lab\run_film_lab.py --filters 1 12 fuji_velvia_50

# one whole family
.venv\Scripts\python.exe tools\film_lab\run_film_lab.py --family CF

# a single image, smaller contact sheets for quick browsing
.venv\Scripts\python.exe tools\film_lab\run_film_lab.py --input test-images/img_0027.jpg --thumb-width 190
```

Then open `tools/film_lab/out/index.html` — it shows the original next to all
results per sample photo. Note the **numbers** you like.

### Output layout

```
tools/film_lab/out/
  <filter_slug>/<image>__<filter_slug>.jpg     full-size result, e.g.
                                               kodak_gold_200/img_0027__kodak_gold_200.jpg
  contact_sheets/<image>__contact_sheet.jpg    original + all looks, labelled
  index.html                                   gallery
```

Every output filename carries the filter slug, so a result is never ambiguous.

---

## The looks

| # | Filter | Family | Character |
|---|---|---|---|
| 01 | `kodak_portra_400` | Kodak | Warm pastel skin, soft contrast, creamy highlights |
| 02 | `kodak_portra_160` | Kodak | Cleaner and cooler than 400, fine grain |
| 03 | `kodak_gold_200` | Kodak | Golden warmth, saturated yellows, punchy |
| 04 | `kodak_ektar_100` | Kodak | Vivid reds, cool blue shadows, crisp |
| 05 | `kodak_ultramax_400` | Kodak | Consumer punch, warm-yellow cast, coarse grain |
| 06 | `kodak_colorplus_200` | Kodak | Warm, faded, brown shadows |
| 07 | `kodak_ektachrome_e100` | Kodak | Slide: deep blacks, clean cool blues |
| 08 | `kodak_kodachrome_64` | Kodak | Rich reds, dense shadows, muted greens |
| 09 | `fuji_pro_400h` | Fujifilm | Airy pastel, mint shadows, magenta highlights |
| 10 | `fuji_superia_400` | Fujifilm | Punchy greens, cyan-green shadows |
| 11 | `fuji_c200` | Fujifilm | Cool neutral, mild, subtle green shadows |
| 12 | `fuji_velvia_50` | Fujifilm | Extreme saturation, magenta-red, deep blacks |
| 13 | `fuji_provia_100f` | Fujifilm | Neutral slide, high contrast, clean |
| 14 | `fuji_astia_100f` | Fujifilm | Soft muted slide, gentle contrast |
| 15 | `fuji_classic_chrome` | Fujifilm | Desaturated documentary, cyan shadows |
| 16 | `fuji_eterna` | Fujifilm | Cinema: milky blacks, green shadows, muted |
| 17 | `cinestill_800t` | Creative | Tungsten Vision3: cool cast, red halation |
| 18 | `cross_process` | Creative | E6-in-C-41: cyan shadows, magenta highlights |
| 19 | `expired_faded` | Creative | Lifted blacks, warm haze, low contrast |
| 20 | `ilford_hp5_bw` | Mono | Panchromatic B&W, deep blacks, coarse grain |
| 21 | `diana_f` | Toy | Plastic lens: heavy vignette, dreamy edges, low contrast, barrel bulge, light leak |
| 22 | `cf01` | CF | Neutral: saturation +5%, unsharp mask +10%, very light vignette |
| 23 | `cf02` | CF | Contrast +10% |
| 24 | `cf03` | CF | Contrast +20% |
| 25 | `cf04` | CF | Contrast +35% |
| 26 | `cf05` | CF | Contrast +50% |
| 27 | `cf06` | CF | Contrast +75% |
| 28 | `cf07` | CF | Contrast +35%, mid-tones lifted |
| 29 | `cf08` | CF | Contrast +50%, mid-tones lifted |
| 30 | `cf09` | CF | Contrast +20%, mid-tones lowered |
| 31 | `cf10` | CF | Saturation +10% |
| 32 | `cf11` | CF | Saturation +20% |
| 33 | `cf12` | CF | Saturation +30% |
| 34 | `cf13` | CF | Saturation −10% |
| 35 | `cf14` | CF | Sharpen 0.20 |
| 36 | `cf15` | CF | Sharpen 0.35 |
| 37 | `cf16` | CF | Vignette 0.12 |
| 38 | `cf17` | CF | Vignette 0.22 |
| 39 | `cf18` | CF | Exposure +0.3 EV |
| 40 | `cf19` | CF | Exposure −0.3 EV |
| 41 | `cf20` | CF | Punchy: contrast +30%, saturation 1.20, sharpen 0.18 |
| 42 | `cf21` | CF | Vivid landscape: contrast +25%, saturation 1.35, vignette 0.02 |
| 43 | `polaroid_sx70` | Instant | Integral instant film: olive lifted blacks, ochre mid-tones, cyan shadows, creamy highlights |
| 44 | `polaroid_sx70_border` | Instant | The same SX-70 chemistry inside a white border of equal width on all four sides |
| 45 | `vintage_warm` | Creative | Reference-matched grade: lifted blacks, rolled cream highlights, deeper mid-tones, colour boosted |

---

## How a look is built

All looks share one pipeline in `film_emulation.py`; only the preset numbers
differ. This matters for porting — the firmware needs to implement *one*
chain, not one algorithm per look.

```
auto-balance      remove the source colour cast (mid-tones only)
auto-contrast     stretch the flat histogram, leaving highlight headroom
exposure + WB     per-channel gain, done in linear light
colour matrix     3x3 channel mixer (only the "character" films use it)
saturation        mix toward Rec.709 luma
tone curve        midtone gamma -> contrast (smoothstep and/or gain) -> highlight shoulder -> black lift
tints             additive shadow, mid-tone and highlight colour
monochrome        optional channel-mixed B&W
barrel            radial warp, bulges the centre outwards (toy lens)
chromatic aberr.  lateral red/blue fringing
halation          bloom bled off highlights: red for film, warm veiling glow for toy glass
softness          blend with a blurred copy
edge softness     extra blur towards the frame edge, sharp-ish centre
sharpen           unsharp mask
light leak        warm flare bleeding in from one corner
vignette          radial falloff; `vignette_power` sets how far in it reaches
grain             luma-weighted monochrome + slight chroma noise
```

Notes on the design:

- **Exposure and white balance run in linear light**; everything else runs on
  gamma-encoded sRGB, which is how film looks are normally tuned.
- **Contrast is applied to luma** and the colour is rescaled by the luma ratio,
  so boosting contrast does not smear hues the way per-channel curves do.
- **`auto_contrast` deliberately stops at 0.96**, not 1.0. The sample JPEGs
  already have a near-clipped sky; landing the histogram on pure white made
  every look read as the same blown-out highlight.
- **`auto_balance` is a mid-tone gray-world estimate.** It is applied *before*
  the grade so each film keeps its own colour idea instead of fighting a green
  cast. On warm indoor light it will neutralise some of the warmth — that is
  intentional but worth knowing when judging the portrait sample.
- **Halation is not peak-normalised**, otherwise a large bright region like an
  overcast sky would be forced to full bloom just for being the brightest thing
  in frame.

## Tuning a look

Edit the preset literal in `film_emulation.py` and re-run. Useful knobs:

| Knob | Effect |
|---|---|
| `auto_balance` / `auto_contrast` | how much of the flat source is corrected first |
| `exposure` | EV, linear |
| `wb` | R, G, B linear gains — the cheapest way to warm/cool a look |
| `saturation` | < 1 mutes, > 1 enriches |
| `midtone_gamma` | < 1 brightens mid-tones, > 1 darkens |
| `contrast` | -1..1 smoothstep strength (gentle shaping) |
| `contrast_gain` | linear slope about mid-grey; `1.20` = +20%, clips like real contrast |
| `fade` | black lift; the "washed-out negative" knob |
| `shadow_tint` / `midtone_tint` / `highlight_tint` | additive split-tone offsets for shadows, mid-grey and highlights |
| `grain` / `grain_size` | grain strength / coarseness |
| `halation` | red bloom strength |
| `sharpen` / `sharpen_radius` | unsharp mask amount (`0.10` = +10%) and radius |
| `vignette` / `vignette_power` | corner darkening / how far inwards it reaches (lower = wider) |
| `edge_softness` | blur that grows towards the frame edge, centre stays sharp |
| `barrel` | barrel distortion; 0 = off |
| `chromatic_aberration` | lateral fringing in pixels at the frame corner |
| `light_leak` | warm flare from one corner; set `0` to disable |
| `frame_px` / `frame_colour` | white instant-film border width in pixels, and its colour |

## The Diana F+ preset

`21 diana_f` models the plastic meniscus lens of a Diana F+ rather than a film
stock, so it uses the lens stages the other looks leave at zero:

- `vignette=0.82, vignette_power=1.7` — very heavy, and reaching well into the
  mid-frame, so the frame edges fall towards black.
- `edge_softness=0.90` — the centre stays acceptably sharp while the outer frame
  smears, which is what the single-element lens actually does.
- `barrel=0.10` — the centre bulges, straight lines bow outwards.
- `chromatic_aberration=1.8` — red and blue fringes of ~2 px at the corners.
- `halation` with a neutral warm colour — the broad veiling glow of light
  scattering inside cheap glass, not the red halation of a film base.
- `light_leak=0.22` — a warm flare from the top-left. **Set this to `0` if you
  want the lens look without the leak.**
- Plus `contrast=0.02`, `fade=0.05` and `auto_contrast=0.30`, which give the
  washed, low-contrast rendering.

The preset does **not** crop to the Diana's native 6x6 square frame — your
sampled captures stay 4:3. Add a square crop in `apply_filter` if you want the
full toy-camera presentation.

## The Polaroid SX-70 preset

`43 polaroid_sx70` models the SX-70 integral film's *chemistry* rather than a
lens, so it leans on the colour and tone stages:

- `fade=0.055` with `highlight_rolloff=0.15` — the short, compressed tonal range
  of integral film: blacks lift to dark olive rather than black, and highlights
  roll off to cream instead of clipping.
- `saturation=0.80` — SX-70 dyes are muted; vivid colour is the quickest way to
  spot a frame that is not real instant film.
- A three-way split tone, which is what actually makes it read as Polaroid:
  `shadow_tint` cyan-blue, `midtone_tint` ochre, `highlight_tint` warm cream.
  The mid-tone tint is weighted `4 * luma * (1 - luma)`, so it peaks at mid-grey
  and vanishes at both ends.
- `contrast_gain=1.12` with `contrast=0.05` — just enough structure to stop the
  frame going foggy. An earlier attempt with weak contrast, heavy fade and a
  strong rolloff produced a uniform pink haze over the whole image instead.
- A gentle warm highlight bloom (`halation=0.10` above luma 0.86) and
  `vignette=0.12` for the slight corner falloff of the plastic body.

### Bordered presentation: `44 polaroid_sx70_border`

The same preset with `frame_px=20`, which pads the render with a white border of
equal width on all four sides - 20 px per side on an 800x600 source, giving an
840x640 output. It is built with `dataclasses.replace` from `polaroid_sx70`, so
the chemistry cannot drift between the two presets.

- `frame_px` is an absolute pixel width, so the border is exactly the size you
  ask for. It does not scale with the image, so a `--scale 0.5` preview shows it
  proportionally twice as thick.
- `frame_colour` defaults to `(0.98, 0.975, 0.965)` - a slightly warm white,
  because instant-film borders yellow a little. Use `(1, 1, 1)` for pure white.
- The border is drawn **after** the vignette and grain, so neither bleeds into it.

A real Polaroid has a much deeper bottom edge. This variant uses an equal border
on all four sides; for the classic look, give `add_frame` a separate bottom
multiplier.

Because bordered renders have a different aspect ratio from the originals, the
contact sheet sizes its cells from the tallest thumbnail rather than assuming
4:3 - otherwise the label bar would overlap the image.

## The CF family (CF01 - CF21)

`22 cf01` is not a film look. It is a neutral enhancement pass, useful as a
baseline and as a sanity check on the pipeline:

| Knob | Value | Effect |
|---|---|---|
| `saturation` | `1.05` | colours +5% (mixed about Rec.709 luma) |
| `sharpen` | `0.10` | unsharp mask, +10% of the high-pass residual |
| `vignette` | `0.04` | corners fall ~4%; the centre is untouched |

Every film-grade stage (auto-balance, auto-contrast, exposure, WB, tints,
grain, halation, softness, blur, leak) is left at its default. The pipeline is
a **pixel-identical no-op** when all of them are zero, so CF01 output is the
untouched JPEG plus exactly those three adjustments.

One caveat on "+10%": the amount is applied to the high-pass residual, so it is
exact wherever the image has headroom. On this camera's JPEGs 11% of a typical
frame is already clipped at white, where the extra detail cannot be added — the
measured global effect is therefore ~7%.

### Variations: CF02 - CF21

Twenty variants on the same CF01 base (saturation +5%, sharpen +10%,
vignette 0.04). Only the knob listed changes. Render just this family with
`--family CF`.

| # | Filter | Change from CF01 |
|---|---|---|
| 23 | `cf02` | contrast +10% |
| 24 | `cf03` | contrast +20% |
| 25 | `cf04` | contrast +35% |
| 26 | `cf05` | contrast +50% |
| 27 | `cf06` | contrast +75% |
| 28 | `cf07` | contrast +35%, mid-tones lifted |
| 29 | `cf08` | contrast +50%, mid-tones lifted |
| 30 | `cf09` | contrast +20%, mid-tones lowered |
| 31 | `cf10` | saturation 1.10 (+10%) |
| 32 | `cf11` | saturation 1.21 (+20%) |
| 33 | `cf12` | saturation 1.35 (+30%) |
| 34 | `cf13` | saturation 0.95 (-10%) |
| 35 | `cf14` | sharpen 0.20 |
| 36 | `cf15` | sharpen 0.35 |
| 37 | `cf16` | vignette 0.12 |
| 38 | `cf17` | vignette 0.22 |
| 39 | `cf18` | exposure +0.3 EV |
| 40 | `cf19` | exposure -0.3 EV |
| 41 | `cf20` | punchy: contrast +30%, saturation 1.20, sharpen 0.18 |
| 42 | `cf21` | vivid landscape: contrast +25%, saturation 1.35, vignette 0.02 |

**Contrast here is a real slope change.** `contrast_gain` scales luma linearly
about mid-grey (0.5), so `1.20` genuinely separates tones by 20% more, and
values pushed past 0 or 1 clip - which is what raising contrast actually does.
Measured on `img_0038`, the p5-p95 tonal spread runs 1.00, 1.04, 1.07, 1.13,
1.18, 1.26 across CF01 and `cf02`-`cf06`. It lands under the nominal gain only
because that frame's sky is already clipped at white.

This is deliberately *not* the `contrast` field the film looks use. That one is
a smoothstep blend on `0..1` and is far too gentle for "I want more contrast" -
the CF ladder needed a different, honest knob.

## The vintage warm grade (45)

`45 vintage_warm` is not tied to a film stock — it was matched to four
before/after reference pairs, each showing the left half ungraded and the right
half graded. Measuring the two halves gives a consistent signature:

| Metric | Before → after |
|---|---|
| Black point (luma p1) | **+0.023** — lifted |
| Highlights (luma p99) | **−0.083** — rolled off, never clipped |
| HSV saturation | **+0.11** |
| R−B warmth | **+0.04** |

That combination is not a faded look, which is the easy mistake: blacks come up
*and* colour goes up. The preset reproduces it in two moves — `fade` lifts the
blacks, while `midtone_gamma=1.28` and `highlight_rolloff=0.34` pull the top
back down — then `wb` and the three tints supply the warmth and `saturation`
the colour push.

Run on the four `test-images/` captures, the deltas land within 0.03 of those
targets (`p1` exact, `p50` −0.061, `p99` −0.057, `sat` +0.084, warmth +0.038).

One tuning note worth keeping: the highlight tint has to be **red-dominant**,
with green barely moved. Lifting R and G together over a blue sky reads as
mint-green, whereas every reference only ever goes cream.

## Porting to the firmware (later)

Nothing here is on the device yet. When a look is chosen, the same chain has to
be reimplemented in C++ over the decoded JPEG. The cheap parts are luma gamma,
contrast, saturation and the tints; `auto_balance`/`auto_contrast` need a
one-pass mean/percentile estimate; grain needs a PRNG; halation and softness
need a separable blur. Presets that only use tone + tint + grain will be far
cheaper on an ESP32 than ones that need blurs.
