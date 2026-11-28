#pragma once
#include <Arduino.h>
#include <string.h>
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// Post-processing pipeline applied to a captured JPEG before it is written to
// the SD card. The look and the border are selected at runtime:
//
//   NONE             write the camera frame unchanged (zero cost)
//   WITH_BORDER      image unchanged, inset into a larger canvas with a white
//                    frame around it (no crop, no resampling)
//   SUPERIA_USM      Fuji Superia 400 look + unsharp sharpen (retired from the
//                    button cycle; kept for the host test)
//   KODACHROME_USM   Kodak Kodachrome 64 look + vibrance + unsharp sharpen
//   TMAX_USM         Kodak T-Max 400 panchromatic B&W + unsharp sharpen
//   PORTRA_USM       Kodak Portra 400 warm golden-pastel + unsharp sharpen
//
// The four looks can also be combined with the same white frame. The runtime
// state is {look, border}; the long-press border toggle in main.cpp turns any
// look (including NONE) into its bordered variant. NONE + border is the filter
// formerly called WHITE_BORDER, now WITH_BORDER.
//
// There is no compile-time filter switch: jpegFilterSetLook() /
// jpegFilterSetBorder() select the active state, and jpegFilterStream() reads
// it per capture. The old selection flags fail loudly instead of being
// silently ignored.
// ---------------------------------------------------------------------------
#ifdef JPEG_FILTER
#error "JPEG_FILTER is no longer a compile-time switch; use FILTER_DEFAULT_LOOK / FILTER_DEFAULT_BORDER"
#endif
#ifdef JPEG_FILTER_BORDER_PX
#error "JPEG_FILTER_BORDER_PX was removed; use JPEG_FILTER_INSET_PX"
#endif
#ifdef JPEG_FILTER_SWAP565
#error "JPEG_FILTER_SWAP565 was removed; the pipeline always swaps before encode"
#endif

enum FilterLook : uint8_t {
  FILTER_LOOK_NONE = 0,
  FILTER_LOOK_SUPERIA,
  FILTER_LOOK_KODACHROME,
  FILTER_LOOK_TMAX,
  FILTER_LOOK_PORTRA,
  FILTER_LOOK_COUNT
};

#ifndef FILTER_DEFAULT_LOOK
#define FILTER_DEFAULT_LOOK FILTER_LOOK_TMAX
#endif
#ifndef FILTER_DEFAULT_BORDER
#define FILTER_DEFAULT_BORDER 0
#endif

static_assert(FILTER_DEFAULT_LOOK >= 0 &&
                  FILTER_DEFAULT_LOOK < FILTER_LOOK_COUNT,
              "FILTER_DEFAULT_LOOK must be one of FILTER_LOOK_*");

// Border width, in output pixels, on every side. The canvas grows by 2x this;
// the image itself is never cropped. One knob for every bordered variant,
// including WITH_BORDER.
#ifndef JPEG_FILTER_INSET_PX
#define JPEG_FILTER_INSET_PX 24
#endif
static_assert(JPEG_FILTER_INSET_PX >= 0,
              "JPEG_FILTER_INSET_PX must be non-negative");
static_assert(2 * JPEG_FILTER_INSET_PX + 1600 < 65535,
              "JPEG_FILTER_INSET_PX is too large for uint16_t output dimensions");

// Runtime selection. The setters validate / wrap so the rest of the firmware
// never has to clamp a look index. jpegFilterSetLook()/NextLook() are defined
// below FILM_LOOKS because they also refresh the active parameter pointer.
static FilterLook gFilterLook = FILTER_DEFAULT_LOOK;
static bool gFilterBorder = FILTER_DEFAULT_BORDER;

static inline FilterLook jpegFilterGetLook(void) { return gFilterLook; }
static inline void jpegFilterSetBorder(bool on) { gFilterBorder = on; }
static inline bool jpegFilterGetBorder(void) { return gFilterBorder; }
static inline bool jpegFilterToggleBorder(void) {
  gFilterBorder = !gFilterBorder;
  return gFilterBorder;
}
static inline const char* jpegFilterLookName(FilterLook look) {
  switch (look) {
    case FILTER_LOOK_NONE:       return "None";
    case FILTER_LOOK_SUPERIA:    return "Superia 400";
    case FILTER_LOOK_KODACHROME: return "Kodachrome 64";
    case FILTER_LOOK_TMAX:       return "T-Max 400";
    case FILTER_LOOK_PORTRA:     return "Portra 400";
    default:                     return "?";
  }
}

// The pipeline streams its output instead of buffering the encoded JPEG, so a
// filter never has to hold the file in RAM. That is what keeps peak PSRAM
// inside the 8 MB module.
typedef size_t (*JpegWriteFn)(void* arg, const uint8_t* data, size_t len);

#include "esp32-hal-psram.h"
#include "img_converters.h"

// TEMP diagnostic: -DCAPTURE_TIMING=1 records per-stage ms; main.cpp appends a
// line per shot to /timing.txt. Remove with the flag when tuning is done.
#ifndef CAPTURE_TIMING
#define CAPTURE_TIMING 0
#endif
struct JpegCaptureTiming {
  uint32_t fbMs, decodeMs, statsMs, lookMs, sharpenMs, encodeMs, closeMs,
      totalMs;
};
static JpegCaptureTiming gJpegCaptureTiming;

// Re-encode quality, fmt2jpg()/jpge scale (higher = better). NOT comparable
// with the camera's config.jpeg_quality: that scale is inverted, and handing
// 10 to the encoder puts ~5.5x-scaled standard tables plus 4:2:0 subsampling
// into every still -- the chroma solarization seen on dusk gradients. 80's
// luma table (DC 6) matches the q6 sensor's, so the re-encode no longer
// re-flattens the finer input; 60 was matched to the old q10 sensor and left
// visible tiles in smooth sky.
#ifndef JPEG_REENCODE_QUALITY
#define JPEG_REENCODE_QUALITY 80
#endif

// Output-size bound for the free-space check before a file is opened. NONE
// without a border is exact (pass-through). The re-encoding paths measured
// ~0.125 bytes/pixel worst case at encoder quality 10; the old 0.25 B/px
// budget carried a 2x margin, but finer tables grow the files ~2.5x, so above
// encoder-quality 10 the budget doubles. A border grows the canvas, so its
// bound uses the expanded dimensions.
#if JPEG_REENCODE_QUALITY > 10
#define JPEG_FILTER_WC_SHIFT 1  // re-encode budget: 0.5 bytes/pixel
#else
#define JPEG_FILTER_WC_SHIFT 2  // re-encode budget: 0.25 bytes/pixel
#endif
static inline size_t jpegFilterWorstCaseBytes(const camera_fb_t* fb) {
  if (gFilterLook == FILTER_LOOK_NONE && !gFilterBorder) return fb->len;
  const size_t w = fb->width;
  const size_t h = fb->height;
  if (gFilterBorder) {
    return ((size_t)(w + 2 * JPEG_FILTER_INSET_PX) *
            (size_t)(h + 2 * JPEG_FILTER_INSET_PX)) >>
           JPEG_FILTER_WC_SHIFT;
  }
  return (w * h) >> JPEG_FILTER_WC_SHIFT;
}

// The pass-through and unbordered paths keep the capture's dimensions. Every
// bordered variant needs the larger canvas: at SXGA with a 24 px frame that is
// 1328x1072 (2,847,232 bytes). The decoder writes the image into the leading
// 1280x1024 pixels, the look/sharpen passes run there, and only at the end are
// the rows shifted into the inset and the border filled. Peak PSRAM stays
// about 3.4 MB including two camera buffers.

// ESP32-S3 PIE 8-lane 16-bit dot product (ee.vmulas.s16.accx). The .S file is
// assembled only in the S3 build; the host test and other targets use the
// scalar fallback.
#ifndef SUPERIA_USM_SIMD
#define SUPERIA_USM_SIMD 1
#endif
#if SUPERIA_USM_SIMD && defined(CONFIG_IDF_TARGET_ESP32S3)
#define USM_SIMD_ENABLED 1
extern "C" void usmVblurS3(const int16_t* ring, uint8_t* out,
                           const int16_t* coeffs, int n);
#else
#define USM_SIMD_ENABLED 0
#endif

// ---- dual-core row splitting ----------------------------------------------
// The stats, look and sharpen passes are independent per row (sharpen needs a
// small copy of the boundary rows because it writes the canvas in place), so
// the second LX7 core runs one half-frame while the loop task runs the other.
// The first S3 port kept this serial because a second busy core browned out the
// board on USB power; the unit is now powered from an 18650 + 5 V boost, which
// has the headroom. -DJPEG_FILTER_DUALCORE=0 restores the single-core path for
// power experiments; the host test always takes it (no FreeRTOS there).
#ifndef JPEG_FILTER_DUALCORE
#define JPEG_FILTER_DUALCORE 1
#endif
#if JPEG_FILTER_DUALCORE != 0 && JPEG_FILTER_DUALCORE != 1
#error "JPEG_FILTER_DUALCORE must be 0 or 1"
#endif
#if JPEG_FILTER_DUALCORE && defined(CONFIG_IDF_TARGET_ESP32S3)
#define JPEG_FILTER_PARALLEL 1
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#else
#define JPEG_FILTER_PARALLEL 0
#endif

// Film-look + unsharp sharpening, shared scaffolding for the look filters:
//   Superia    "Fuji Superia 400 -> Sharpen"
//   Kodachrome "Kodak Kodachrome 64 -> Vibrance -> Sharpen"
//   T-Max      "Kodak T-Max 400 -> Sharpen" (panchromatic B&W)
//   Portra     "Kodak Portra 400 -> Sharpen (0.4, sigma 1)"
// All are ported from tools/preview_lab/app.html. Design: one decode, a single
// fused colour pass, one neighbourhood pass, one encode. Every look stage is
// elementwise, so running them as separate JPEG filters would only add
// decode/encode round trips: extra CPU per pass and one generation of JPEG
// quantisation loss each.
//
// Stage order follows filmLook() and the downstream nodes:
//   auto balance -> auto contrast -> linear white balance -> 3x3 matrix
//   -> saturation -> gamma + smoothstep contrast -> shadow/highlight tint
//   -> monochrome -> vignette -> grain -> vibrance -> gaussian unsharp sharpen
// Superia skips the matrix, saturation, vibrance and grain stages (its node
// defaults make them identity/off); Kodachrome adds a matrix, saturation,
// vibrance and grain (off by default); T-Max skips matrix, saturation,
// vibrance and tints, and enables the monochrome mix; Portra adds WB,
// saturation, tone, tints, vignette and grain (off by default), with no
// matrix, vibrance or monochrome stage. Values
// differ from the lab by construction because the canvas is RGB565 (5/6/5
// bits): expect mild extra banding on smooth gradients vs. the lab's 8-bit
// model. When grain is enabled, Kodachrome's/T-Max's/Portra's grain size of
// 1.2 is below the lab's blur threshold (sigma < 0.3), so only the amount is
// reproduced; the device grain is the same per-pixel noise, monochrome.

// Superia tunables, all -D overridable. Values = preview_lab node defaults.
#ifndef SUPERIA_AUTO_BALANCE
#define SUPERIA_AUTO_BALANCE 0.55f
#endif
#ifndef SUPERIA_AUTO_CONTRAST
#define SUPERIA_AUTO_CONTRAST 0.56f
#endif
#ifndef SUPERIA_WB_R
#define SUPERIA_WB_R 0.98f
#endif
#ifndef SUPERIA_WB_G
#define SUPERIA_WB_G 1.01f
#endif
#ifndef SUPERIA_WB_B
#define SUPERIA_WB_B 1.00f
#endif
#ifndef SUPERIA_MIDTONE_GAMMA
#define SUPERIA_MIDTONE_GAMMA 0.97f
#endif
#ifndef SUPERIA_SHADOW_R
#define SUPERIA_SHADOW_R (-0.010f)
#endif
#ifndef SUPERIA_SHADOW_G
#define SUPERIA_SHADOW_G 0.010f
#endif
#ifndef SUPERIA_SHADOW_B
#define SUPERIA_SHADOW_B 0.010f
#endif
#ifndef SUPERIA_HIGHLIGHT_R
#define SUPERIA_HIGHLIGHT_R 0.020f
#endif
#ifndef SUPERIA_HIGHLIGHT_G
#define SUPERIA_HIGHLIGHT_G 0.014f
#endif
#ifndef SUPERIA_HIGHLIGHT_B
#define SUPERIA_HIGHLIGHT_B (-0.006f)
#endif
#ifndef SUPERIA_VIGNETTE
#define SUPERIA_VIGNETTE 0.50f
#endif
#ifndef SUPERIA_USM_AMOUNT
#define SUPERIA_USM_AMOUNT 0.90f
#endif
#ifndef SUPERIA_USM_SIGMA
#define SUPERIA_USM_SIGMA 1.00f
#endif
// Kernel radius clamps here to bound the ring buffer and CPU cost; sigma
// beyond a radius of 16 sharpens nothing visibly anyway.
#ifndef SUPERIA_USM_MAX_RADIUS
#define SUPERIA_USM_MAX_RADIUS 8
#endif
#if SUPERIA_USM_MAX_RADIUS > 8
#error "SUPERIA_USM_MAX_RADIUS > 8: extend the radius switch in jpegFilterStream"
#endif
// The radius is derived per capture from the active look's usmSigma and
// clamped to SUPERIA_USM_MAX_RADIUS; sigma is the tuning knob.
// Ring width in pixels; SXGA is 1280. Frames wider than this (UXGA = 1600)
// fall back to a PSRAM ring at capture time.
#ifndef SUPERIA_USM_RING_W
#define SUPERIA_USM_RING_W 1280
#endif

// ---- per-filter look parameters -------------------------------------------
// The shared pipeline reads the active FilmLookParams. Kodachrome values are
// the preview_lab node defaults for the chain
// "Kodachrome 64 -> Vibrance (0.5) -> Sharpen (0.4, sigma 1)", with grain
// off by default.
// Auto balance is off: the camera AWB already leaves neutral scenes neutral,
// while the grey-world mean of a foliage-heavy frame lifts R/B and the warm
// matrix then turns the whites magenta. The other looks' WB/tints absorb that
// correction; Kodachrome's identity WB and red matrix do not.
#ifndef KODACHROME_AUTO_BALANCE
#define KODACHROME_AUTO_BALANCE 0.0f
#endif
#ifndef KODACHROME_AUTO_CONTRAST
#define KODACHROME_AUTO_CONTRAST 0.62f
#endif
#ifndef KODACHROME_SATURATION
#define KODACHROME_SATURATION 1.24f
#endif
#ifndef KODACHROME_MIDTONE_GAMMA
#define KODACHROME_MIDTONE_GAMMA 0.94f
#endif
#ifndef KODACHROME_TONE_CONTRAST
#define KODACHROME_TONE_CONTRAST 0.40f
#endif
#ifndef KODACHROME_MAT_RR
#define KODACHROME_MAT_RR 1.10f
#endif
#ifndef KODACHROME_MAT_RG
#define KODACHROME_MAT_RG (-0.04f)
#endif
#ifndef KODACHROME_MAT_RB
#define KODACHROME_MAT_RB (-0.02f)
#endif
#ifndef KODACHROME_MAT_GR
#define KODACHROME_MAT_GR (-0.03f)
#endif
#ifndef KODACHROME_MAT_GG
#define KODACHROME_MAT_GG 1.00f
#endif
#ifndef KODACHROME_MAT_GB
#define KODACHROME_MAT_GB 0.02f
#endif
#ifndef KODACHROME_MAT_BR
#define KODACHROME_MAT_BR (-0.02f)
#endif
#ifndef KODACHROME_MAT_BG
#define KODACHROME_MAT_BG 0.02f
#endif
#ifndef KODACHROME_MAT_BB
#define KODACHROME_MAT_BB 0.99f
#endif
#ifndef KODACHROME_SHADOW_R
#define KODACHROME_SHADOW_R (-0.004f)
#endif
#ifndef KODACHROME_SHADOW_G
#define KODACHROME_SHADOW_G 0.002f
#endif
#ifndef KODACHROME_SHADOW_B
#define KODACHROME_SHADOW_B 0.012f
#endif
#ifndef KODACHROME_HIGHLIGHT_R
#define KODACHROME_HIGHLIGHT_R 0.046f
#endif
#ifndef KODACHROME_HIGHLIGHT_G
#define KODACHROME_HIGHLIGHT_G 0.004f
#endif
#ifndef KODACHROME_HIGHLIGHT_B
#define KODACHROME_HIGHLIGHT_B (-0.024f)
#endif
#ifndef KODACHROME_VIGNETTE
#define KODACHROME_VIGNETTE 0.10f
#endif
#ifndef KODACHROME_GRAIN
#define KODACHROME_GRAIN 0.00f
#endif
#ifndef KODACHROME_VIBRANCE
#define KODACHROME_VIBRANCE 0.50f
#endif
#ifndef KODACHROME_USM_AMOUNT
#define KODACHROME_USM_AMOUNT 0.40f
#endif
#ifndef KODACHROME_USM_SIGMA
#define KODACHROME_USM_SIGMA 1.00f
#endif
// The S-curve, same two stages and same order as the T-Max look (and as the
// lab's tone()): a mid-grey slope that darkens shadows and lifts highlights,
// then a tanh shoulder that keeps the lifted highlights from clipping. The
// values mirror T-Max's 1.12/0.08 so the slide gains the same separation;
// Kodachrome's gentler smoothstep blend (0.60 vs 0.80) keeps the overall
// curve below T-Max's. Identity is 1.0/0.0, which reproduces the pre-S-curve
// look exactly.
#ifndef KODACHROME_TONE_GAIN
#define KODACHROME_TONE_GAIN 1.12f
#endif
#ifndef KODACHROME_HIGHLIGHT_ROLLOFF
#define KODACHROME_HIGHLIGHT_ROLLOFF 0.08f
#endif
static_assert(
    KODACHROME_HIGHLIGHT_ROLLOFF >= 0.0f && KODACHROME_HIGHLIGHT_ROLLOFF < 1.0f,
    "KODACHROME_HIGHLIGHT_ROLLOFF must be in [0, 1): 1 would flatten the "
    "whole tonal range into the knee");

// T-Max 400 values are the preview_lab node defaults for
// "T-Max 400 -> Sharpen (0.4, sigma 1)".
#ifndef TMAX_AUTO_BALANCE
#define TMAX_AUTO_BALANCE 0.70f
#endif
#ifndef TMAX_AUTO_CONTRAST
#define TMAX_AUTO_CONTRAST 0.55f
#endif
#ifndef TMAX_MIDTONE_GAMMA
#define TMAX_MIDTONE_GAMMA 0.98f
#endif
#ifndef TMAX_TONE_CONTRAST
#define TMAX_TONE_CONTRAST 0.80f
#endif
#ifndef TMAX_MONO_R
#define TMAX_MONO_R 0.30f
#endif
#ifndef TMAX_MONO_G
#define TMAX_MONO_G 0.59f
#endif
#ifndef TMAX_MONO_B
#define TMAX_MONO_B 0.11f
#endif
#ifndef TMAX_VIGNETTE
#define TMAX_VIGNETTE 0.30f
#endif
#ifndef TMAX_GRAIN
#define TMAX_GRAIN 0.030f
#endif
#ifndef TMAX_USM_AMOUNT
#define TMAX_USM_AMOUNT 0.40f
#endif
#ifndef TMAX_USM_SIGMA
#define TMAX_USM_SIGMA 1.00f
#endif
// The S-curve has two halves, both folded into the tonal LUT:
//
//   TMAX_TONE_GAIN       slope about mid-grey, applied after the smoothstep
//                        blend. >1 darkens the shadows and lifts the highlights
//                        symmetrically, which is what a genuine contrast
//                        increase does -- and, on its own, it clips.
//   TMAX_HIGHLIGHT_ROLLOFF  film's shoulder: everything above 1 - rolloff is
//                        compressed into a tanh knee that asymptotes below
//                        white instead of clipping. It is what keeps a lifted
//                        highlight from reading as blown, and it buys back the
//                        headroom the unsharp mask below would otherwise eat
//                        (USM runs after the tone stage and clamps at 255).
//
// Same two stages, same order, as tools/film_lab/film_emulation.py's tone()
// (contrast_gain then _highlight_rolloff) -- the lab is the design bench, so
// the firmware copies its curve rather than inventing one. The identity is 1.0
// for the gain and 0 for the rolloff: either one alone is a valid look, and
// both at identity reproduce the pre-S-curve T-Max exactly.
//
// 1.12 / 0.08 is measured, not guessed: against the pre-change binary on a
// linear ramp, the upper-mid tones rise 208 -> 218 and the top of the ramp
// rises 228 -> 238, while the last code stays at the old value (228) instead of
// following it up -- that held top is the shoulder. Shadows move the other way
// (pixels under 6/255 go 6.5% -> 14.6% of the ramp) and the distinct levels in
// the ramp go 36 -> 54, so the S adds separation rather than flattening it. A
// 0.16 knee put the top of the ramp back at 228, undoing the lift; 0.08 is the
// narrowest knee that still gives the unsharp mask headroom to clip into.
#ifndef TMAX_TONE_GAIN
#define TMAX_TONE_GAIN 1.12f
#endif
#ifndef TMAX_HIGHLIGHT_ROLLOFF
#define TMAX_HIGHLIGHT_ROLLOFF 0.08f
#endif
static_assert(TMAX_HIGHLIGHT_ROLLOFF >= 0.0f && TMAX_HIGHLIGHT_ROLLOFF < 1.0f,
              "TMAX_HIGHLIGHT_ROLLOFF must be in [0, 1): 1 would flatten the "
              "whole tonal range into the knee");

// Portra 400 values are the preview_lab node defaults for
// "Portra 400 -> Sharpen (0.4, sigma 1)", with grain off by default. The
// grade is soft, slightly
// desaturated and golden-yellow; the 0.03 shoulder keeps the highlights creamy.
#ifndef PORTRA_AUTO_BALANCE
#define PORTRA_AUTO_BALANCE 0.60f
#endif
#ifndef PORTRA_AUTO_CONTRAST
#define PORTRA_AUTO_CONTRAST 0.45f
#endif
#ifndef PORTRA_SATURATION
#define PORTRA_SATURATION 0.95f
#endif
#ifndef PORTRA_WB_R
#define PORTRA_WB_R 1.04f
#endif
#ifndef PORTRA_WB_G
#define PORTRA_WB_G 1.00f
#endif
#ifndef PORTRA_WB_B
#define PORTRA_WB_B 0.96f
#endif
#ifndef PORTRA_MIDTONE_GAMMA
#define PORTRA_MIDTONE_GAMMA 0.98f
#endif
#ifndef PORTRA_TONE_CONTRAST
#define PORTRA_TONE_CONTRAST 0.14f
#endif
#ifndef PORTRA_TONE_GAIN
#define PORTRA_TONE_GAIN 1.00f
#endif
#ifndef PORTRA_HIGHLIGHT_ROLLOFF
#define PORTRA_HIGHLIGHT_ROLLOFF 0.03f
#endif
#ifndef PORTRA_SHADOW_R
#define PORTRA_SHADOW_R 0.005f
#endif
#ifndef PORTRA_SHADOW_G
#define PORTRA_SHADOW_G 0.003f
#endif
#ifndef PORTRA_SHADOW_B
#define PORTRA_SHADOW_B 0.001f
#endif
#ifndef PORTRA_HIGHLIGHT_R
#define PORTRA_HIGHLIGHT_R 0.045f
#endif
#ifndef PORTRA_HIGHLIGHT_G
#define PORTRA_HIGHLIGHT_G 0.020f
#endif
#ifndef PORTRA_HIGHLIGHT_B
#define PORTRA_HIGHLIGHT_B (-0.020f)
#endif
#ifndef PORTRA_VIGNETTE
#define PORTRA_VIGNETTE 0.06f
#endif
#ifndef PORTRA_GRAIN
#define PORTRA_GRAIN 0.00f
#endif
#ifndef PORTRA_USM_AMOUNT
#define PORTRA_USM_AMOUNT 0.40f
#endif
#ifndef PORTRA_USM_SIGMA
#define PORTRA_USM_SIGMA 1.00f
#endif
static_assert(
    PORTRA_HIGHLIGHT_ROLLOFF >= 0.0f && PORTRA_HIGHLIGHT_ROLLOFF < 1.0f,
    "PORTRA_HIGHLIGHT_ROLLOFF must be in [0, 1): 1 would flatten the whole "
    "tonal range into the knee");

// Per-look runtime parameters. The five looks share one pipeline; selecting a
// look changes only this table entry. Every value stays -D-overridable through
// the SUPERIA_*, KODACHROME_*, TMAX_* and PORTRA_* macros above.
struct FilmLookParams {
  float autoBalance, autoContrast;
  float wbR, wbG, wbB;
  float midtoneGamma, toneContrast;
  float toneGain, highlightRolloff;
  float saturation;
  float shadowR, shadowG, shadowB;
  float highlightR, highlightG, highlightB;
  float vignette, grain, vibrance;
  float usmAmount, usmSigma;
  bool hasMatrix, gradedTints, mono;
  float matRR, matRG, matRB;
  float matGR, matGG, matGB;
  float matBR, matBG, matBB;
  float monoR, monoG, monoB;
};

// constexpr so the per-look stage set below is a true compile-time value:
// each look gets its own instantiation of the look pass and the per-pixel
// stage branches fold away (that is the ~0.3 s the runtime-generic loop cost).
static constexpr FilmLookParams FILM_LOOKS[FILTER_LOOK_COUNT] = {
  // NONE: the look pass is skipped, so these values are inert.
  { 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
    1.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, false, false, false,
    1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    1.0f, 1.0f, 1.0f },
  // Superia 400: no matrix, no saturation/vibrance/grain, no S-curve.
  { SUPERIA_AUTO_BALANCE, SUPERIA_AUTO_CONTRAST,
    SUPERIA_WB_R, SUPERIA_WB_G, SUPERIA_WB_B,
    SUPERIA_MIDTONE_GAMMA, 0.0f,
    1.0f, 0.0f,
    1.0f,
    SUPERIA_SHADOW_R, SUPERIA_SHADOW_G, SUPERIA_SHADOW_B,
    SUPERIA_HIGHLIGHT_R, SUPERIA_HIGHLIGHT_G, SUPERIA_HIGHLIGHT_B,
    SUPERIA_VIGNETTE, 0.0f, 0.0f,
    SUPERIA_USM_AMOUNT, SUPERIA_USM_SIGMA,
    false, false, false,
    1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    1.0f, 1.0f, 1.0f },
  // Kodachrome 64: warm matrix, vibrance, grain, graded tints, S-curve.
  { KODACHROME_AUTO_BALANCE, KODACHROME_AUTO_CONTRAST,
    1.0f, 1.0f, 1.0f,
    KODACHROME_MIDTONE_GAMMA, KODACHROME_TONE_CONTRAST,
    KODACHROME_TONE_GAIN, KODACHROME_HIGHLIGHT_ROLLOFF,
    KODACHROME_SATURATION,
    KODACHROME_SHADOW_R, KODACHROME_SHADOW_G, KODACHROME_SHADOW_B,
    KODACHROME_HIGHLIGHT_R, KODACHROME_HIGHLIGHT_G, KODACHROME_HIGHLIGHT_B,
    KODACHROME_VIGNETTE, KODACHROME_GRAIN, KODACHROME_VIBRANCE,
    KODACHROME_USM_AMOUNT, KODACHROME_USM_SIGMA,
    true, true, false,
    KODACHROME_MAT_RR, KODACHROME_MAT_RG, KODACHROME_MAT_RB,
    KODACHROME_MAT_GR, KODACHROME_MAT_GG, KODACHROME_MAT_GB,
    KODACHROME_MAT_BR, KODACHROME_MAT_BG, KODACHROME_MAT_BB,
    1.0f, 1.0f, 1.0f },
  // T-Max 400: panchromatic monochrome, grain, S-curve with a film shoulder,
  // no matrix/saturation/vibrance.
  { TMAX_AUTO_BALANCE, TMAX_AUTO_CONTRAST,
    1.0f, 1.0f, 1.0f,
    TMAX_MIDTONE_GAMMA, TMAX_TONE_CONTRAST,
    TMAX_TONE_GAIN, TMAX_HIGHLIGHT_ROLLOFF,
    1.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    TMAX_VIGNETTE, TMAX_GRAIN, 0.0f,
    TMAX_USM_AMOUNT, TMAX_USM_SIGMA,
    false, false, true,
    1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    TMAX_MONO_R, TMAX_MONO_G, TMAX_MONO_B },
  // Portra 400: soft colour, golden highlights, tints read after the tone
  // stage, no matrix/vibrance/monochrome.
  { PORTRA_AUTO_BALANCE, PORTRA_AUTO_CONTRAST,
    PORTRA_WB_R, PORTRA_WB_G, PORTRA_WB_B,
    PORTRA_MIDTONE_GAMMA, PORTRA_TONE_CONTRAST,
    PORTRA_TONE_GAIN, PORTRA_HIGHLIGHT_ROLLOFF,
    PORTRA_SATURATION,
    PORTRA_SHADOW_R, PORTRA_SHADOW_G, PORTRA_SHADOW_B,
    PORTRA_HIGHLIGHT_R, PORTRA_HIGHLIGHT_G, PORTRA_HIGHLIGHT_B,
    PORTRA_VIGNETTE, PORTRA_GRAIN, 0.0f,
    PORTRA_USM_AMOUNT, PORTRA_USM_SIGMA,
    false, true, false,
    1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    1.0f, 1.0f, 1.0f },
};

// Which stages a look actually runs, derived from the table itself so the
// dispatch below and the parameters can never disagree. This replaces the
// per-pixel `if (c.do...)` flags the runtime-generic loop paid for: each look
// instantiates its own look pass and the dead branches vanish at compile time.
enum LookStage : int {
  LOOK_NONE          = 0,
  LOOK_WB            = 1 << 0,
  LOOK_TONE          = 1 << 1,
  LOOK_VIGN          = 1 << 2,
  LOOK_MATRIX        = 1 << 3,
  LOOK_SAT           = 1 << 4,
  LOOK_VIBRANCE      = 1 << 5,
  LOOK_GRAIN         = 1 << 6,
  LOOK_TINTS         = 1 << 7,
  LOOK_GRADED_TINTS  = 1 << 8,
  LOOK_MONO          = 1 << 9,
};

static constexpr int lookStages(const FilmLookParams& p) {
  return ((p.wbR != 1.0f || p.wbG != 1.0f || p.wbB != 1.0f) ? LOOK_WB : 0) |
         ((p.midtoneGamma != 1.0f || p.toneContrast != 0.0f ||
           p.toneGain != 1.0f || p.highlightRolloff > 0.0f) ? LOOK_TONE : 0) |
         ((p.vignette > 0.0f) ? LOOK_VIGN : 0) |
         (p.hasMatrix ? LOOK_MATRIX : 0) |
         ((p.saturation != 1.0f) ? LOOK_SAT : 0) |
         ((p.vibrance > 0.0f) ? LOOK_VIBRANCE : 0) |
         ((p.grain > 0.0f) ? LOOK_GRAIN : 0) |
         ((p.shadowR != 0.0f || p.shadowG != 0.0f || p.shadowB != 0.0f ||
           p.highlightR != 0.0f || p.highlightG != 0.0f ||
           p.highlightB != 0.0f) ? LOOK_TINTS : 0) |
         (p.gradedTints ? LOOK_GRADED_TINTS : 0) |
         (p.mono ? LOOK_MONO : 0);
}

static constexpr int LOOK_STAGES[FILTER_LOOK_COUNT] = {
  lookStages(FILM_LOOKS[FILTER_LOOK_NONE]),
  lookStages(FILM_LOOKS[FILTER_LOOK_SUPERIA]),
  lookStages(FILM_LOOKS[FILTER_LOOK_KODACHROME]),
  lookStages(FILM_LOOKS[FILTER_LOOK_TMAX]),
  lookStages(FILM_LOOKS[FILTER_LOOK_PORTRA]),
};

// The macros below are the hot-loop names the existing pipeline already uses;
// they now resolve to the active look, so selecting a look costs one pointer
// assignment rather than a second copy of the pipeline.
static const FilmLookParams* gFilm = &FILM_LOOKS[FILTER_DEFAULT_LOOK];
#define FILM_AUTO_BALANCE   (gFilm->autoBalance)
#define FILM_AUTO_CONTRAST  (gFilm->autoContrast)
#define FILM_WB_R           (gFilm->wbR)
#define FILM_WB_G           (gFilm->wbG)
#define FILM_WB_B           (gFilm->wbB)
#define FILM_MIDTONE_GAMMA  (gFilm->midtoneGamma)
#define FILM_TONE_CONTRAST  (gFilm->toneContrast)
#define FILM_TONE_GAIN      (gFilm->toneGain)
#define FILM_TONE_ROLLOFF   (gFilm->highlightRolloff)
#define FILM_SATURATION     (gFilm->saturation)
#define FILM_SHADOW_R       (gFilm->shadowR)
#define FILM_SHADOW_G       (gFilm->shadowG)
#define FILM_SHADOW_B       (gFilm->shadowB)
#define FILM_HIGHLIGHT_R    (gFilm->highlightR)
#define FILM_HIGHLIGHT_G    (gFilm->highlightG)
#define FILM_HIGHLIGHT_B    (gFilm->highlightB)
#define FILM_VIGNETTE       (gFilm->vignette)
#define FILM_GRAIN          (gFilm->grain)
#define FILM_VIBRANCE       (gFilm->vibrance)
#define FILM_USM_AMOUNT     (gFilm->usmAmount)
#define FILM_USM_SIGMA      (gFilm->usmSigma)
#define FILM_USM_RAD        ((int)(3.0f * gFilm->usmSigma + 0.999f))
#define FILM_MONO_R         (gFilm->monoR)
#define FILM_MONO_G         (gFilm->monoG)
#define FILM_MONO_B         (gFilm->monoB)
#define FILM_MAT_RR         (gFilm->matRR)
#define FILM_MAT_RG         (gFilm->matRG)
#define FILM_MAT_RB         (gFilm->matRB)
#define FILM_MAT_GR         (gFilm->matGR)
#define FILM_MAT_GG         (gFilm->matGG)
#define FILM_MAT_GB         (gFilm->matGB)
#define FILM_MAT_BR         (gFilm->matBR)
#define FILM_MAT_BG         (gFilm->matBG)
#define FILM_MAT_BB         (gFilm->matBB)
#define FILM_BORDER         (gFilterBorder)

// Look selection has to keep gFilm in step with gFilterLook. These live after
// the table so the pointer can be refreshed; the rest of the runtime API is at
// the top of the file.
static inline void jpegFilterSetLook(FilterLook look) {
  if ((unsigned)look < (unsigned)FILTER_LOOK_COUNT) {
    gFilterLook = look;
    gFilm = &FILM_LOOKS[look];
  }
}

// Button cycle. Superia stays implemented and keeps its enum slot so stored
// look indices do not shift, but it is no longer offered at runtime.
static constexpr FilterLook FILTER_CYCLE[] = {
    FILTER_LOOK_NONE, FILTER_LOOK_KODACHROME, FILTER_LOOK_TMAX,
    FILTER_LOOK_PORTRA,
};
static constexpr uint8_t FILTER_CYCLE_COUNT =
    (uint8_t)(sizeof(FILTER_CYCLE) / sizeof(FILTER_CYCLE[0]));

static inline FilterLook jpegFilterNextLook(void) {
  for (uint8_t i = 0; i < FILTER_CYCLE_COUNT; i++) {
    if (FILTER_CYCLE[i] == gFilterLook) {
      gFilterLook = FILTER_CYCLE[(i + 1) % FILTER_CYCLE_COUNT];
      gFilm = &FILM_LOOKS[gFilterLook];
      return gFilterLook;
    }
  }
  // A retired look (e.g. an old NVS value) restarts at the first option.
  gFilterLook = FILTER_CYCLE[0];
  gFilm = &FILM_LOOKS[gFilterLook];
  return gFilterLook;
}

// ---- lab math helpers ------------------------------------------------------

// always_inline: the LX6 has no float divide or square root, so every `/` on a
// float -- even `/ 255.0f` -- is an out-of-line __divsf3 call costing ~210
// cycles. GCC declines to inline these helpers at -Os, which turned the look
// pass into ~7 M calls per shot. Every one of them is now a multiply or a
// table lookup; the attributes keep the small ones inside the loops.
#define USM_ALWAYS_INLINE __attribute__((always_inline)) inline

static USM_ALWAYS_INLINE float usmClampF(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

static USM_ALWAYS_INLINE uint8_t usmToU8(float v) {
  return (uint8_t)(usmClampF(v, 0.0f, 255.0f) + 0.5f);
}

// 1/255 as a literal multiply. Written as a division anywhere in a hot loop it
// becomes __divsf3; the two differ by at most 1 ulp, far below the 5-bit canvas.
#define USM_INV255 (1.0f / 255.0f)

// RGB565 <-> 8-bit channels (bit replication). The linked esp32-camera JPEG
// decoder stores the canvas as native little-endian standard R5G6B5 (verified
// in the linked lib: its RGB565 writer stores out[0] = v & 0xFF, out[1] =
// v >> 8), so a plain u16 read already holds the true channels. The byte swap
// the canvas needs lives at the encode boundary (see jpegFilterStream).
static USM_ALWAYS_INLINE uint16_t usmPack565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((uint16_t)(r >> 3) << 11)
                  | ((uint16_t)(g >> 2) << 5)
                  | (uint16_t)(b >> 3));
}

static USM_ALWAYS_INLINE void usmUnpack565(uint16_t v, uint8_t rgb[3]) {
  const uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
  const uint8_t g6 = (uint8_t)((v >> 5) & 0x3F);
  const uint8_t b5 = (uint8_t)(v & 0x1F);
  rgb[0] = (uint8_t)((r5 << 3) | (r5 >> 2));  // 5 -> 8 bit replication
  rgb[1] = (uint8_t)((g6 << 2) | (g6 >> 4));  // 6 -> 8 bit replication
  rgb[2] = (uint8_t)((b5 << 3) | (b5 >> 2));
}

// The lab evaluates powf() per pixel (midtone gamma, vignette) and a full
// sRGB<->linear round trip per channel for the white balance. Per-pixel
// transcendentals are the slowest op on this MCU, so every curve becomes a
// table. The first version used 4096-entry float tables in PSRAM (64 KB
// total): every lookup was a random access that missed the 32 KB data cache,
// and pass 3 alone accounted for most of the shot. The canvas is 5/6/5, so
// 8-bit inputs are enough; the tables now live in internal DRAM and fit the
// cache. All lookups are one indexed load, no interpolation.
#define SUPERIA_LUT_N 256
static float   usmLutToLin[SUPERIA_LUT_N];  // sRGB -> linear, built once
static uint8_t usmLutWb[3][SUPERIA_LUT_N];  // sRGB -> gain -> sRGB, once
static float   usmLutVign[SUPERIA_LUT_N];   // powf(d, 1.2), once
// The midtone gamma scales all three channels by lutGamma[L]/L. That division
// is a __divsf3 call per pixel, so the quotient is tabulated next to the curve.
// The film "contrast" smoothstep, the mid-grey slope and the highlight shoulder
// are folded into the same curve. (The curve's own value is not tabulated: the
// only consumer is this quotient.)
static float   usmLutGammaScale[SUPERIA_LUT_N];
// Auto contrast's scale is (L + (s-L)*ac)/L, and `s` depends on this frame's
// percentiles, so it is rebuilt per shot -- 512 divisions instead of 1.31 M.
// Bins cover L in [0, 2]: the balance gain can push luma past 1, and clamping
// that back to 1 would visibly lift highlights.
// The stretch's local slope is (1-ac) + ac*(0.945/(hi-lo)): on a flat frame
// (small hi-lo) that exceeds 1 and amplifies the source JPEG's coarse chroma
// blocks and the 5/6-bit canvas staircase into visible block-aligned banding.
// The cap limits that slope; normal scenes (wide histogram) never reach it.
// Mirrored in tools/film_lab and tools/preview_lab.
#ifndef SUPERIA_CONTRAST_SLOPE_CAP
#define SUPERIA_CONTRAST_SLOPE_CAP 1.5f
#endif
#define SUPERIA_CSCALE_N 512
static float   usmLutContrast[SUPERIA_CSCALE_N];
// Vibrance needs 1/max(r,g,b); 256 divisions once instead of one per pixel.
static float   usmLutInvMax[SUPERIA_LUT_N];

static float filmWbGain[3];

static inline float usmLinearToSrgb(float t) {
  return t <= 0.0031308f ? t * 12.92f
                         : 1.055f * powf(t, 1.0f / 2.4f) - 0.055f;
}

// One table covers the whole toLin -> fixed WB gain -> toSrgb round trip. The
// auto-balance gain is applied to the sRGB value before this lookup (as in
// the lab), so it does not enter this table and the table is built once.
static void usmBuildWbLut(void) {
  for (int c = 0; c < 3; c++)
    for (int i = 0; i < SUPERIA_LUT_N; i++)
      usmLutWb[c][i] =
          usmToU8(255.0f * usmLinearToSrgb(usmLutToLin[i] * filmWbGain[c]));
}

static void usmInitLuts(void) {
  // Rebuilt per capture because the active look can change between shots.
  // 4x256 table entries is milliseconds against a ~7 s capture, and it keeps
  // the LUTs from ever describing a look that is no longer selected.
  filmWbGain[0] = FILM_WB_R;
  filmWbGain[1] = FILM_WB_G;
  filmWbGain[2] = FILM_WB_B;
  for (int i = 0; i < SUPERIA_LUT_N; i++) {
    const float t = i / (float)(SUPERIA_LUT_N - 1);
    usmLutToLin[i] = t <= 0.04045f ? t / 12.92f
                                   : powf((t + 0.055f) / 1.055f, 2.4f);
    usmLutVign[i] = powf(t, 1.2f);
    usmLutInvMax[i] = i > 0 ? 255.0f / (float)i : 0.0f;
  }
  // Identity gains (Kodachrome/T-Max) never read this table -- doWb is false
  // for them -- so skip ~768 powf round trips that decode timing would
  // otherwise pay on every shot.
  if (FILM_WB_R != 1.0f || FILM_WB_G != 1.0f || FILM_WB_B != 1.0f)
    usmBuildWbLut();
  // An identity/unwanted curve is skipped, but the decision is now runtime.
  // Every stage of the tonal curve has to be listed here AND in lookStages()
  // below: a stage missing from either one is silently dropped, not merely
  // mis-tuned.
  if (FILM_MIDTONE_GAMMA != 1.0f || FILM_TONE_CONTRAST != 0.0f ||
      FILM_TONE_GAIN != 1.0f || FILM_TONE_ROLLOFF > 0.0f)
    for (int i = 0; i < SUPERIA_LUT_N; i++) {
      const float t = i / (float)(SUPERIA_LUT_N - 1);
      float v = FILM_MIDTONE_GAMMA != 1.0f ? powf(t, FILM_MIDTONE_GAMMA) : t;
      if (FILM_TONE_CONTRAST != 0.0f)
        v = v + FILM_TONE_CONTRAST * (v * v * (3.0f - 2.0f * v) - v);
      // Slope about mid-grey: the S proper. Shadows go down, highlights go up,
      // and everything past 0/1 would clip -- which is why the shoulder below
      // is not optional when this is above 1.
      if (FILM_TONE_GAIN != 1.0f) v = 0.5f + (v - 0.5f) * FILM_TONE_GAIN;
      v = usmClampF(v, 0.0f, 1.0f);
      // Film's shoulder. Same expression as the lab's _highlight_rolloff():
      // a tanh knee of width `rolloff` starting at 1 - rolloff, so white
      // saturates towards 1 instead of being pinned there. Clipping happens
      // above *before* this, exactly as in the lab, so the knee never receives
      // a value over 1 and the plateau it leaves is bounded by the clip point.
      const float rolloff = usmClampF(FILM_TONE_ROLLOFF, 0.0f, 0.99f);
      if (rolloff > 0.0f) {
        const float knee = 1.0f - rolloff;
        if (v > knee) v = knee + rolloff * tanhf((v - knee) / rolloff);
      }
      usmLutGammaScale[i] = t > 1e-4f ? v / t : 0.0f;
    }
}

// 8-bit indexed lookup; inputs/outputs are quantised far below the 5-bit
// canvas, so interpolation would only add work.
static USM_ALWAYS_INLINE float usmLut8(const float* lut, float x) {
  const int i =
      (int)(usmClampF(x, 0.0f, 1.0f) * (float)(SUPERIA_LUT_N - 1) + 0.5f);
  return lut[i];
}

// Percentiles for the auto contrast come from a histogram, not the lab's full
// float sort: sorting 1.31M luma floats would need 5.25 MB of PSRAM that does
// not exist. Bins cover luma in [0, 1.5] (balance gains can push luma past 1),
// bounding the percentile error at ~0.05% of full scale.
#define SUPERIA_HIST_N 2048
#define SUPERIA_HIST_LMAX 1.5f
static uint32_t usmHist[SUPERIA_HIST_N];
#if JPEG_FILTER_PARALLEL
// The two cores fill separate bin tables so neither touches the other's bins;
// usmHistB is merged into usmHist before the percentile scan.
static uint32_t usmHistB[SUPERIA_HIST_N];
#endif

// Sharpen ring window, in internal DRAM. One 16-byte group per (channel, x)
// holding 8 int16 slots in ring-slot order; one aligned 128-bit load feeds
// the S3 dot-product kernel. At SXGA this is 61 KB (the old byte layout was
// 27 KB), still comfortably inside the S3's 512 KB SRAM. Wider kernels use
// 16/24-slot groups and the heap + scalar path.
#define USM_SLOTS_PER_GROUP 8
static int16_t usmRing[3u * SUPERIA_USM_RING_W * USM_SLOTS_PER_GROUP]
    __attribute__((aligned(16)));
static uint8_t usmRow8[3u * SUPERIA_USM_RING_W];
#if JPEG_FILTER_PARALLEL
// The second band needs its own ring and 8-bit row buffer. The halo holds the
// 2*rad canvas rows around the split: the canvas is sharpened in place, so
// both bands read those rows from this copy instead of racing the other band's
// writes. Sized for the default rad 3; wider kernels stay single-core.
#define USM_PARALLEL_MAX_RAD 3
static int16_t usmRing2[3u * SUPERIA_USM_RING_W * USM_SLOTS_PER_GROUP]
    __attribute__((aligned(16)));
static uint8_t usmRow8b[3u * SUPERIA_USM_RING_W];
static uint16_t usmHalo[2u * USM_PARALLEL_MAX_RAD * SUPERIA_USM_RING_W];
#endif

// Slots per 16-byte group for a given radius: 8 for rad <= 3, 16/24 above.
static inline int usmSlotsPerGroup(int rad) {
  return ((2 * rad + 1 + 7) / 8) * 8;
}

// ---- row-splitting worker --------------------------------------------------
// One persistent worker pinned to core 0 owns the first half of a stage; the
// loop task (core 1) runs the second half and joins on a semaphore. Rows are
// independent in the stats and look passes; the sharpen pass is also row-
// independent once the 2R boundary rows are copied (see usmSharpenT), because
// it rewrites the canvas it reads. The split changes no arithmetic: integer
// reductions are merged exactly and every pixel keeps its own x/y, including
// the grain hash and vignette.
typedef void (*ParallelBandFn)(int y0, int y1, int slot);

#if JPEG_FILTER_PARALLEL
static TaskHandle_t gParTask = nullptr;
static SemaphoreHandle_t gParJob = nullptr;
static SemaphoreHandle_t gParDone = nullptr;
static ParallelBandFn gParFn = nullptr;
static int gParSplit = 0;

static void parallelWorker(void*) {
  for (;;) {
    if (xSemaphoreTake(gParJob, portMAX_DELAY) != pdTRUE) continue;
    gParFn(0, gParSplit, 0);
    xSemaphoreGive(gParDone);
  }
}

// Lazily created on the first filtered capture: NONE never starts a task, and
// a failed allocation leaves the proven single-core path in place instead of
// failing the shot. `tried` is only touched by the loop task.
static bool parallelInit(void) {
  static bool tried = false;
  if (tried) return gParTask != nullptr;
  tried = true;
  gParJob = xSemaphoreCreateBinary();
  gParDone = xSemaphoreCreateBinary();
  if (gParJob && gParDone &&
      xTaskCreatePinnedToCore(parallelWorker, "jpeg_par", 4096, nullptr, 2,
                              &gParTask, 0) == pdPASS) {
    return true;
  }
  if (gParJob) { vSemaphoreDelete(gParJob); gParJob = nullptr; }
  if (gParDone) { vSemaphoreDelete(gParDone); gParDone = nullptr; }
  gParTask = nullptr;
  return false;
}
#endif

// Split `h` rows across both cores. With no worker (host build, task creation
// failure) the same band function runs the whole frame on slot 0, so callers
// have one code path and the single-core result stays identical.
static inline void parallelRun(ParallelBandFn fn, int h) {
#if JPEG_FILTER_PARALLEL
  if (h >= 2 && parallelInit()) {
    gParFn = fn;
    gParSplit = h / 2;
    xSemaphoreGive(gParJob);
    fn(gParSplit, h, 1);
    xSemaphoreTake(gParDone, portMAX_DELAY);
    return;
  }
#endif
  fn(0, h, 0);
}

// ---- look pass -------------------------------------------------------------
// Per-pixel, no neighbour reads. Kept as a row-range function so the second
// core can take a band: the stats, look and sharpen passes split the frame in
// half on the S3 (JPEG_FILTER_DUALCORE). The old USB-powered board browned out
// with a second busy core, the 18650 + 5 V boost is the intended supply.
//
// The look-constant stages are a template parameter (LOOK_STAGES, derived
// from the table at compile time) so each look gets its own instantiation and
// the per-pixel stage branches fold away -- the runtime-generic loop cost
// about 0.3 s per colour shot in branch checks and register pressure alone.
// The content-dependent stages (auto balance needs this frame's masked means,
// auto contrast its percentiles) stay runtime flags.
struct UsmLook {
  uint16_t* px;
  int w, h;
  float gainR, gainG, gainB;
  bool doBalance, doContrast;
  float cx, cy, invCxn, invCyn;
};

// Cheap deterministic per-pixel noise in [-1, 1]. One integer hash, no state.
// The lab's grain size 1.2 is below its own blur threshold (sigma < 0.3), so
// it is unblurred noise; at a 1% amount the distribution shape is invisible on
// a 5/6/5 canvas, so a uniform hash replaces the lab's Gaussian.
static USM_ALWAYS_INLINE float usmNoise(int x, int y) {
  uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + 0x9E3779B9u;
  h = (h ^ (h >> 13)) * 1274126177u;
  h ^= h >> 16;
  return (float)(h & 0xFFFFu) * (2.0f / 65535.0f) - 1.0f;
}

template <int STAGES>
static void usmLookRows(const UsmLook& c, int y0, int y1) {
  uint16_t* px = c.px;
  const int w = c.w;
  // The look table is const, so it lives in flash: reading FILM_* inside the
  // pixel loop became one pointer load per parameter per pixel when looks
  // turned runtime-selectable. Read them once per call instead so the loop
  // works from registers (or at worst a cached stack slot).
  const float sat = FILM_SATURATION;
  const float shR = FILM_SHADOW_R, shG = FILM_SHADOW_G, shB = FILM_SHADOW_B;
  const float hiR = FILM_HIGHLIGHT_R, hiG = FILM_HIGHLIGHT_G,
      hiB = FILM_HIGHLIGHT_B;
  const float mRR = FILM_MAT_RR, mRG = FILM_MAT_RG, mRB = FILM_MAT_RB;
  const float mGR = FILM_MAT_GR, mGG = FILM_MAT_GG, mGB = FILM_MAT_GB;
  const float mBR = FILM_MAT_BR, mBG = FILM_MAT_BG, mBB = FILM_MAT_BB;
  const float monoR = FILM_MONO_R, monoG = FILM_MONO_G, monoB = FILM_MONO_B;
  const float vign = FILM_VIGNETTE, grain = FILM_GRAIN, vib = FILM_VIBRANCE;
  for (int y = y0; y < y1; y++) {
    const float yy = ((float)y - c.cy) * c.invCyn;
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(px[(size_t)y * w + x], rgb);
      float f[3] = { rgb[0] * USM_INV255, rgb[1] * USM_INV255, rgb[2] * USM_INV255 };

      if (c.doBalance) {
        f[0] *= c.gainR; f[1] *= c.gainG; f[2] *= c.gainB;
      }
      float L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      if (c.doContrast) {
        int li = (int)(L * 255.0f + 0.5f);
        if (li < 0) li = 0;
        if (li >= SUPERIA_CSCALE_N) li = SUPERIA_CSCALE_N - 1;
        const float sc = usmLutContrast[li];
        f[0] *= sc; f[1] *= sc; f[2] *= sc;
      }
      if (STAGES & LOOK_WB) {
        for (int ch = 0; ch < 3; ch++)
          f[ch] = usmLutWb[ch][usmToU8(f[ch] * 255.0f)] * USM_INV255;
      }
      L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];

      if (STAGES & LOOK_MATRIX) {
        const float r = f[0], g = f[1], b = f[2];
        f[0] = mRR * r + mRG * g + mRB * b;
        f[1] = mGR * r + mGG * g + mGB * b;
        f[2] = mBR * r + mBG * g + mBB * b;
      }
      if (STAGES & LOOK_SAT) {
        // Saturation pivots on the post-matrix luma; the tone stage below then
        // needs the post-saturation luma, so both are recomputed here. Without
        // either stage the WB-step luma is still current.
        L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
        for (int ch = 0; ch < 3; ch++) f[ch] = L + (f[ch] - L) * sat;
        L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      } else if (STAGES & LOOK_MATRIX) {
        L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      }

      if (STAGES & LOOK_TONE) {
        const float sc = usmLutGammaScale[usmToU8(L * 255.0f)];
        f[0] *= sc; f[1] *= sc; f[2] *= sc;
      }
      if (STAGES & LOOK_GRADED_TINTS) {
        // The lab tints read the luma of the graded image, after gamma/contrast.
        L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      }
      if (STAGES & LOOK_TINTS) {
        const float Lc = usmClampF(L, 0.0f, 1.0f);
        const float shw = (1.0f - Lc) * (1.0f - Lc);
        const float hiw = Lc * Lc;
        f[0] += shR * shw + hiR * hiw;
        f[1] += shG * shw + hiG * hiw;
        f[2] += shB * shw + hiB * hiw;
      }
      if (STAGES & LOOK_MONO) {
        // Panchromatic B&W mix. Everything after the mix is channel-uniform,
        // so the tail runs on one scalar instead of three channels; vibrance
        // is exactly identity on grey (sat == 0, so the gain cancels around
        // the average) and is skipped outright.
        float g = monoR * f[0] + monoG * f[1] + monoB * f[2];
        if (STAGES & LOOK_VIGN) {
          const float xx = ((float)x - c.cx) * c.invCxn;
          const float d = (xx * xx + yy * yy) * 0.5f;
          g *= usmClampF(1.0f - vign * usmLut8(usmLutVign, d), 0.0f, 1.0f);
        }
        if (STAGES & LOOK_GRAIN) {
          // Same luma expression as the colour path (the weights do not sum to
          // exactly 1.0f in binary), so the output stays bit-identical.
          const float Lg = usmClampF(0.2126f * g + 0.7152f * g + 0.0722f * g,
                                     0.0f, 1.0f);
          const float wgt = 0.35f + 0.65f * (1.0f - fabsf(2.0f * Lg - 1.0f));
          g += usmNoise(x, y) * (1.514f * grain * wgt);
        }
        const uint8_t v = usmToU8(g * 255.0f);
        px[(size_t)y * w + x] = usmPack565(v, v, v);
        continue;
      }
      if (STAGES & LOOK_VIGN) {
        const float xx = ((float)x - c.cx) * c.invCxn;
        const float d = (xx * xx + yy * yy) * 0.5f;
        const float vs = usmClampF(1.0f - vign * usmLut8(usmLutVign, d), 0.0f, 1.0f);
        f[0] *= vs; f[1] *= vs; f[2] *= vs;
      }
      if (STAGES & LOOK_GRAIN) {
        const float Lg = usmClampF(0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2],
                                   0.0f, 1.0f);
        const float wgt = 0.35f + 0.65f * (1.0f - fabsf(2.0f * Lg - 1.0f));
        const float n = usmNoise(x, y) * (1.514f * grain * wgt);
        f[0] += n; f[1] += n; f[2] += n;
      }
      if (STAGES & LOOK_VIBRANCE) {
        float mx = f[0], mn = f[0];
        if (f[1] > mx) mx = f[1];
        if (f[2] > mx) mx = f[2];
        if (f[1] < mn) mn = f[1];
        if (f[2] < mn) mn = f[2];
        const float s = (mx - mn) * usmLutInvMax[usmToU8(mx * 255.0f)];
        const float gain = 1.0f + vib * (1.0f - s);
        const float avg = (f[0] + f[1] + f[2]) * (1.0f / 3.0f);
        for (int ch = 0; ch < 3; ch++) f[ch] = avg + (f[ch] - avg) * gain;
      }
      px[(size_t)y * w + x] = usmPack565(usmToU8(f[0] * 255.0f),
                                        usmToU8(f[1] * 255.0f),
                                        usmToU8(f[2] * 255.0f));
    }
  }
}

// Runtime dispatch to the compile-time look instantiation. The switch in
// jpegFilterStream() selects the function pointer once per shot, so the worker
// does not need the template or the FILM_* table at all.
typedef void (*LookBandFn)(const UsmLook&, int, int);
static LookBandFn gLookBandFn = nullptr;
static const UsmLook* gLookCtx = nullptr;

template <int STAGES>
static void usmLookBandT(const UsmLook& c, int y0, int y1) {
  usmLookRows<STAGES>(c, y0, y1);
}

static void usmLookBand(int y0, int y1, int slot) {
  (void)slot;
  if (gLookBandFn) gLookBandFn(*gLookCtx, y0, y1);
}


// Sharpen: planar rows, one channel at a time, and a ring of 16-byte column
// groups so the vertical pass can use the S3's 8-lane 16-bit dot product.
//
// Horizontal pass: 2R+1 taps over one planar channel, fully unrolled. Doing
// all three channels at once made GCC spill 21 live products, so each channel
// gets its own accumulator; the taps and coefficients stay in registers.
//
// The ring stores, for every (channel, x), eight int16 values in ring-slot
// order. One 16-byte group is one aligned 128-bit load, and the kernel is
// rotated into the same slot order per output row, so usmVblurS3() does a
// whole 7-tap column with one ee.vmulas.s16.accx instead of seven scalar
// MACs. Wider kernels use 16/24 slots and the scalar path.
template <int R>
static void usmHblurRowT(const uint8_t* __restrict src, int16_t* __restrict dst,
                         const int* __restrict k, int w, int stride, int chans) {
  for (int c = 0; c < chans; c++) {
    const uint8_t* s = src + c * w;
    int16_t* d = dst + (size_t)c * w * stride;

    // Interior columns: no edge clamp, so the taps are one straight MAC chain.
    int16_t* p = d + (size_t)R * stride;
    for (int x = R; x < w - R; x++, p += stride) {
      int acc = 0;
#pragma GCC unroll 17
      for (int t = -R; t <= R; t++) acc += k[t + R] * (int)s[x + t];
      *p = (int16_t)((acc + 128) >> 8);  // Q8 -> 8-bit, rounded
    }

    // Left and right borders: replicate the edge pixel (2*R columns per row).
    for (int x = 0; x < R; x++) {
      int acc = 0;
      for (int t = -R; t <= R; t++) {
        int xi = x + t;
        xi = xi < 0 ? 0 : (xi > w - 1 ? w - 1 : xi);
        acc += k[t + R] * (int)s[xi];
      }
      d[(size_t)x * stride] = (int16_t)((acc + 128) >> 8);
    }
    for (int x = w - R; x < w; x++) {
      int acc = 0;
      for (int t = -R; t <= R; t++) {
        int xi = x + t;
        xi = xi < 0 ? 0 : (xi > w - 1 ? w - 1 : xi);
        acc += k[t + R] * (int)s[xi];
      }
      d[(size_t)x * stride] = (int16_t)((acc + 128) >> 8);
    }
  }
}

// Scalar vertical blur (host test, rad > 3, heap ring). `coeff` is the kernel
// rotated into ring-slot order, so the group is read in slot order regardless
// of where the ring window wraps.
template <int R>
static void usmVblurRowT(const int16_t* __restrict ring,
                         uint8_t* __restrict out,
                         const int16_t* __restrict coeff, int w, int stride) {
  for (int x = 0; x < w; x++) {
    const int16_t* p = ring + (size_t)x * stride;
    int acc = 0;
#pragma GCC unroll 17
    for (int d = 0; d <= 2 * R; d++) acc += (int)p[d] * (int)coeff[d];
    out[x] = (uint8_t)((acc + 128) >> 8);  // Q8 -> 8-bit, rounded
  }
}

#if CAPTURE_TIMING && USM_SIMD_ENABLED
// One-shot boot check that the PIE dot product matches the scalar path.
static bool usmSimdSelfTest() {
  static int16_t ring[64 * 8] __attribute__((aligned(16)));
  static int16_t coeff[8] __attribute__((aligned(16)));
  uint8_t out[64];
  const int16_t kk[8] = { 3, 21, 60, 88, 60, 21, 3, 0 };  // sum 256
  for (int i = 0; i < 8; i++) coeff[i] = kk[i];
  for (int i = 0; i < 64 * 8; i++)
    ring[i] = (int16_t)((i * 37 + 11) & 0xFF);
  usmVblurS3(ring, out, coeff, 64);
  for (int x = 0; x < 64; x++) {
    int acc = 0;
    for (int s = 0; s < 8; s++) acc += (int)ring[x * 8 + s] * (int)coeff[s];
    if (out[x] != (uint8_t)((acc + 128) >> 8)) return false;
  }
  return true;
}
#endif

// One sharpen band. `y0..y1` is the output row range, `ring`/`row8` are this
// band's working set, and `halo` (parallel only) holds the 2R source rows
// around the split so neither band reads canvas rows the other is rewriting.
template <int R>
static void usmSharpenBandT(uint16_t* px, int w, int h, int16_t* ring,
                            uint8_t* row8, const uint16_t* halo, int haloY0,
                            int haloY1, const int* k, int amountQ8, int stride,
                            bool allowSimd, bool mono, int y0, int y1) {
  const int ringRows = 2 * R + 1;
  // A mono look packs one value into all three channels, and R and B share the
  // same 5-bit grid, so their planes -- and therefore their unsharp results --
  // are identical: two blurs cover the frame instead of three.
  const int chans = mono ? 2 : 3;

  // Horizontal blur of canvas row ysrc into its ring slot (ysrc outside the
  // frame replicates row 0/h-1). `ring + slot` is the group's slot 0.
  auto hblurSlot = [&](int ysrc) {
    const int yc = ysrc < 0 ? 0 : (ysrc > h - 1 ? h - 1 : ysrc);
    const uint16_t* src =
        (halo && yc >= haloY0 && yc < haloY1)
            ? halo + (size_t)(yc - haloY0) * w
            : px + (size_t)yc * w;
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(src[x], rgb);
      row8[x] = rgb[0];
      row8[w + x] = rgb[1];
      if (!mono) row8[2 * w + x] = rgb[2];
    }
    const int slot = (ysrc % ringRows + ringRows) % ringRows;
    usmHblurRowT<R>(row8, ring + slot, k, w, stride, chans);
  };

  for (int y = y0 - R; y <= y0 + R; y++) hblurSlot(y);  // prime the ring

  int16_t coeff[24] __attribute__((aligned(16)));

  for (int y = y0; y < y1; y++) {
    uint16_t* prow = px + (size_t)y * w;

    // Rotate the kernel into ring-slot order for this output row: slot s
    // holds the row whose ring slot is s, so tap y+d lands in (y+d)%ringRows.
    memset(coeff, 0, sizeof(coeff));
    for (int d = -R; d <= R; d++) {
      const int slot = (y + d + ringRows) % ringRows;
      coeff[slot] = (int16_t)k[d + R];
    }

    // Vertical blur, one channel at a time, into row8 (free again: the
    // horizontal pass consumed the row before this output row started).
#if !USM_SIMD_ENABLED
    (void)allowSimd;
#endif
    for (int c = 0; c < chans; c++) {
      int16_t* grp = ring + (size_t)c * w * stride;
#if USM_SIMD_ENABLED
      if (allowSimd && stride == USM_SLOTS_PER_GROUP)
        usmVblurS3(grp, row8 + c * w, coeff, w);
      else
#endif
        usmVblurRowT<R>(grp, row8 + c * w, coeff, w, stride);
    }

    // Unsharp mask against the blurred planes, then pack back to RGB565.
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(prow[x], rgb);
      if (mono) {
        const int b0 = row8[x], b1 = row8[w + x];
        int v0 = (int)rgb[0] + ((((int)rgb[0] - b0) * amountQ8 + 128) >> 8);
        int v1 = (int)rgb[1] + ((((int)rgb[1] - b1) * amountQ8 + 128) >> 8);
        const uint8_t o0 = (uint8_t)(v0 < 0 ? 0 : (v0 > 255 ? 255 : v0));
        const uint8_t o1 = (uint8_t)(v1 < 0 ? 0 : (v1 > 255 ? 255 : v1));
        prow[x] = usmPack565(o0, o1, o0);
        continue;
      }
      uint8_t out[3];
      for (int c = 0; c < 3; c++) {
        const int blurred = row8[c * w + x];
        int v = (int)rgb[c] + ((((int)rgb[c] - blurred) * amountQ8 + 128) >> 8);
        out[c] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      }
      prow[x] = usmPack565(out[0], out[1], out[2]);
    }
    // Slide the window for the next output row. Skipping the last row avoids
    // reading a source row outside this band's halo after the other band may
    // have rewritten it.
    if (y + 1 < y1) hblurSlot(y + R + 1);
  }
}

#if JPEG_FILTER_PARALLEL
// Sharpen state for the second core. Only the default rad <= 3 uses this
// path; the band split and the halo layout follow the same h/2 split as
// parallelRun(). Slot 0 is the worker (the existing ring), slot 1 the loop
// task (the second static ring).
static uint16_t* gSharpPx = nullptr;
static int gSharpW = 0, gSharpH = 0;
static int16_t* gSharpRing[2] = {nullptr, nullptr};
static uint8_t* gSharpRow8[2] = {nullptr, nullptr};
static const uint16_t* gSharpHalo = nullptr;
static int gSharpHaloY0 = 0, gSharpHaloY1 = 0;
static const int* gSharpK = nullptr;
static int gSharpAmountQ8 = 0, gSharpStride = 0;
static bool gSharpSimd = false, gSharpMono = false;
static void (*gSharpThunk)(int slot, int y0, int y1) = nullptr;

static void usmSharpenBand(int y0, int y1, int slot) {
  gSharpThunk(slot, y0, y1);
}

template <int R>
static void usmSharpenThunkT(int slot, int y0, int y1) {
  usmSharpenBandT<R>(gSharpPx, gSharpW, gSharpH, gSharpRing[slot],
                     gSharpRow8[slot], gSharpHalo, gSharpHaloY0,
                     gSharpHaloY1, gSharpK, gSharpAmountQ8, gSharpStride,
                     gSharpSimd, gSharpMono, y0, y1);
}
#endif

template <int R>
static void usmSharpenT(uint16_t* px, int w, int h, int16_t* ring,
                        uint8_t* row8, const int* k, int amountQ8,
                        int stride, bool allowSimd, bool mono) {
#if JPEG_FILTER_PARALLEL
  // `allowSimd` is true only when the ring is the static one (w <= RING_W and
  // rad <= 3); that is also exactly when the second static ring exists.
  if (allowSimd && R <= USM_PARALLEL_MAX_RAD && h >= 2 * R) {
    const int haloY0 = h / 2 - R;
    const int haloRows = 2 * R;
    memcpy(usmHalo, px + (size_t)haloY0 * w,
           (size_t)haloRows * w * sizeof(uint16_t));

    gSharpPx = px;
    gSharpW = w;
    gSharpH = h;
    gSharpRing[0] = ring;
    gSharpRow8[0] = row8;
    gSharpRing[1] = usmRing2;
    gSharpRow8[1] = usmRow8b;
    gSharpHalo = usmHalo;
    gSharpHaloY0 = haloY0;
    gSharpHaloY1 = haloY0 + haloRows;
    gSharpK = k;
    gSharpAmountQ8 = amountQ8;
    gSharpStride = stride;
    gSharpSimd = allowSimd;
    gSharpMono = mono;
    gSharpThunk = &usmSharpenThunkT<R>;
    parallelRun(usmSharpenBand, h);
    return;
  }
#endif
  usmSharpenBandT<R>(px, w, h, ring, row8, nullptr, 0, 0, k, amountQ8,
                     stride, allowSimd, mono, 0, h);
}

// ---- statistics bands ------------------------------------------------------
// Canvas and per-shot constants are set by the loop task before parallelRun()
// and read-only while the worker runs; the semaphore hand-off orders them.
static const uint16_t* gBandPx = nullptr;
static int gBandW = 0;
static float gBandGainR = 1.0f, gBandGainG = 1.0f, gBandGainB = 1.0f;
static float gBandHistPerL = 0.0f;

struct UsmStatsSums {
  uint32_t sumR, sumG, sumB, maskedN;
};
static UsmStatsSums usmSums[2];

// Pass 1: masked colour sums for the auto balance. The lab restricts the mean
// to midtone pixels (0.10 < L < 0.90) so shadows/highlights do not skew the
// channel gains. Integer sums merge exactly in either order.
static void usmBalanceBand(int y0, int y1, int slot) {
  const uint16_t* px = gBandPx;
  const int w = gBandW;
  UsmStatsSums s = {0, 0, 0, 0};
  for (int y = y0; y < y1; y++) {
    const uint16_t* row = px + (size_t)y * w;
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(row[x], rgb);
      const float L =
          ((0.2126f * rgb[0]) + (0.7152f * rgb[1]) + (0.0722f * rgb[2])) *
          USM_INV255;
      if (L > 0.10f && L < 0.90f) {
        s.sumR += rgb[0];
        s.sumG += rgb[1];
        s.sumB += rgb[2];
        s.maskedN++;
      }
    }
  }
  usmSums[slot] = s;
}

// Pass 2: histogram of the post-balance, pre-WB luma (the lab computes its
// contrast percentiles at exactly this stage of its chain). The two cores fill
// separate tables so neither touches the same bins; they are merged before the
// percentile scan. On the single-core path only usmHist is used.
static void usmHistBand(int y0, int y1, int slot) {
#if JPEG_FILTER_PARALLEL
  uint32_t* hist = slot ? usmHistB : usmHist;
#else
  (void)slot;
  uint32_t* hist = usmHist;
#endif
  const uint16_t* px = gBandPx;
  const int w = gBandW;
  const float histPerL = gBandHistPerL;
  const float gainR = gBandGainR, gainG = gBandGainG, gainB = gBandGainB;
  for (int y = y0; y < y1; y++) {
    const uint16_t* row = px + (size_t)y * w;
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(row[x], rgb);
      const float L = ((0.2126f * rgb[0] * gainR) +
                       (0.7152f * rgb[1] * gainG) +
                       (0.0722f * rgb[2] * gainB)) *
                      USM_INV255;
      int bin = (int)(L * histPerL);
      if (bin < 0) bin = 0;
      if (bin > SUPERIA_HIST_N - 1) bin = SUPERIA_HIST_N - 1;
      hist[bin]++;
    }
  }
}

// ---- entry point -----------------------------------------------------------

// Adapter from fmt2jpg_cb's callback into the caller's sink (as in the border
// filter; only one filter branch is compiled at a time).
struct FilmJpgOutCtx {
  JpegWriteFn write;
  void* arg;
  size_t written;
  bool failed;
};

static size_t filmJpgOutCb(void* arg, size_t index, const void* data, size_t len) {
  FilmJpgOutCtx* c = (FilmJpgOutCtx*)arg;
  (void)index;
  const size_t n = c->write(c->arg, (const uint8_t*)data, len);
  c->written += n;
  if (n != len) c->failed = true;
  return n;
}

static inline bool jpegFilterStream(camera_fb_t* fb, uint8_t quality,
                                    JpegWriteFn write, void* arg,
                                    size_t* written, uint16_t* outW,
                                    uint16_t* outH) {
  *written = 0;
  *outW = 0;
  *outH = 0;

  // NONE without a border is the only zero-cost path: the camera's JPEG is
  // written verbatim, with no decode or re-encode.
  if (gFilterLook == FILTER_LOOK_NONE && !gFilterBorder) {
    *outW = fb->width;
    *outH = fb->height;
    const size_t n = write(arg, fb->buf, fb->len);
    *written = n;
    return n == fb->len;
  }

  const uint16_t w = fb->width;
  const uint16_t h = fb->height;
  const size_t n = (size_t)w * (size_t)h;
  if (n == 0) return false;

  // Bordered variants keep the image at native size and grow the frame by this
  // much on every side. The image is decoded/processed in the leading w*h
  // pixels, then shifted into the inset just before encoding.
  const int border = gFilterBorder ? JPEG_FILTER_INSET_PX : 0;
  const uint16_t fw = (uint16_t)(w + 2 * border);
  const uint16_t fh = (uint16_t)(h + 2 * border);
  const size_t canvasBytes = (size_t)fw * (size_t)fh * sizeof(uint16_t);

  // Always collect the stage timings: the EXIF header records them.
  // CAPTURE_TIMING only controls the optional diagnostic log/selftest.
  uint32_t tStage = (uint32_t)micros();
  auto mark = [&](uint32_t& field) {
    const uint32_t now = (uint32_t)micros();
    field = (now - tStage) / 1000u;
    tStage = now;
  };

  uint8_t* canvas = (uint8_t*)ps_malloc(canvasBytes);
  if (!canvas) return false;

  // Decoding goes through jpg2rgb565(): the callback form of esp_jpg_decode()
  // proved unstable on this target (see the border filter) and panicked.
  if (!jpg2rgb565(fb->buf, fb->len, canvas, JPG_SCALE_NONE)) {
    free(canvas);
    return false;
  }

  const bool doLook = gFilterLook != FILTER_LOOK_NONE;
  if (doLook) usmInitLuts();
  mark(gJpegCaptureTiming.decodeMs);

  uint16_t* px = (uint16_t*)canvas;
  if (doLook) {
    const float histPerL = (float)SUPERIA_HIST_N / SUPERIA_HIST_LMAX;

    // Pass 1: masked colour sums for the auto balance, split across both cores.
    // The lab restricts the mean to midtone pixels (0.10 < L < 0.90) so
    // shadows/highlights do not skew the channel gains.
    gBandPx = px;
    gBandW = w;
    memset(usmSums, 0, sizeof(usmSums));
    parallelRun(usmBalanceBand, h);
    const uint32_t sumR = usmSums[0].sumR + usmSums[1].sumR;
    const uint32_t sumG = usmSums[0].sumG + usmSums[1].sumG;
    const uint32_t sumB = usmSums[0].sumB + usmSums[1].sumB;
    const uint32_t maskedN = usmSums[0].maskedN + usmSums[1].maskedN;

    float gainR = 1.0f, gainG = 1.0f, gainB = 1.0f;
    if (FILM_AUTO_BALANCE > 0 && maskedN >= 200) {
      const float mr = sumR / (float)maskedN / 255.0f;
      const float mg = sumG / (float)maskedN / 255.0f;
      const float mb = sumB / (float)maskedN / 255.0f;
      const float target = (mr + mg + mb) / 3.0f;
      gainR = 1.0f + (target / (mr > 1e-4f ? mr : 1e-4f) - 1.0f) * FILM_AUTO_BALANCE;
      gainG = 1.0f + (target / (mg > 1e-4f ? mg : 1e-4f) - 1.0f) * FILM_AUTO_BALANCE;
      gainB = 1.0f + (target / (mb > 1e-4f ? mb : 1e-4f) - 1.0f) * FILM_AUTO_BALANCE;
    }

    // Pass 2: histogram of the post-balance, pre-WB luma (the lab computes its
    // contrast percentiles at exactly this stage of its chain), split across
    // both cores. The two bin tables are merged before the percentile scan.
    memset(usmHist, 0, sizeof(usmHist));
#if JPEG_FILTER_PARALLEL
    memset(usmHistB, 0, sizeof(usmHistB));
#endif
    gBandGainR = gainR;
    gBandGainG = gainG;
    gBandGainB = gainB;
    gBandHistPerL = histPerL;
    parallelRun(usmHistBand, h);
#if JPEG_FILTER_PARALLEL
    for (int b = 0; b < SUPERIA_HIST_N; b++) usmHist[b] += usmHistB[b];
#endif
    const uint32_t kLo = (uint32_t)(0.01f * (float)(n - 1));
    const uint32_t kHi = (uint32_t)(0.99f * (float)(n - 1));
    float lo = SUPERIA_HIST_LMAX, hi = SUPERIA_HIST_LMAX;
    uint32_t cum = 0;
    for (int b = 0; b < SUPERIA_HIST_N; b++) {
      cum += usmHist[b];
      if (cum > kLo) { lo = ((float)b + 0.5f) / histPerL; break; }
    }
    cum = 0;
    for (int b = 0; b < SUPERIA_HIST_N; b++) {
      cum += usmHist[b];
      if (cum > kHi) { hi = ((float)b + 0.5f) / histPerL; break; }
    }

    mark(gJpegCaptureTiming.statsMs);

    // Pass 3: the whole look in one per-pixel pass, in lab order. The white
    // balance is one byte lookup per channel (fused toLin/gain/toSrgb table).
    // The look-constant stage set comes from LOOK_STAGES (compile time, derived
    // from the same table); only the content-gated stages stay runtime flags.
    const bool doBalance  = (FILM_AUTO_BALANCE > 0) && (maskedN >= 200);
    const bool doContrast = (FILM_AUTO_CONTRAST > 0) && ((hi - lo) >= 0.02f);
    const float cx = (float)(w - 1) * 0.5f, cy = (float)(h - 1) * 0.5f;
    const float cxn = cx > 1.0f ? cx : 1.0f, cyn = cy > 1.0f ? cy : 1.0f;
    const float invCxn = 1.0f / cxn, invCyn = 1.0f / cyn;

    if (doContrast) {
      float ac = FILM_AUTO_CONTRAST;
      const float gain = (0.960f - 0.015f) / (hi - lo);
      if (gain > 1.0f) {
        const float acCap =
            (SUPERIA_CONTRAST_SLOPE_CAP - 1.0f) / (gain - 1.0f);
        if (ac > acCap) ac = acCap;
      }
      for (int i = 0; i < SUPERIA_CSCALE_N; i++) {
        const float L = (float)i * USM_INV255;
        const float t = usmClampF((L - lo) / (hi - lo), 0.0f, 1.0f);
        const float s = 0.015f + t * (0.960f - 0.015f);
        usmLutContrast[i] = L > 1e-4f ? (L + (s - L) * ac) / L : 1.0f;
      }
    }

    const UsmLook look = { px, w, h, gainR, gainG, gainB,
                           doBalance, doContrast,
                           cx, cy, invCxn, invCyn };
    // Select the look's compile-time instantiation once, then let parallelRun
    // hand both row bands through the same function pointer.
    gLookCtx = &look;
    switch (gFilterLook) {
      case FILTER_LOOK_SUPERIA:
        gLookBandFn = &usmLookBandT<LOOK_STAGES[FILTER_LOOK_SUPERIA]>;
        break;
      case FILTER_LOOK_KODACHROME:
        gLookBandFn = &usmLookBandT<LOOK_STAGES[FILTER_LOOK_KODACHROME]>;
        break;
      case FILTER_LOOK_TMAX:
        gLookBandFn = &usmLookBandT<LOOK_STAGES[FILTER_LOOK_TMAX]>;
        break;
      case FILTER_LOOK_PORTRA:
        gLookBandFn = &usmLookBandT<LOOK_STAGES[FILTER_LOOK_PORTRA]>;
        break;
      default:
        gLookBandFn = nullptr;
        break;  // NONE never reaches the look pass (doLook guard above)
    }
    if (gLookBandFn) parallelRun(usmLookBand, h);
    mark(gJpegCaptureTiming.lookMs);

    // ---- sharpen: separable gaussian unsharp per channel ----------------------
    // A full-frame blur plane would cost another 2.6 MB of PSRAM, and the first
    // implementation kept a ~0.67 MB banded temp buffer instead -- which pushed
    // peak allocation to ~3.6 MB, past what the ESP32-CAM's PSRAM actually
    // answers, and the captures came out as block-garbage colour chaos. So the
    // blur now keeps only the vertical kernel window: 2*rad+1 horizontally-blurred
    // rows (61 KB at SXGA, rad = 3) in a ring. Row y is unsharpened against rows
    // y-rad..y+rad of that ring -- identical math to the full-frame separable
    // gaussian with edge replication, without the second plane.
    //
    // The ring lives in internal DRAM, not PSRAM: the vertical pass streams
    // 2*rad+1 rows per output row, and in PSRAM that was ~110 MB of bus traffic
    // per shot -- 13 s of the capture. Internal SRAM answers it without touching
    // the SPI bus. The ring is statically sized for the default sigma; a wider
    // frame or kernel falls back to the heap.
    //
    // The taps are integer Q8, not float. On this core a float MAC is a
    // dependent `madd.s` chain with ~10 cycle latency and `/ 64.0f` costs a
    // __divsf3 call, while `mull` + add pipelines; the rows are already
    // quantised to 8 bits, so nothing is lost beyond 1 LSB of blur rounding.

    if (FILM_USM_AMOUNT > 0 && FILM_USM_SIGMA > 0) {
      int rad = FILM_USM_RAD;
      if (rad < 1) rad = 1;
      if (rad > 8) rad = 8;  // the switch below instantiates radii 1..8

      // Q8 weights normalised to sum exactly 256, so the blur is mean-preserving
      // and the tails survive rounding (Q6 zeroed them: 0.011 * 64 < 0.5).
      float kernF[SUPERIA_USM_MAX_RADIUS * 2 + 1];
      float ksum = 0.0f;
      for (int d = -rad; d <= rad; d++) {
        const float v = expf(-0.5f * (float)(d * d) * (1.0f / (FILM_USM_SIGMA * FILM_USM_SIGMA)));
        kernF[d + rad] = v;
        ksum += v;
      }
      int kern[SUPERIA_USM_MAX_RADIUS * 2 + 1];
      int ksumQ8 = 0;
      for (int d = 0; d < 2 * rad + 1; d++) {
        kern[d] = (int)(kernF[d] * (256.0f / ksum) + 0.5f);
        ksumQ8 += kern[d];
      }
      kern[rad] += 256 - ksumQ8;
      const int amountQ8 = (int)(FILM_USM_AMOUNT * 256.0f + 0.5f);

      const int slots = usmSlotsPerGroup(rad);
      const bool heapRing =
          (w > SUPERIA_USM_RING_W) || (slots > USM_SLOTS_PER_GROUP);
      const bool heapRow = (w > SUPERIA_USM_RING_W);
      int16_t* ring = heapRing
          ? (int16_t*)ps_malloc(3u * (size_t)w * (size_t)slots * sizeof(int16_t))
          : usmRing;
      uint8_t* row8 = heapRow ? (uint8_t*)ps_malloc(3u * (size_t)w) : usmRow8;
      if (!ring || !row8) {
        if (heapRing && ring) free(ring);
        if (heapRow && row8) free(row8);
        free(canvas);
        return false;
      }

      switch (rad) {
        case 1: usmSharpenT<1>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 2: usmSharpenT<2>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 3: usmSharpenT<3>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 4: usmSharpenT<4>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 5: usmSharpenT<5>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 6: usmSharpenT<6>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        case 7: usmSharpenT<7>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
        default: usmSharpenT<8>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing, gFilm->mono); break;
      }
      if (heapRing) free(ring);
      if (heapRow) free(row8);
    }

    mark(gJpegCaptureTiming.sharpenMs);
  }  // doLook

  // ---- encode ----------------------------------------------------------------

  // Byte-order boundary. fmt2jpg_cb() reads RGB565 input unconditionally
  // big-endian (its convert_line_format takes R from the first byte), while
  // the canvas is little-endian standard R5G6B5. One swap pass brings the
  // processed canvas to the encoder's byte order -- the same swap the white
  // border filter does before its encode.
  for (size_t i = 0; i < n; i++) {
    px[i] = (uint16_t)((px[i] >> 8) | (px[i] << 8));
  }

  // Final node: preserve the image and add the white frame around it. Rows
  // move bottom-up (destination is always below the source) so no unread image
  // row is overwritten. 0xFFFF is white in either byte order.
  if (border > 0) {
    for (int y = (int)h - 1; y >= 0; y--) {
      memmove(px + (size_t)(y + border) * fw + border,
              px + (size_t)y * w,
              (size_t)w * sizeof(uint16_t));
    }
    const size_t rowBytes = (size_t)fw * sizeof(uint16_t);
    for (int y = 0; y < border; y++) {
      memset(px + (size_t)y * fw, 0xFF, rowBytes);
      memset(px + (size_t)(fh - 1 - y) * fw, 0xFF, rowBytes);
    }
    for (int y = border; y < (int)fh - border; y++) {
      uint16_t* row = px + (size_t)y * fw;
      for (int x = 0; x < border; x++) {
        row[x] = 0xFFFF;
        row[fw - 1 - x] = 0xFFFF;
      }
    }
  }

  FilmJpgOutCtx out = { write, arg, 0, false };
  const bool encoded = fmt2jpg_cb(canvas, canvasBytes, fw, fh, PIXFORMAT_RGB565,
                                  quality, filmJpgOutCb, &out);
  free(canvas);
  mark(gJpegCaptureTiming.encodeMs);

  *written = out.written;
  *outW = fw;
  *outH = fh;
  return encoded && !out.failed && out.written > 0;
}
