#pragma once
#include <Arduino.h>
#include <string.h>
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// Post-processing pipeline applied to a captured JPEG before it is written to
// the SD card. Selected at compile time; NO_FILTER is the default and is a
// pure pass-through -- the camera's JPEG is written verbatim, no decode.
//
//   JPEG_FILTER_NONE          write the camera frame unchanged (zero cost)
//   JPEG_FILTER_WHITE_BORDER  paint a white border over the frame edges; the
//                             outer 20 px are lost, with no resampling
//   JPEG_FILTER_SUPERIA_USM   Fuji Superia 400 film look + unsharp sharpen,
//                             ported from the tools/preview_lab chain with
//                             the lab's default parameters
//   JPEG_FILTER_KODACHROME_USM Kodak Kodachrome 64 film look + vibrance +
//                             unsharp sharpen, ported from the preview_lab
//                             chain "Kodachrome 64 -> Vibrance -> Sharpen"
//   JPEG_FILTER_TMAX_USM      Kodak T-Max 400 panchromatic B&W + unsharp
//                             sharpen, ported from the preview_lab chain
//                             "T-Max 400 -> Sharpen"
//
// Switch in platformio.ini:
//   build_flags = -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
// ---------------------------------------------------------------------------
#define JPEG_FILTER_NONE           0
#define JPEG_FILTER_WHITE_BORDER   1
#define JPEG_FILTER_SUPERIA_USM    2
#define JPEG_FILTER_KODACHROME_USM 3
#define JPEG_FILTER_TMAX_USM       4

#ifndef JPEG_FILTER
#define JPEG_FILTER JPEG_FILTER_NONE
#endif

// The pipeline streams its output instead of buffering the encoded JPEG, so a
// filter never has to hold the file in RAM. That is what keeps peak PSRAM
// inside the 8 MB module.
typedef size_t (*JpegWriteFn)(void* arg, const uint8_t* data, size_t len);

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
// with the camera's config.jpeg_quality: that scale is inverted (10 = best),
// and handing 10 to the encoder puts ~5.5x-scaled standard tables plus 4:2:0
// subsampling into every still -- the chroma solarization seen on dusk
// gradients. 60 keeps coarseness at the level of the 5-bit canvas.
#ifndef JPEG_REENCODE_QUALITY
#define JPEG_REENCODE_QUALITY 60
#endif

// Output-size bound for the free-space check before a file is opened. NONE
// is exact (pass-through). The re-encoding branches measured ~0.125
// bytes/pixel worst case at encoder quality 10; the old 0.25 B/px budget
// carried a 2x margin, but finer tables grow the files ~2.5x, so above
// encoder-quality 10 the budget doubles.
#if JPEG_REENCODE_QUALITY > 10
#define JPEG_FILTER_WC_SHIFT 1  // re-encode budget: 0.5 bytes/pixel
#else
#define JPEG_FILTER_WC_SHIFT 2  // re-encode budget: 0.25 bytes/pixel
#endif
static inline size_t jpegFilterWorstCaseBytes(const camera_fb_t* fb) {
#if JPEG_FILTER == JPEG_FILTER_NONE
  return fb->len;  // exact: nothing is re-encoded
#else
  return ((size_t)fb->width * (size_t)fb->height) >> JPEG_FILTER_WC_SHIFT;
#endif
}

// Border thickness, in output pixels, on every side.
#ifndef JPEG_FILTER_BORDER_PX
#define JPEG_FILTER_BORDER_PX 20
#endif

// The canvas keeps the capture's dimensions (SXGA stays 1280x1024); the
// border is carved out of the image area instead of growing the frame. That
// needs only ONE large buffer -- the RGB565 canvas -- because the JPEG is
// decoded directly into it. At SXGA that canvas is 2,621,440 bytes (2.5 MiB).
// With two SXGA camera buffers, the calculated peak is about 3.2 MB including
// decoder/encoder work space, well within 8 MB PSRAM.

// The decoder emits RGB565 in a byte order intended for LCD panels. If the
// re-encoded photo comes out with red and blue swapped, set this to 1.
#ifndef JPEG_FILTER_SWAP565
#define JPEG_FILTER_SWAP565 0
#endif

// The pipeline streams its output instead of buffering the encoded JPEG, so a
// filter never has to hold the file in RAM. That is what keeps peak PSRAM
// inside the 8 MB module.
typedef size_t (*JpegWriteFn)(void* arg, const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
#if (JPEG_FILTER == JPEG_FILTER_NONE)

// Pass-through: one write of the camera buffer, untouched.
static inline bool jpegFilterStream(camera_fb_t* fb, uint8_t quality,
                                    JpegWriteFn write, void* arg,
                                    size_t* written, uint16_t* outW,
                                    uint16_t* outH) {
  (void)quality;
  *outW = fb->width;
  *outH = fb->height;
  const size_t n = write(arg, fb->buf, fb->len);
  *written = n;
  return n == fb->len;
}

// ---------------------------------------------------------------------------
#elif (JPEG_FILTER == JPEG_FILTER_WHITE_BORDER)

#include "esp32-hal-psram.h"
#include "img_converters.h"
#include "esp_jpg_decode.h"

// Decoding goes through jpg2rgb565() below: the reader/writer callback form of
// esp_jpg_decode() proved unstable on this target and panicked.

// Adapter from fmt2jpg_cb's callback into the caller's sink.
struct JpgOutCtx {
  JpegWriteFn write;
  void* arg;
  size_t written;
  bool failed;
};

static size_t jpgOutCb(void* arg, size_t index, const void* data, size_t len) {
  JpgOutCtx* c = (JpgOutCtx*)arg;
  (void)index;
  const size_t n = c->write(c->arg, (const uint8_t*)data, len);
  c->written += n;
  if (n != len) c->failed = true;
  return n;
}

// Decode the frame into one RGB565 canvas the size of the capture, then paint
// a white band around the outside. The canvas keeps the capture's dimensions;
// the photo is inset by cropping `b` pixels on each edge, which needs no extra
// memory and does no resampling (a 1.6 % crop at SXGA).
static inline bool jpegFilterStream(camera_fb_t* fb, uint8_t quality,
                                    JpegWriteFn write, void* arg,
                                    size_t* written, uint16_t* outW,
                                    uint16_t* outH) {
  *written = 0;
  *outW = 0;
  *outH = 0;

  const uint16_t w = fb->width;
  const uint16_t h = fb->height;
  const uint16_t b = JPEG_FILTER_BORDER_PX;
  if (w <= (uint16_t)(2 * b) || h <= (uint16_t)(2 * b)) return false;

  const size_t canvasBytes = (size_t)w * h * sizeof(uint16_t);
  uint8_t* canvas = (uint8_t*)ps_malloc(canvasBytes);
  if (!canvas) return false;

  if (!jpg2rgb565(fb->buf, fb->len, canvas, JPG_SCALE_NONE)) {
    free(canvas);
    return false;
  }

  // White border; 0xFFFF is white in either RGB565 byte order.
  uint16_t* px = (uint16_t*)canvas;
  for (uint16_t y = 0; y < h; y++) {
    uint16_t* row = px + (size_t)y * w;
    if (y < b || y >= (uint16_t)(h - b)) {
      memset(row, 0xFF, (size_t)w * sizeof(uint16_t));
    } else {
      memset(row, 0xFF, (size_t)b * sizeof(uint16_t));
      memset(row + (w - b), 0xFF, (size_t)b * sizeof(uint16_t));
    }
  }

#if JPEG_FILTER_SWAP565
  // fmt2jpg_cb() reads RGB565 input big-endian (high byte first) while
  // jpg2rgb565() wrote this canvas little-endian; the swap converts the canvas
  // to the encoder's byte order so the channels encode true.
  for (size_t i = 0; i < (size_t)w * h; i++) {
    px[i] = (uint16_t)((px[i] >> 8) | (px[i] << 8));
  }
#endif

  JpgOutCtx out = { write, arg, 0, false };
  const bool encoded = fmt2jpg_cb(canvas, canvasBytes, w, h, PIXFORMAT_RGB565,
                                  quality, jpgOutCb, &out);
  free(canvas);

  *written = out.written;
  *outW = w;
  *outH = h;
  return encoded && !out.failed && out.written > 0;
}

// ---------------------------------------------------------------------------
#elif (JPEG_FILTER == JPEG_FILTER_SUPERIA_USM) || \
      (JPEG_FILTER == JPEG_FILTER_KODACHROME_USM) || \
      (JPEG_FILTER == JPEG_FILTER_TMAX_USM)

#include "esp32-hal-psram.h"
#include "img_converters.h"

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

// Film-look + unsharp sharpening, shared scaffolding for the look filters:
//   Superia    "Fuji Superia 400 -> Sharpen"
//   Kodachrome "Kodak Kodachrome 64 -> Vibrance -> Sharpen"
//   T-Max      "Kodak T-Max 400 -> Sharpen" (panchromatic B&W)
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
// defaults make them identity/off); T-Max skips matrix, saturation, vibrance
// and tints, and enables the monochrome mix. Values differ from the lab by
// construction because the canvas is RGB565 (5/6/5 bits): expect mild extra
// banding on smooth gradients vs. the lab's 8-bit model. Kodachrome's/T-Max's
// grain size of 1.2 is below the lab's blur threshold (sigma < 0.3), so only
// the amount is reproduced; the device grain is the same per-pixel noise,
// monochrome.

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
// ceil(3*sigma) as an integer constant, so the DRAM ring can be a static array.
#ifndef SUPERIA_USM_RAD
#define SUPERIA_USM_RAD ((int)(3.0f * SUPERIA_USM_SIGMA + 0.999f))
#endif
// Ring width in pixels; SXGA is 1280. Frames wider than this (UXGA = 1600)
// fall back to a PSRAM ring at capture time.
#ifndef SUPERIA_USM_RING_W
#define SUPERIA_USM_RING_W 1280
#endif

// ---- per-filter look parameters -------------------------------------------
// The shared pipeline reads FILM_*; each filter maps those to its own
// -D-overridable names. Kodachrome values are the preview_lab node defaults
// for the chain "Kodachrome 64 -> Vibrance (0.5) -> Sharpen (0.4, sigma 1)".
#if JPEG_FILTER == JPEG_FILTER_KODACHROME_USM
#ifndef KODACHROME_AUTO_BALANCE
#define KODACHROME_AUTO_BALANCE 0.62f
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
#define KODACHROME_TONE_CONTRAST 0.60f
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
#define KODACHROME_GRAIN 0.010f
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
#endif  // JPEG_FILTER_KODACHROME_USM

// T-Max 400 values are the preview_lab node defaults for
// "T-Max 400 -> Sharpen (0.4, sigma 1)".
#if JPEG_FILTER == JPEG_FILTER_TMAX_USM
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
#endif  // JPEG_FILTER_TMAX_USM

#if JPEG_FILTER == JPEG_FILTER_KODACHROME_USM
#define FILM_AUTO_BALANCE KODACHROME_AUTO_BALANCE
#define FILM_AUTO_CONTRAST KODACHROME_AUTO_CONTRAST
#define FILM_MIDTONE_GAMMA KODACHROME_MIDTONE_GAMMA
#define FILM_TONE_CONTRAST KODACHROME_TONE_CONTRAST
#define FILM_SATURATION KODACHROME_SATURATION
#define FILM_SHADOW_R KODACHROME_SHADOW_R
#define FILM_SHADOW_G KODACHROME_SHADOW_G
#define FILM_SHADOW_B KODACHROME_SHADOW_B
#define FILM_HIGHLIGHT_R KODACHROME_HIGHLIGHT_R
#define FILM_HIGHLIGHT_G KODACHROME_HIGHLIGHT_G
#define FILM_HIGHLIGHT_B KODACHROME_HIGHLIGHT_B
#define FILM_VIGNETTE KODACHROME_VIGNETTE
#define FILM_GRAIN KODACHROME_GRAIN
#define FILM_VIBRANCE KODACHROME_VIBRANCE
#define FILM_USM_AMOUNT KODACHROME_USM_AMOUNT
#define FILM_USM_SIGMA KODACHROME_USM_SIGMA
#define FILM_USM_RAD ((int)(3.0f * KODACHROME_USM_SIGMA + 0.999f))
#define FILM_WB_R 1.0f
#define FILM_WB_G 1.0f
#define FILM_WB_B 1.0f
#define FILM_HAS_MATRIX 1
#define FILM_TINTS_GRADED 1
#define FILM_MONO 0
#elif JPEG_FILTER == JPEG_FILTER_TMAX_USM
#define FILM_AUTO_BALANCE TMAX_AUTO_BALANCE
#define FILM_AUTO_CONTRAST TMAX_AUTO_CONTRAST
#define FILM_MIDTONE_GAMMA TMAX_MIDTONE_GAMMA
#define FILM_TONE_CONTRAST TMAX_TONE_CONTRAST
#define FILM_SATURATION 1.0f
#define FILM_SHADOW_R 0.0f
#define FILM_SHADOW_G 0.0f
#define FILM_SHADOW_B 0.0f
#define FILM_HIGHLIGHT_R 0.0f
#define FILM_HIGHLIGHT_G 0.0f
#define FILM_HIGHLIGHT_B 0.0f
#define FILM_VIGNETTE TMAX_VIGNETTE
#define FILM_GRAIN TMAX_GRAIN
#define FILM_VIBRANCE 0.0f
#define FILM_USM_AMOUNT TMAX_USM_AMOUNT
#define FILM_USM_SIGMA TMAX_USM_SIGMA
#define FILM_USM_RAD ((int)(3.0f * TMAX_USM_SIGMA + 0.999f))
#define FILM_WB_R 1.0f
#define FILM_WB_G 1.0f
#define FILM_WB_B 1.0f
#define FILM_HAS_MATRIX 0
#define FILM_TINTS_GRADED 0
#define FILM_MONO 1
#define FILM_MONO_R TMAX_MONO_R
#define FILM_MONO_G TMAX_MONO_G
#define FILM_MONO_B TMAX_MONO_B
#else
#define FILM_AUTO_BALANCE SUPERIA_AUTO_BALANCE
#define FILM_AUTO_CONTRAST SUPERIA_AUTO_CONTRAST
#define FILM_MIDTONE_GAMMA SUPERIA_MIDTONE_GAMMA
#define FILM_TONE_CONTRAST 0.0f
#define FILM_SATURATION 1.0f
#define FILM_SHADOW_R SUPERIA_SHADOW_R
#define FILM_SHADOW_G SUPERIA_SHADOW_G
#define FILM_SHADOW_B SUPERIA_SHADOW_B
#define FILM_HIGHLIGHT_R SUPERIA_HIGHLIGHT_R
#define FILM_HIGHLIGHT_G SUPERIA_HIGHLIGHT_G
#define FILM_HIGHLIGHT_B SUPERIA_HIGHLIGHT_B
#define FILM_VIGNETTE SUPERIA_VIGNETTE
#define FILM_GRAIN 0.0f
#define FILM_VIBRANCE 0.0f
#define FILM_USM_AMOUNT SUPERIA_USM_AMOUNT
#define FILM_USM_SIGMA SUPERIA_USM_SIGMA
#define FILM_USM_RAD SUPERIA_USM_RAD
#define FILM_WB_R SUPERIA_WB_R
#define FILM_WB_G SUPERIA_WB_G
#define FILM_WB_B SUPERIA_WB_B
#define FILM_HAS_MATRIX 0
#define FILM_TINTS_GRADED 0
#define FILM_MONO 0
#endif

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
static float   usmLutGamma[SUPERIA_LUT_N];  // powf(x, midtone gamma), once
static float   usmLutVign[SUPERIA_LUT_N];   // powf(d, 1.2), once
// The midtone gamma scales all three channels by lutGamma[L]/L. That division
// is a __divsf3 call per pixel, so the quotient is tabulated next to the curve.
// The film "contrast" smoothstep is folded into the same curve.
static float   usmLutGammaScale[SUPERIA_LUT_N];
// Auto contrast's scale is (L + (s-L)*ac)/L, and `s` depends on this frame's
// percentiles, so it is rebuilt per shot -- 512 divisions instead of 1.31 M.
// Bins cover L in [0, 2]: the balance gain can push luma past 1, and clamping
// that back to 1 would visibly lift highlights.
#define SUPERIA_CSCALE_N 512
static float   usmLutContrast[SUPERIA_CSCALE_N];
// Vibrance needs 1/max(r,g,b); 256 divisions once instead of one per pixel.
static float   usmLutInvMax[SUPERIA_LUT_N];

static const float filmWbGain[3] = { FILM_WB_R, FILM_WB_G, FILM_WB_B };
static bool usmLutsReady = false;

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
  if (usmLutsReady) return;
  for (int i = 0; i < SUPERIA_LUT_N; i++) {
    const float t = i / (float)(SUPERIA_LUT_N - 1);
    usmLutToLin[i] = t <= 0.04045f ? t / 12.92f
                                   : powf((t + 0.055f) / 1.055f, 2.4f);
  }
  usmBuildWbLut();
  // Compile-time conditions: an identity/unwanted curve is never built.
  if (FILM_MIDTONE_GAMMA != 1.0f || FILM_TONE_CONTRAST != 0.0f)
    for (int i = 0; i < SUPERIA_LUT_N; i++) {
      const float t = i / (float)(SUPERIA_LUT_N - 1);
      float v = FILM_MIDTONE_GAMMA != 1.0f ? powf(t, FILM_MIDTONE_GAMMA) : t;
      if (FILM_TONE_CONTRAST != 0.0f)
        v = v + FILM_TONE_CONTRAST * (v * v * (3.0f - 2.0f * v) - v);
      v = usmClampF(v, 0.0f, 1.0f);
      usmLutGamma[i] = v;
      usmLutGammaScale[i] = t > 1e-4f ? v / t : 0.0f;
    }
  // Vignette needs r^2.4 where r = sqrt(d), so powf(d, 1.2) -- one lookup, no
  // per-pixel sqrt.
  if (FILM_VIGNETTE > 0)
    for (int i = 0; i < SUPERIA_LUT_N; i++)
      usmLutVign[i] = powf(i / (float)(SUPERIA_LUT_N - 1), 1.2f);
  if (FILM_VIBRANCE > 0)
    for (int i = 0; i < SUPERIA_LUT_N; i++)
      usmLutInvMax[i] = i > 0 ? 255.0f / (float)i : 0.0f;
  usmLutsReady = true;
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

// Sharpen ring window, in internal DRAM. One 16-byte group per (channel, x)
// holding 8 int16 slots in ring-slot order; one aligned 128-bit load feeds
// the S3 dot-product kernel. At SXGA this is 61 KB (the old byte layout was
// 27 KB), still comfortably inside the S3's 512 KB SRAM. Wider kernels use
// 16/24-slot groups and the heap + scalar path.
#define USM_SLOTS_PER_GROUP 8
static int16_t usmRing[3u * SUPERIA_USM_RING_W * USM_SLOTS_PER_GROUP]
    __attribute__((aligned(16)));
static uint8_t usmRow8[3u * SUPERIA_USM_RING_W];

// Slots per 16-byte group for a given radius: 8 for rad <= 3, 16/24 above.
static inline int usmSlotsPerGroup(int rad) {
  return ((2 * rad + 1 + 7) / 8) * 8;
}

// ---- look pass -------------------------------------------------------------
// Per-pixel, no neighbour reads. Kept as a row-range function so a second core
// could take a band later; currently run on one core (a second busy core drew
// enough current to brown out the board).
struct UsmLook {
  uint16_t* px;
  int w, h;
  float gainR, gainG, gainB;
  bool doBalance, doContrast, doWb, doTone, doVign;
  bool doMatrix, doSat, doVibrance, doGrain;
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

static void usmLookRows(const UsmLook& c, int y0, int y1) {
  uint16_t* px = c.px;
  const int w = c.w;
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
      if (c.doWb) {
        for (int ch = 0; ch < 3; ch++)
          f[ch] = usmLutWb[ch][usmToU8(f[ch] * 255.0f)] * USM_INV255;
      }
      L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];

#if FILM_HAS_MATRIX
      if (c.doMatrix) {
        const float r = f[0], g = f[1], b = f[2];
        f[0] = KODACHROME_MAT_RR * r + KODACHROME_MAT_RG * g + KODACHROME_MAT_RB * b;
        f[1] = KODACHROME_MAT_GR * r + KODACHROME_MAT_GG * g + KODACHROME_MAT_GB * b;
        f[2] = KODACHROME_MAT_BR * r + KODACHROME_MAT_BG * g + KODACHROME_MAT_BB * b;
      }
#endif
      if (c.doSat) {
        L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
        for (int ch = 0; ch < 3; ch++) f[ch] = L + (f[ch] - L) * FILM_SATURATION;
      }
      L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];

      if (c.doTone) {
        const float sc = usmLutGammaScale[usmToU8(L * 255.0f)];
        f[0] *= sc; f[1] *= sc; f[2] *= sc;
      }
#if FILM_TINTS_GRADED
      // The lab tints read the luma of the graded image, after gamma/contrast.
      L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
#endif
      {
        const float Lc = usmClampF(L, 0.0f, 1.0f);
        const float shw = (1.0f - Lc) * (1.0f - Lc);
        const float hiw = Lc * Lc;
        f[0] += FILM_SHADOW_R * shw + FILM_HIGHLIGHT_R * hiw;
        f[1] += FILM_SHADOW_G * shw + FILM_HIGHLIGHT_G * hiw;
        f[2] += FILM_SHADOW_B * shw + FILM_HIGHLIGHT_B * hiw;
      }
#if FILM_MONO
      // Panchromatic B&W mix; the lab applies it after the tints and before
      // the optical stages. T-Max's node tint is neutral, so no tint term.
      {
        const float g = FILM_MONO_R * f[0] + FILM_MONO_G * f[1] + FILM_MONO_B * f[2];
        f[0] = g; f[1] = g; f[2] = g;
      }
#endif
      if (c.doVign) {
        const float xx = ((float)x - c.cx) * c.invCxn;
        const float d = (xx * xx + yy * yy) * 0.5f;
        const float vs = usmClampF(1.0f - FILM_VIGNETTE * usmLut8(usmLutVign, d), 0.0f, 1.0f);
        f[0] *= vs; f[1] *= vs; f[2] *= vs;
      }
      if (c.doGrain) {
        const float Lg = usmClampF(0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2],
                                   0.0f, 1.0f);
        const float wgt = 0.35f + 0.65f * (1.0f - fabsf(2.0f * Lg - 1.0f));
        const float n = usmNoise(x, y) * (1.514f * FILM_GRAIN * wgt);
        f[0] += n; f[1] += n; f[2] += n;
      }
      if (c.doVibrance) {
        float mx = f[0], mn = f[0];
        if (f[1] > mx) mx = f[1];
        if (f[2] > mx) mx = f[2];
        if (f[1] < mn) mn = f[1];
        if (f[2] < mn) mn = f[2];
        const float sat = (mx - mn) * usmLutInvMax[usmToU8(mx * 255.0f)];
        const float gain = 1.0f + FILM_VIBRANCE * (1.0f - sat);
        const float avg = (f[0] + f[1] + f[2]) * (1.0f / 3.0f);
        for (int ch = 0; ch < 3; ch++) f[ch] = avg + (f[ch] - avg) * gain;
      }
      px[(size_t)y * w + x] = usmPack565(usmToU8(f[0] * 255.0f),
                                        usmToU8(f[1] * 255.0f),
                                        usmToU8(f[2] * 255.0f));
    }
  }
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
                         const int* __restrict k, int w, int stride) {
  for (int c = 0; c < 3; c++) {
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

template <int R>
static void usmSharpenT(uint16_t* px, int w, int h, int16_t* ring,
                        uint8_t* row8, const int* k, int amountQ8,
                        int stride, bool allowSimd) {
  const int ringRows = 2 * R + 1;

  // Horizontal blur of canvas row ysrc into its ring slot (ysrc outside the
  // frame replicates row 0/h-1). `ring + slot` is the group's slot 0.
  auto hblurSlot = [&](int ysrc) {
    const int yc = ysrc < 0 ? 0 : (ysrc > h - 1 ? h - 1 : ysrc);
    const uint16_t* src = px + (size_t)yc * w;
    for (int x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(src[x], rgb);
      row8[x] = rgb[0];
      row8[w + x] = rgb[1];
      row8[2 * w + x] = rgb[2];
    }
    const int slot = (ysrc % ringRows + ringRows) % ringRows;
    usmHblurRowT<R>(row8, ring + slot, k, w, stride);
  };

  for (int y = -R; y <= R; y++) hblurSlot(y);  // prime the ring window

  static int16_t coeff[24] __attribute__((aligned(16)));

  for (int y = 0; y < h; y++) {
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
    for (int c = 0; c < 3; c++) {
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
      uint8_t out[3];
      for (int c = 0; c < 3; c++) {
        const int blurred = row8[c * w + x];
        int v = (int)rgb[c] + ((((int)rgb[c] - blurred) * amountQ8 + 128) >> 8);
        out[c] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      }
      prow[x] = usmPack565(out[0], out[1], out[2]);
    }
    hblurSlot(y + R + 1);  // slide the window; overwrites row y-R's slot
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

  const uint16_t w = fb->width;
  const uint16_t h = fb->height;
  const size_t n = (size_t)w * (size_t)h;
  if (n == 0) return false;

#if CAPTURE_TIMING
  uint32_t tStage = (uint32_t)micros();
  auto mark = [&](uint32_t& field) {
    const uint32_t now = (uint32_t)micros();
    field = (now - tStage) / 1000u;
    tStage = now;
  };
#else
  auto mark = [](uint32_t&) {};
#endif

  const size_t canvasBytes = n * sizeof(uint16_t);
  uint8_t* canvas = (uint8_t*)ps_malloc(canvasBytes);
  if (!canvas) return false;

  // Decoding goes through jpg2rgb565(): the callback form of esp_jpg_decode()
  // proved unstable on this target (see the border filter) and panicked.
  if (!jpg2rgb565(fb->buf, fb->len, canvas, JPG_SCALE_NONE)) {
    free(canvas);
    return false;
  }

  usmInitLuts();
  mark(gJpegCaptureTiming.decodeMs);

  uint16_t* px = (uint16_t*)canvas;
  const float histPerL = (float)SUPERIA_HIST_N / SUPERIA_HIST_LMAX;

  // Pass 1: masked colour sums for the auto balance. The lab restricts the
  // mean to midtone pixels (0.10 < L < 0.90) so shadows/highlights do not
  // skew the channel gains.
  uint32_t sumR = 0, sumG = 0, sumB = 0, maskedN = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t rgb[3];
    usmUnpack565(px[i], rgb);
    const float L = ((0.2126f * rgb[0]) + (0.7152f * rgb[1]) + (0.0722f * rgb[2])) * USM_INV255;
    if (L > 0.10f && L < 0.90f) {
      sumR += rgb[0]; sumG += rgb[1]; sumB += rgb[2];
      maskedN++;
    }
  }

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
  // contrast percentiles at exactly this stage of its chain).
  memset(usmHist, 0, sizeof(usmHist));
  for (size_t i = 0; i < n; i++) {
    uint8_t rgb[3];
    usmUnpack565(px[i], rgb);
    const float L = ((0.2126f * rgb[0] * gainR) + (0.7152f * rgb[1] * gainG)
                   + (0.0722f * rgb[2] * gainB)) * USM_INV255;
    int bin = (int)(L * histPerL);
    if (bin < 0) bin = 0;
    if (bin > SUPERIA_HIST_N - 1) bin = SUPERIA_HIST_N - 1;
    usmHist[bin]++;
  }
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
  const bool doBalance  = (FILM_AUTO_BALANCE > 0) && (maskedN >= 200);
  const bool doContrast = (FILM_AUTO_CONTRAST > 0) && ((hi - lo) >= 0.02f);
  const bool doWb       = (FILM_WB_R != 1.0f || FILM_WB_G != 1.0f || FILM_WB_B != 1.0f);
  const bool doTone     = (FILM_MIDTONE_GAMMA != 1.0f || FILM_TONE_CONTRAST != 0.0f);
  const bool doVign     = (FILM_VIGNETTE > 0);
  const bool doMatrix   = FILM_HAS_MATRIX != 0;
  const bool doSat      = FILM_SATURATION != 1.0f;
  const bool doVibrance = FILM_VIBRANCE > 0;
  const bool doGrain    = FILM_GRAIN > 0;
  const float ac = FILM_AUTO_CONTRAST;
  const float cx = (float)(w - 1) * 0.5f, cy = (float)(h - 1) * 0.5f;
  const float cxn = cx > 1.0f ? cx : 1.0f, cyn = cy > 1.0f ? cy : 1.0f;
  const float invCxn = 1.0f / cxn, invCyn = 1.0f / cyn;

  if (doContrast) {
    for (int i = 0; i < SUPERIA_CSCALE_N; i++) {
      const float L = (float)i * USM_INV255;
      const float t = usmClampF((L - lo) / (hi - lo), 0.0f, 1.0f);
      const float s = 0.015f + t * (0.960f - 0.015f);
      usmLutContrast[i] = L > 1e-4f ? (L + (s - L) * ac) / L : 1.0f;
    }
  }

  const UsmLook look = { px, w, h, gainR, gainG, gainB,
                         doBalance, doContrast, doWb, doTone, doVign,
                         doMatrix, doSat, doVibrance, doGrain,
                         cx, cy, invCxn, invCyn };
  usmLookRows(look, 0, h);
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
      case 1: usmSharpenT<1>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 2: usmSharpenT<2>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 3: usmSharpenT<3>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 4: usmSharpenT<4>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 5: usmSharpenT<5>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 6: usmSharpenT<6>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      case 7: usmSharpenT<7>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
      default: usmSharpenT<8>(px, w, h, ring, row8, kern, amountQ8, slots, !heapRing); break;
    }
    if (heapRing) free(ring);
    if (heapRow) free(row8);
  }

  mark(gJpegCaptureTiming.sharpenMs);

  // ---- encode ----------------------------------------------------------------

  // Byte-order boundary. fmt2jpg_cb() reads RGB565 input unconditionally
  // big-endian (its convert_line_format takes R from the first byte), while
  // the canvas is little-endian standard R5G6B5. One swap pass brings the
  // processed canvas to the encoder's byte order -- the same swap the white
  // border filter does before its encode.
  for (size_t i = 0; i < n; i++) {
    px[i] = (uint16_t)((px[i] >> 8) | (px[i] << 8));
  }

  FilmJpgOutCtx out = { write, arg, 0, false };
  const bool encoded = fmt2jpg_cb(canvas, canvasBytes, w, h, PIXFORMAT_RGB565,
                                  quality, filmJpgOutCb, &out);
  free(canvas);
  mark(gJpegCaptureTiming.encodeMs);

  *written = out.written;
  *outW = w;
  *outH = h;
  return encoded && !out.failed && out.written > 0;
}

#else
#error "JPEG_FILTER must be JPEG_FILTER_NONE, JPEG_FILTER_WHITE_BORDER, JPEG_FILTER_SUPERIA_USM, JPEG_FILTER_KODACHROME_USM or JPEG_FILTER_TMAX_USM"
#endif
