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
//
// Switch in platformio.ini:
//   build_flags = -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
// ---------------------------------------------------------------------------
#define JPEG_FILTER_NONE          0
#define JPEG_FILTER_WHITE_BORDER  1
#define JPEG_FILTER_SUPERIA_USM   2

#ifndef JPEG_FILTER
#define JPEG_FILTER JPEG_FILTER_NONE
#endif

// The pipeline streams its output instead of buffering the encoded JPEG, so a
// filter never has to hold the file in RAM. That is what keeps peak PSRAM
// inside a 4 MB module.
typedef size_t (*JpegWriteFn)(void* arg, const uint8_t* data, size_t len);

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
// With one SXGA camera buffer, the calculated peak is about 2.9 MB including
// decoder/encoder work space, within 4 MB PSRAM.

// The decoder emits RGB565 in a byte order intended for LCD panels. If the
// re-encoded photo comes out with red and blue swapped, set this to 1.
#ifndef JPEG_FILTER_SWAP565
#define JPEG_FILTER_SWAP565 0
#endif

// The pipeline streams its output instead of buffering the encoded JPEG, so a
// filter never has to hold the file in RAM. That is what keeps peak PSRAM
// inside a 4 MB module.
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
#elif (JPEG_FILTER == JPEG_FILTER_SUPERIA_USM)

#include "esp32-hal-psram.h"
#include "img_converters.h"

// Fuji Superia 400 look + unsharp sharpening -- the tools/preview_lab chain
// "Fuji Superia 400 -> Sharpen" ported with the lab's default parameters.
// Design: one decode, a single fused colour pass, one neighbourhood pass,
// one encode. Every look stage is elementwise, so running them as separate
// JPEG filters would only add decode/encode round trips: extra CPU per pass
// and one generation of JPEG quantisation loss each.
//
// Stage order and formulas mirror superiaLook() / unsharpMask() in
// tools/preview_lab/app.html:
//   auto balance -> auto contrast -> linear white balance -> midtone gamma
//   -> shadow/highlight tint -> vignette -> gaussian unsharp sharpen
// The lab's grain stage is not ported: its default is 0 (off) and noise here
// would cost a full extra pass. Values differ from the lab by construction
// because the canvas is RGB565 (5/6/5 bits): expect mild extra banding on
// smooth gradients vs. the lab's 8-bit model.

// Tunables, all -D overridable. Values = preview_lab node defaults.
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
#define SUPERIA_USM_MAX_RADIUS 16
#endif

// ---- lab math helpers ------------------------------------------------------

static inline float usmClampF(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

static inline uint8_t usmToU8(float v) {
  return (uint8_t)(usmClampF(v, 0.0f, 255.0f) + 0.5f);
}

// RGB565 <-> 8-bit channels (bit replication). The linked esp32-camera JPEG
// decoder stores the canvas as native little-endian standard R5G6B5 (verified
// in the linked lib: its RGB565 writer stores out[0] = v & 0xFF, out[1] =
// v >> 8), so a plain u16 read already holds the true channels. The byte swap
// the canvas needs lives at the encode boundary (see jpegFilterStream).
static inline uint16_t usmPack565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((uint16_t)(r >> 3) << 11)
                  | ((uint16_t)(g >> 2) << 5)
                  | (uint16_t)(b >> 3));
}

static inline void usmUnpack565(uint16_t v, uint8_t rgb[3]) {
  const uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
  const uint8_t g6 = (uint8_t)((v >> 5) & 0x3F);
  const uint8_t b5 = (uint8_t)(v & 0x1F);
  rgb[0] = (uint8_t)((r5 << 3) | (r5 >> 2));  // 5 -> 8 bit replication
  rgb[1] = (uint8_t)((g6 << 2) | (g6 >> 4));  // 6 -> 8 bit replication
  rgb[2] = (uint8_t)((b5 << 3) | (b5 >> 2));
}

// The lab evaluates powf() per pixel (midtone gamma, vignette) and twice per
// channel for the white balance round trip. Per-pixel transcendentals are the
// slowest op on this MCU, so every curve becomes a once-allocated LUT with
// linear interpolation; error < 1/4096, invisible next to a 5-bit canvas.
#define SUPERIA_LUT_N 4096
static float* usmLutToLin  = NULL;  // sRGB -> linear
static float* usmLutToSrgb = NULL;  // linear -> sRGB
static float* usmLutGamma  = NULL;  // powf(x, midtone gamma)
static float* usmLutVign   = NULL;  // powf(d, 1.2), vignette falloff

// The linear white balance applies the compile-time gains via the two pow
// LUTs (below) — no separate table needed.
static const float superiaWbGain[3] = { SUPERIA_WB_R, SUPERIA_WB_G, SUPERIA_WB_B };
static bool  usmLutsReady = false;

static float* usmBuildPowLut(float exponent) {
  float* lut = (float*)ps_malloc(SUPERIA_LUT_N * sizeof(float));
  if (!lut) return NULL;
  for (int i = 0; i < SUPERIA_LUT_N; i++)
    lut[i] = powf(i / (float)(SUPERIA_LUT_N - 1), exponent);
  return lut;
}

// True sRGB transfer curves. A pure powf pair would drop the linear toe and
// the 1.055/-0.055 shoulder and bend the whole look (the lab uses the real
// piecewise curves), so the white-balance round trip gets its own builder.
static float* usmBuildSrgbLut(bool toLinear) {
  float* lut = (float*)ps_malloc(SUPERIA_LUT_N * sizeof(float));
  if (!lut) return NULL;
  for (int i = 0; i < SUPERIA_LUT_N; i++) {
    const float t = i / (float)(SUPERIA_LUT_N - 1);
    lut[i] = toLinear
        ? (t <= 0.04045f ? t / 12.92f : powf((t + 0.055f) / 1.055f, 2.4f))
        : (t <= 0.0031308f ? t * 12.92f : 1.055f * powf(t, 1.0f / 2.4f) - 0.055f);
  }
  return lut;
}

static bool usmInitLuts(void) {
  if (usmLutsReady) return true;
  usmLutToLin = usmBuildSrgbLut(true);
  usmLutToSrgb = usmBuildSrgbLut(false);
  // Compile-time conditions: an identity/unwanted curve is never allocated.
  if (SUPERIA_MIDTONE_GAMMA != 1.0f)
    usmLutGamma = usmBuildPowLut(SUPERIA_MIDTONE_GAMMA);
  // Vignette needs r^2.4 where r = sqrt(d), so powf(d, 1.2) -- one lerp, no
  // per-pixel sqrt.
  if (SUPERIA_VIGNETTE > 0)
    usmLutVign = usmBuildPowLut(1.2f);
  if (!usmLutToLin || !usmLutToSrgb
      || (SUPERIA_MIDTONE_GAMMA != 1.0f && !usmLutGamma)
      || (SUPERIA_VIGNETTE > 0 && !usmLutVign))
    return false;
  usmLutsReady = true;
  return true;
}

static inline float usmLutGet(const float* lut, float x) {
  const float t = usmClampF(x, 0.0f, 1.0f) * (float)(SUPERIA_LUT_N - 1);
  int i = (int)t;
  if (i > SUPERIA_LUT_N - 2) i = SUPERIA_LUT_N - 2;
  const float f = t - (float)i;
  return lut[i] + (lut[i + 1] - lut[i]) * f;
}

// Percentiles for the auto contrast come from a histogram, not the lab's full
// float sort: sorting 1.31M luma floats would need 5.25 MB of PSRAM that does
// not exist. Bins cover luma in [0, 1.5] (balance gains can push luma past 1),
// bounding the percentile error at ~0.05% of full scale.
#define SUPERIA_HIST_N 2048
#define SUPERIA_HIST_LMAX 1.5f
static uint32_t usmHist[SUPERIA_HIST_N];

// ---- entry point -----------------------------------------------------------

// Adapter from fmt2jpg_cb's callback into the caller's sink (as in the border
// filter; only one filter branch is compiled at a time).
struct SuperiaJpgOutCtx {
  JpegWriteFn write;
  void* arg;
  size_t written;
  bool failed;
};

static size_t superiaJpgOutCb(void* arg, size_t index, const void* data, size_t len) {
  SuperiaJpgOutCtx* c = (SuperiaJpgOutCtx*)arg;
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

  const size_t canvasBytes = n * sizeof(uint16_t);
  uint8_t* canvas = (uint8_t*)ps_malloc(canvasBytes);
  if (!canvas) return false;

  // Decoding goes through jpg2rgb565(): the callback form of esp_jpg_decode()
  // proved unstable on this target (see the border filter) and panicked.
  if (!jpg2rgb565(fb->buf, fb->len, canvas, JPG_SCALE_NONE)) {
    free(canvas);
    return false;
  }

  if (!usmInitLuts()) {
    free(canvas);
    return false;
  }

  uint16_t* px = (uint16_t*)canvas;
  const float histPerL = (float)SUPERIA_HIST_N / SUPERIA_HIST_LMAX;

  // Pass 1: masked colour sums for the auto balance. The lab restricts the
  // mean to midtone pixels (0.10 < L < 0.90) so shadows/highlights do not
  // skew the channel gains.
  uint32_t sumR = 0, sumG = 0, sumB = 0, maskedN = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t rgb[3];
    usmUnpack565(px[i], rgb);
    const float L = ((0.2126f * rgb[0]) + (0.7152f * rgb[1]) + (0.0722f * rgb[2])) / 255.0f;
    if (L > 0.10f && L < 0.90f) {
      sumR += rgb[0]; sumG += rgb[1]; sumB += rgb[2];
      maskedN++;
    }
  }

  float gainR = 1.0f, gainG = 1.0f, gainB = 1.0f;
  if (SUPERIA_AUTO_BALANCE > 0 && maskedN >= 200) {
    const float mr = sumR / (float)maskedN / 255.0f;
    const float mg = sumG / (float)maskedN / 255.0f;
    const float mb = sumB / (float)maskedN / 255.0f;
    const float target = (mr + mg + mb) / 3.0f;
    gainR = 1.0f + (target / (mr > 1e-4f ? mr : 1e-4f) - 1.0f) * SUPERIA_AUTO_BALANCE;
    gainG = 1.0f + (target / (mg > 1e-4f ? mg : 1e-4f) - 1.0f) * SUPERIA_AUTO_BALANCE;
    gainB = 1.0f + (target / (mb > 1e-4f ? mb : 1e-4f) - 1.0f) * SUPERIA_AUTO_BALANCE;
  }

  // Pass 2: histogram of the post-balance, pre-WB luma (the lab computes its
  // contrast percentiles at exactly this stage of its chain).
  memset(usmHist, 0, sizeof(usmHist));
  for (size_t i = 0; i < n; i++) {
    uint8_t rgb[3];
    usmUnpack565(px[i], rgb);
    const float L = ((0.2126f * rgb[0] * gainR) + (0.7152f * rgb[1] * gainG)
                   + (0.0722f * rgb[2] * gainB)) / 255.0f;
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

  // Pass 3: the whole look in one per-pixel pass, in lab order. The white
  // balance uses two LUT lerps per channel -- same formula as
  // linToSrgb(srgbToLin(f) * gain), just without two powf calls per channel.
  const bool doBalance  = (SUPERIA_AUTO_BALANCE > 0) && (maskedN >= 200);
  const bool doContrast = (SUPERIA_AUTO_CONTRAST > 0) && ((hi - lo) >= 0.02f);
  const bool doGamma    = (SUPERIA_MIDTONE_GAMMA != 1.0f);
  const bool doVign     = (SUPERIA_VIGNETTE > 0);
  const float ac = SUPERIA_AUTO_CONTRAST;
  const float cx = (float)(w - 1) * 0.5f, cy = (float)(h - 1) * 0.5f;
  const float cxn = cx > 1.0f ? cx : 1.0f, cyn = cy > 1.0f ? cy : 1.0f;

  for (uint16_t y = 0; y < h; y++) {
    const float yy = ((float)y - cy) / cyn;
    for (uint16_t x = 0; x < w; x++) {
      uint8_t rgb[3];
      usmUnpack565(px[(size_t)y * w + x], rgb);
      float f[3] = { rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f };

      if (doBalance) {
        f[0] *= gainR; f[1] *= gainG; f[2] *= gainB;
      }
      float L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      if (doContrast) {
        const float t = usmClampF((L - lo) / (hi - lo), 0.0f, 1.0f);
        const float s = 0.015f + t * (0.960f - 0.015f);
        const float sc = (L + (s - L) * ac) / (L > 1e-4f ? L : 1e-4f);
        f[0] *= sc; f[1] *= sc; f[2] *= sc;
      }
      for (int c = 0; c < 3; c++)
        f[c] = usmLutGet(usmLutToSrgb,
                         usmLutGet(usmLutToLin, f[c]) * superiaWbGain[c]);
      L = 0.2126f * f[0] + 0.7152f * f[1] + 0.0722f * f[2];
      if (doGamma) {
        const float v = usmLutGet(usmLutGamma, L);
        const float sc = v / (L > 1e-4f ? L : 1e-4f);
        f[0] *= sc; f[1] *= sc; f[2] *= sc;
      }
      {
        const float Lc = usmClampF(L, 0.0f, 1.0f);
        const float shw = (1.0f - Lc) * (1.0f - Lc);
        const float hiw = Lc * Lc;
        f[0] += SUPERIA_SHADOW_R * shw + SUPERIA_HIGHLIGHT_R * hiw;
        f[1] += SUPERIA_SHADOW_G * shw + SUPERIA_HIGHLIGHT_G * hiw;
        f[2] += SUPERIA_SHADOW_B * shw + SUPERIA_HIGHLIGHT_B * hiw;
      }
      if (doVign) {
        const float xx = ((float)x - cx) / cxn;
        const float d = (xx * xx + yy * yy) * 0.5f;
        const float vs = usmClampF(1.0f - SUPERIA_VIGNETTE * usmLutGet(usmLutVign, d), 0.0f, 1.0f);
        f[0] *= vs; f[1] *= vs; f[2] *= vs;
      }
      px[(size_t)y * w + x] = usmPack565(usmToU8(f[0] * 255.0f),
                                        usmToU8(f[1] * 255.0f),
                                        usmToU8(f[2] * 255.0f));
    }
  }

  // ---- sharpen: separable gaussian unsharp per channel ----------------------
  // A full-frame blur plane would cost another 2.6 MB of PSRAM, and the first
  // implementation kept a ~0.67 MB banded temp buffer instead -- which pushed
  // peak allocation to ~3.6 MB, past what this board's PSRAM actually answers,
  // and the captures came out as block-garbage colour chaos. So the blur now
  // keeps only the vertical kernel window: 2*rad+1 horizontally-blurred rows
  // (~54 KB at SXGA, rad = 3) in a ring. Row y is unsharpened against rows
  // y-rad..y+rad of that ring -- identical math to the full-frame separable
  // gaussian with edge replication, without the second plane.

  if (SUPERIA_USM_AMOUNT > 0 && SUPERIA_USM_SIGMA > 0) {
    int rad = (int)ceilf(3.0f * SUPERIA_USM_SIGMA);
    if (rad < 1) rad = 1;
    if (rad > SUPERIA_USM_MAX_RADIUS) rad = SUPERIA_USM_MAX_RADIUS;
    const int taps = 2 * rad + 1;
    float kern[SUPERIA_USM_MAX_RADIUS * 2 + 1];
    float ksum = 0.0f;
    for (int d = -rad; d <= rad; d++) {
      const float v = expf(-0.5f * (float)(d * d) * (1.0f / (SUPERIA_USM_SIGMA * SUPERIA_USM_SIGMA)));
      kern[d + rad] = v;
      ksum += v;
    }
    for (int d = 0; d < taps; d++) kern[d] /= ksum;

    const int ringRows = 2 * rad + 1;
    uint16_t* ring = (uint16_t*)ps_malloc(3u * (size_t)w * (size_t)ringRows * sizeof(uint16_t));
    uint8_t* row8 = (uint8_t*)ps_malloc(3u * (size_t)w);
    if (!ring || !row8) {
      if (ring) free(ring);
      if (row8) free(row8);
      free(canvas);
      return false;
    }

    // Horizontal blur of canvas row ysrc into ring slot ysrc % ringRows
    // (Q6, unpacked channels; ysrc outside the frame replicates row 0/h-1).
    auto hblurRow = [&](int ysrc) {
      const int yc = ysrc < 0 ? 0 : (ysrc > h - 1 ? h - 1 : ysrc);
      const uint16_t* src = px + (size_t)yc * w;
      uint16_t* dst = ring + (size_t)((ysrc % ringRows + ringRows) % ringRows) * w * 3;
      for (uint16_t x = 0; x < w; x++) {
        uint8_t rgb[3];
        usmUnpack565(src[x], rgb);
        row8[x * 3 + 0] = rgb[0];
        row8[x * 3 + 1] = rgb[1];
        row8[x * 3 + 2] = rgb[2];
      }
      for (uint16_t x = 0; x < w; x++) {
        for (int c = 0; c < 3; c++) {
          float acc = 0.0f;
          for (int d = -rad; d <= rad; d++) {
            int xi = (int)x + d;
            if (xi < 0) xi = 0;
            if (xi > w - 1) xi = w - 1;
            acc += kern[d + rad] * (float)row8[xi * 3 + c];
          }
          dst[x * 3 + c] = (uint16_t)(acc * 64.0f + 0.5f);  // Q6
        }
      }
    };

    for (int y = -rad; y <= rad; y++) hblurRow(y);  // prime the ring window

    for (int y = 0; y < h; y++) {
      // vertical pass over the ring window + unsharp into canvas row y
      uint16_t* prow = px + (size_t)y * w;
      for (uint16_t x = 0; x < w; x++) {
        uint8_t rgb[3];
        usmUnpack565(prow[x], rgb);
        uint8_t out[3];
        for (int c = 0; c < 3; c++) {
          float acc = 0.0f;
          for (int d = -rad; d <= rad; d++) {
            const uint16_t* rrow = ring
                + (size_t)(((y + d) % ringRows + ringRows) % ringRows) * w * 3;
            acc += kern[d + rad] * ((float)rrow[x * 3 + c] / 64.0f);
          }
          const float blurred = acc;  // vertical gaussian of canvas row y
          out[c] = usmToU8((float)rgb[c] + SUPERIA_USM_AMOUNT * ((float)rgb[c] - blurred));
        }
        prow[x] = usmPack565(out[0], out[1], out[2]);
      }
      hblurRow(y + rad + 1);  // slide the window; overwrites row y-rad's slot
    }
    free(ring);
    free(row8);
  }

  // ---- encode ----------------------------------------------------------------

  // Byte-order boundary. fmt2jpg_cb() reads RGB565 input unconditionally
  // big-endian (its convert_line_format takes R from the first byte), while
  // the canvas is little-endian standard R5G6B5. One swap pass brings the
  // processed canvas to the encoder's byte order -- the same swap the white
  // border filter does before its encode.
  for (size_t i = 0; i < n; i++) {
    px[i] = (uint16_t)((px[i] >> 8) | (px[i] << 8));
  }

  SuperiaJpgOutCtx out = { write, arg, 0, false };
  const bool encoded = fmt2jpg_cb(canvas, canvasBytes, w, h, PIXFORMAT_RGB565,
                                  quality, superiaJpgOutCb, &out);
  free(canvas);

  *written = out.written;
  *outW = w;
  *outH = h;
  return encoded && !out.failed && out.written > 0;
}

#else
#error "JPEG_FILTER must be JPEG_FILTER_NONE, JPEG_FILTER_WHITE_BORDER or JPEG_FILTER_SUPERIA_USM"
#endif
