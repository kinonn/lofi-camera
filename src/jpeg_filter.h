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
//
// Switch in platformio.ini:
//   build_flags = -DJPEG_FILTER=JPEG_FILTER_WHITE_BORDER
// ---------------------------------------------------------------------------
#define JPEG_FILTER_NONE          0
#define JPEG_FILTER_WHITE_BORDER  1

#ifndef JPEG_FILTER
#define JPEG_FILTER JPEG_FILTER_NONE
#endif

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
static inline size_t jpegFilterWorstCaseBytes(const camera_fb_t* fb) {
  return fb->len;  // exact: nothing is re-encoded
}

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

// Upper bound on the produced size, for the free-space check before a file is
// opened. 0.25 bytes/pixel is about twice the worst bits-per-pixel measured
// on this sensor, so it is safe without being wildly pessimistic.
static inline size_t jpegFilterWorstCaseBytes(const camera_fb_t* fb) {
  return ((size_t)fb->width * (size_t)fb->height) / 4;
}

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
  // jpg2rgb565() emits LCD-oriented byte order; swap if colours come out wrong.
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

#else
#error "JPEG_FILTER must be JPEG_FILTER_NONE or JPEG_FILTER_WHITE_BORDER"
#endif
