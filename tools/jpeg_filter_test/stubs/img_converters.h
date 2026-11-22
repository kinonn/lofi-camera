#pragma once
// Host stand-in for img_converters.h: supplies the two ESP32-only functions
// src/jpeg_filter.h calls, implemented with stb so the REAL firmware filter
// code runs unmodified on a Linux host.
//
// Byte order: the firmware pipeline expects the decoder canvas in native
// little-endian standard R5G6B5, then swaps once before encoding so
// fmt2jpg_cb() (which reads RGB565 big-endian) sees the right channels. This
// stub mirrors that: decode writes the little-endian value unswapped, the
// caller's final swap produces big-endian bytes, and fmt2jpg_cb() reads them
// back big-endian. Do not enable the legacy SWAP565 stub switch; the firmware
// now performs that swap unconditionally (including WITH_BORDER).
#include "esp_camera.h"
#include "esp_jpg_decode.h"

#define STBI_ONLY_JPEG
#define STB_IMAGE_IMPLEMENTATION
#include "../stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

typedef size_t (*jpg_out_cb)(void *arg, size_t index, const void *data, size_t len);

// Decode JPEG -> u16 canvas, packed + stored bytes exactly like the device:
// truncate to 5/6/5 (no bit replication); the firmware does its own final
// swap before fmt2jpg_cb().
static inline uint16_t stub565Pack(const uint8_t *rgb) {
  return (uint16_t)(((unsigned)(rgb[0] >> 3) << 11)
                  | ((unsigned)(rgb[1] >> 2) << 5)
                  | (unsigned)(rgb[2] >> 3));
}

static inline bool jpg2rgb565(const uint8_t *src, size_t src_len, uint8_t *out,
                              jpg_scale_t scale) {
  (void)scale;
  int w, h, comp;
  uint8_t *rgb = stbi_load_from_memory(src, (int)src_len, &w, &h, &comp, 3);
  if (!rgb) return false;
  uint16_t *px = (uint16_t *)out;
  for (size_t i = 0, n = (size_t)w * h; i < n; i++) {
    px[i] = stub565Pack(rgb + i * 3);
  }
  stbi_image_free(rgb);
  return true;
}

struct StubEncCtx {
  jpg_out_cb cb;
  void *arg;
  size_t index;
};

static void stubEncOut(void *ctx, void *data, int size) {
  StubEncCtx *c = (StubEncCtx *)ctx;
  c->index += c->cb(c->arg, c->index, data, (size_t)size);
}

// Re-encode the canvas to JPEG exactly the way convert_line_format() reads a
// big-endian RGB565 canvas on device (same truncating bit expansion), then
// stream the result through the caller's jpg_out_cb like fmt2jpg_cb().
static inline bool fmt2jpg_cb(uint8_t *src, size_t src_len, uint16_t width,
                              uint16_t height, pixformat_t format,
                              uint8_t quality, jpg_out_cb cb, void *arg) {
  (void)src_len;
  (void)format;
  if (quality < 1) quality = 1;
  if (quality > 100) quality = 100;

  uint8_t *rgb = (uint8_t *)malloc((size_t)width * height * 3);
  if (!rgb) return false;
  const uint8_t *s = src;
  uint8_t *d = rgb;
  for (size_t i = 0, n = (size_t)width * height; i < n; i++, s += 2, d += 3) {
    d[0] = s[0] & 0xF8;
    d[1] = (uint8_t)(((s[0] & 0x07) << 5) | ((s[1] & 0xE0) >> 3));
    d[2] = (uint8_t)((s[1] & 0x1F) << 3);
  }

  StubEncCtx ctx = { cb, arg, 0 };
  const bool ok = stbi_write_jpg_to_func(stubEncOut, &ctx, width, height, 3,
                                         rgb, quality);
  free(rgb);
  return ok;
}
