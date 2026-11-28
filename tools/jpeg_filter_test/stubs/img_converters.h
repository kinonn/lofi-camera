#pragma once
// Host stand-in for img_converters.h: supplies the two ESP32-only functions
// src/jpeg_filter.h calls, implemented with stb so the REAL firmware filter
// code runs unmodified on a Linux host.
//
// Byte order: the canvas is RGB888 and both the decoder and the encoder read
// R,G,B in natural byte order, so no swap is involved on either side.
#include "esp_camera.h"
#include "esp_jpg_decode.h"
#include <string.h>

#define STBI_ONLY_JPEG
#define STB_IMAGE_IMPLEMENTATION
#include "../stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

typedef size_t (*jpg_out_cb)(void *arg, size_t index, const void *data, size_t len);

// Decode JPEG -> 3 bytes/pixel RGB canvas, exactly like the device's helper.
static inline bool fmt2rgb888(const uint8_t *src, size_t src_len,
                              pixformat_t format, uint8_t *out) {
  if (format != PIXFORMAT_JPEG) return false;
  int w, h, comp;
  uint8_t *rgb = stbi_load_from_memory(src, (int)src_len, &w, &h, &comp, 3);
  if (!rgb) return false;
  memcpy(out, rgb, (size_t)w * h * 3);
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

// Re-encode the canvas and stream the result through the caller's jpg_out_cb
// like fmt2jpg_cb(). RGB888 rows are passed through as-is (the device's
// convert_line_format copies them byte for byte).
static inline bool fmt2jpg_cb(uint8_t *src, size_t src_len, uint16_t width,
                              uint16_t height, pixformat_t format,
                              uint8_t quality, jpg_out_cb cb, void *arg) {
  (void)src_len;
  if (format != PIXFORMAT_RGB888) return false;
  if (quality < 1) quality = 1;
  if (quality > 100) quality = 100;

  StubEncCtx ctx = { cb, arg, 0 };
  const bool ok = stbi_write_jpg_to_func(stubEncOut, &ctx, width, height, 3,
                                         src, quality);
  return ok;
}
