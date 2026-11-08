#pragma once
// Minimal stand-in for the esp32-camera driver types host_filter_test needs.
typedef enum {
  PIXFORMAT_NONE,
  PIXFORMAT_GRAYSCALE,
  PIXFORMAT_RGB565,
  PIXFORMAT_YUV422,
  PIXFORMAT_RGB888,
  PIXFORMAT_JPEG,
} pixformat_t;

typedef struct {
  uint8_t *buf;
  size_t len;
  size_t width;
  size_t height;
  pixformat_t format;
} camera_fb_t;
