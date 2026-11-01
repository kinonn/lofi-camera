#pragma once
#include <Arduino.h>

// Largest preview the OLED can show. Dictates the scratch-buffer size used by
// every preview renderer.
#define PREVIEW_MAX_W 128
#define PREVIEW_MAX_H 64

// An aspect-fit, letterboxed downsample of one camera frame, ready for a
// preview renderer. This is the only thing renderers get, so they are free of
// camera/display details and easy to swap.
struct PreviewFrame {
  uint8_t px[PREVIEW_MAX_W * PREVIEW_MAX_H];  // w * h grayscale pixels
  int w;                                      // image width in pixels
  int h;                                      // image height in pixels
  int x;                                      // left offset on the display
  int y;                                      // top offset on the display
};
