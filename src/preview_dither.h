#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "preview_types.h"

// Ordered-dither renderer (the original behaviour).
//
// Each pixel is compared against a 4x4 Bayer threshold so the 1-bit panel
// reproduces mid-tones as a stable dot pattern. Cheap: one lookup and one
// comparison per pixel, no intermediate buffers.

// The sensor runs one neutral stream for both the viewfinder and the stills,
// so the contrast a 1-bit panel wants is applied here instead of at the
// sensor: a linear stretch around mid-grey. 100 = unchanged.
#ifndef DITHER_CONTRAST
#define DITHER_CONTRAST 130
#endif

static const uint8_t BAYER_4X4[16] = {
  0,  8,  2, 10,
  12, 4, 14, 6,
  3, 11,  1,  9,
  15, 7, 13,  5,
};

inline void ditherDraw(Adafruit_SSD1306& disp, const PreviewFrame& f) {
  for (int y = 0; y < f.h; y++) {
    const uint8_t* row = f.px + y * f.w;
    for (int x = 0; x < f.w; x++) {
      // Bayer value 0-15 scaled to 0-255, offset by half a step.
      uint8_t thr = (uint8_t)(BAYER_4X4[((y & 3) << 2) | (x & 3)] * 16 + 8);
      int v = 128 + ((int)row[x] - 128) * DITHER_CONTRAST / 100;
      if (v < 0) v = 0; else if (v > 255) v = 255;
      if (v > thr) disp.drawPixel(f.x + x, f.y + y, SSD1306_WHITE);
    }
  }
}
