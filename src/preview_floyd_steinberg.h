#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "preview_types.h"

// Serpentine Floyd-Steinberg renderer. Two scanline error buffers are enough
// because each pixel only diffuses quantisation error to later pixels; this
// avoids a full-frame copy and any per-frame heap allocation.

// The sensor runs one neutral stream for both the viewfinder and the stills,
// so the contrast a 1-bit panel wants is applied here instead of at the
// sensor: a linear stretch around mid-grey. 100 = unchanged.
#ifndef DITHER_CONTRAST
#define DITHER_CONTRAST 130
#endif

#ifndef FLOYD_STEINBERG_THRESHOLD
#define FLOYD_STEINBERG_THRESHOLD 128
#endif

#ifndef FLOYD_STEINBERG_STRENGTH
#define FLOYD_STEINBERG_STRENGTH 0.45f
#endif

static_assert(FLOYD_STEINBERG_THRESHOLD >= 0 &&
                  FLOYD_STEINBERG_THRESHOLD <= 255,
              "FLOYD_STEINBERG_THRESHOLD must be in 0..255");
static_assert(FLOYD_STEINBERG_STRENGTH >= 0.0f &&
                  FLOYD_STEINBERG_STRENGTH <= 1.0f,
              "FLOYD_STEINBERG_STRENGTH must be in 0..1");

inline void floydSteinbergDraw(Adafruit_SSD1306& disp,
                               const PreviewFrame& f) {
  static float errorA[PREVIEW_MAX_W];
  static float errorB[PREVIEW_MAX_W];
  if (f.w <= 0 || f.h <= 0) return;

  float* currentError = errorA;
  float* nextError = errorB;
  for (int x = 0; x < f.w; x++) {
    currentError[x] = 0.0f;
    nextError[x] = 0.0f;
  }

  for (int y = 0; y < f.h; y++) {
    const uint8_t* row = f.px + y * f.w;
    const bool leftToRight = (y & 1) == 0;
    const int step = leftToRight ? 1 : -1;
    int x = leftToRight ? 0 : f.w - 1;

    for (int i = 0; i < f.w; i++, x += step) {
      int adjusted = 128 + ((int)row[x] - 128) * DITHER_CONTRAST / 100;
      if (adjusted < 0) adjusted = 0;
      else if (adjusted > 255) adjusted = 255;

      const float oldValue = (float)adjusted + currentError[x];
      currentError[x] = 0.0f;
      const float quantized =
          oldValue >= FLOYD_STEINBERG_THRESHOLD ? 255.0f : 0.0f;
      const float error = (oldValue - quantized) * FLOYD_STEINBERG_STRENGTH;
      if (quantized != 0.0f) disp.drawPixel(f.x + x, f.y + y, SSD1306_WHITE);

      const int forwardX = x + step;
      if (forwardX >= 0 && forwardX < f.w) {
        currentError[forwardX] += error * (7.0f / 16.0f);
      }

      if (y + 1 < f.h) {
        const int downBackX = x - step;
        const int downForwardX = x + step;
        if (downBackX >= 0 && downBackX < f.w) {
          nextError[downBackX] += error * (3.0f / 16.0f);
        }
        nextError[x] += error * (5.0f / 16.0f);
        if (downForwardX >= 0 && downForwardX < f.w) {
          nextError[downForwardX] += error * (1.0f / 16.0f);
        }
      }
    }

    float* swap = currentError;
    currentError = nextError;
    nextError = swap;
    for (int x = 0; x < f.w; x++) nextError[x] = 0.0f;
  }
}