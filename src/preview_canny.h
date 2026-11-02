#pragma once
#include <string.h>
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "preview_types.h"

// Canny edge-detection renderer.
//
//   blur (optional) -> Sobel gradient -> non-maximum suppression
//   -> double threshold -> hysteresis -> draw surviving edges
//
// Tuned for the tiny 128x64 preview, so magnitudes use the cheap |gx|+|gy|
// approximation instead of a square root. All buffers are static: the
// renderer never touches the heap.
//
// Tunables (override with -D at build time):
//   CANNY_BLUR  1 = 3x3 binomial pre-blur (KILLS noise, costs ~2x time)
//   CANNY_BLUR_PASSES  repetitions of a 3x3 binomial blur (0 = off). Each extra
//                      pass widens the effective kernel, so skin texture and
//                      sensor noise stop registering as edges while genuine
//                      object outlines survive.
//   CANNY_LOW          weak-edge threshold on the NMS magnitude
//   CANNY_HIGH         strong-edge threshold; weak edges survive only if they
//                      touch a strong one. This is the density knob: raise it
//                      for sparser, outline-only output.
#ifndef CANNY_BLUR_PASSES
#define CANNY_BLUR_PASSES 2
#endif
#ifndef CANNY_LOW
#define CANNY_LOW 80
#endif
#ifndef CANNY_HIGH
#define CANNY_HIGH 160
#endif

#define CANNY_MAX_PX (PREVIEW_MAX_W * PREVIEW_MAX_H)

inline void cannyDraw(Adafruit_SSD1306& disp, const PreviewFrame& f) {
  const int w = f.w;
  const int h = f.h;
  if (w <= 0 || h <= 0) return;

  // 8-bit blur ping-pong buffers, 16-bit packed gradient (magnitude << 2 |
  // direction), classification map, and the hysteresis work stack.
  static uint8_t blurA[CANNY_MAX_PX];
  static uint8_t blurB[CANNY_MAX_PX];
  static uint16_t grad[CANNY_MAX_PX];
  static uint8_t edge[CANNY_MAX_PX];
  static uint16_t stack[CANNY_MAX_PX];

  // Edge-clamped fetch: keeps the border maths simple and branch-free-ish.
  auto at = [&](const uint8_t* buf, int x, int y) -> int {
    if (x < 0) x = 0; else if (x >= w) x = w - 1;
    if (y < 0) y = 0; else if (y >= h) y = h - 1;
    return buf[y * w + x];
  };

  const uint8_t* src = f.px;

#if CANNY_BLUR_PASSES > 0
  const uint8_t* in = f.px;
  uint8_t* out = blurA;
  uint8_t* spare = blurB;
  for (int pass = 0; pass < CANNY_BLUR_PASSES; pass++) {
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int acc = at(in, x - 1, y - 1) + 2 * at(in, x, y - 1) + at(in, x + 1, y - 1)
                + 2 * at(in, x - 1, y)   + 4 * at(in, x, y)   + 2 * at(in, x + 1, y)
                + at(in, x - 1, y + 1) + 2 * at(in, x, y + 1) + at(in, x + 1, y + 1);
        out[y * w + x] = (uint8_t)(acc / 16);
      }
    }
    in = out;
    uint8_t* swap = out;
    out = spare;
    spare = swap;
  }
  src = in;
#endif

  // --- Sobel gradient + quantised direction (0=0deg, 1=45, 2=90, 3=135) ---
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int gx = -at(src, x - 1, y - 1) + at(src, x + 1, y - 1)
             - 2 * at(src, x - 1, y) + 2 * at(src, x + 1, y)
               - at(src, x - 1, y + 1) + at(src, x + 1, y + 1);
      int gy = -at(src, x - 1, y - 1) - 2 * at(src, x, y - 1) - at(src, x + 1, y - 1)
             + at(src, x - 1, y + 1) + 2 * at(src, x, y + 1) + at(src, x + 1, y + 1);

            int mag = abs(gx) + abs(gy);          // max 1530 for 8-bit input

      const int ax = abs(gx);
      const int ay = abs(gy);
      uint8_t dir;
      if (ay * 5 < ax)      dir = 0;        // gradient mostly horizontal
      else if (ax * 5 < ay) dir = 2;        // gradient mostly vertical
      else if ((gx ^ gy) >= 0) dir = 1;     // same sign -> 45 degrees
      else                  dir = 3;        // opposite sign -> 135 degrees

      grad[y * w + x] = (uint16_t)((mag << 2) | dir);
    }
  }

  // --- Non-maximum suppression, classified straight into `edge` ---
  // `edge` is static, so it MUST be cleared every frame: suppressed pixels
  // skip the classification below, and leftover values would otherwise
  // accumulate until the whole panel is white.
  memset(edge, 0, (size_t)w * (size_t)h);

  int top = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const int i = y * w + x;
      const uint16_t g = grad[i];
      const int m = g >> 2;
      const uint8_t dir = g & 3;

      int a, b;
      switch (dir) {
        case 0:  // compare left / right
          a = grad[y * w + (x > 0 ? x - 1 : x)] >> 2;
          b = grad[y * w + (x + 1 < w ? x + 1 : x)] >> 2;
          break;
        case 1:  // compare along the 45 degree diagonal
          a = (x > 0 && y > 0) ? (grad[(y - 1) * w + x - 1] >> 2) : 0;
          b = (x + 1 < w && y + 1 < h) ? (grad[(y + 1) * w + x + 1] >> 2) : 0;
          break;
        case 2:  // compare up / down
          a = (y > 0) ? (grad[(y - 1) * w + x] >> 2) : 0;
          b = (y + 1 < h) ? (grad[(y + 1) * w + x] >> 2) : 0;
          break;
        default:  // compare along the 135 degree diagonal
          a = (x + 1 < w && y > 0) ? (grad[(y - 1) * w + x + 1] >> 2) : 0;
          b = (x > 0 && y + 1 < h) ? (grad[(y + 1) * w + x - 1] >> 2) : 0;
          break;
      }

      if (m <= a || m < b) continue; // break plateaus consistently on one side

      if (m >= CANNY_HIGH) {
        edge[i] = 2;                  // strong
        if (top < CANNY_MAX_PX) stack[top++] = (uint16_t)i;
      } else if (m >= CANNY_LOW) {
        edge[i] = 1;                  // weak, may be promoted
      }
    }
  }

  // --- Hysteresis: promote weak edges reachable from a strong one ---
  while (top > 0) {
    const int i = stack[--top];
    const int x = i % w;
    const int y = i / w;
    for (int dy = -1; dy <= 1; dy++) {
      const int ny = y + dy;
      if (ny < 0 || ny >= h) continue;
      for (int dx = -1; dx <= 1; dx++) {
        const int nx = x + dx;
        if (nx < 0 || nx >= w) continue;
        const int j = ny * w + nx;
        if (edge[j] == 1) {
          edge[j] = 2;
          if (top < CANNY_MAX_PX) stack[top++] = (uint16_t)j;
        }
      }
    }
  }

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      if (edge[y * w + x] == 2) disp.drawPixel(f.x + x, f.y + y, SSD1306_WHITE);
    }
  }
}
