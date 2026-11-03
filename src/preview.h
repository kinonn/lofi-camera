#pragma once
#include <math.h>
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "preview_types.h"

// ---------------------------------------------------------------------------
// OLED viewfinder image processing, selected at compile time.
//
// Select the supported renderer by setting PREVIEW_MODE in platformio.ini.
// Floyd-Steinberg is the only firmware renderer; unsupported selections fail
// below.
//
// This only affects what is drawn on the OLED. Saving full-resolution JPEGs
// to the SD card goes through a completely separate path and is unaffected.
// ---------------------------------------------------------------------------
#define PREVIEW_MODE_FLOYD_STEINBERG 1

#ifndef PREVIEW_MODE
#define PREVIEW_MODE PREVIEW_MODE_FLOYD_STEINBERG
#endif

#if PREVIEW_MODE != PREVIEW_MODE_FLOYD_STEINBERG
#error "PREVIEW_MODE must be PREVIEW_MODE_FLOYD_STEINBERG"
#endif

#include "preview_floyd_steinberg.h"

// Downscale `img` to fit the display while preserving aspect ratio, centring
// the result (letterbox). Nearest-neighbour is deliberate: the source is
// already close to the target size after the sensor's own binning, and it
// keeps per-frame cost predictable at ~30 fps.
inline void previewPrepare(PreviewFrame& f, const uint8_t* img,
                           int srcW, int srcH, int screenW, int screenH) {
  if (srcW <= 0 || srcH <= 0) {
    f.w = 0;
    f.h = 0;
    f.x = 0;
    f.y = 0;
    return;
  }

  const float scale = fminf((float)screenW / (float)srcW,
                            (float)screenH / (float)srcH);
  int dw = (int)(srcW * scale);
  int dh = (int)(srcH * scale);
  if (dw < 1) dw = 1;
  if (dh < 1) dh = 1;
  if (dw > PREVIEW_MAX_W) dw = PREVIEW_MAX_W;
  if (dh > PREVIEW_MAX_H) dh = PREVIEW_MAX_H;

  f.w = dw;
  f.h = dh;
  f.x = (screenW - dw) / 2;
  f.y = (screenH - dh) / 2;

  for (int y = 0; y < dh; y++) {
    const uint8_t* row = img + (y * srcH / dh) * srcW;
    uint8_t* dst = f.px + y * dw;
    for (int x = 0; x < dw; x++) {
      dst[x] = row[x * srcW / dw];
    }
  }
}

// Render a prepared frame using the selected mode. Clearing the display and
// pushing it to the panel stays with the caller, so renderers only draw.
inline void previewDraw(Adafruit_SSD1306& disp, const PreviewFrame& f) {
  floydSteinbergDraw(disp, f);
}
