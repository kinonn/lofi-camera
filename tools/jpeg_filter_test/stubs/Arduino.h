#pragma once
// Host stand-in for Arduino.h -- the firmware's JPEG filter branch uses no
// Arduino API beyond stdint/stddef types, so provide those.
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// The filter records per-stage timings for the EXIF header even when the
// CAPTURE_TIMING log is off; the host run has no clock, so zero is fine.
static inline unsigned long millis() { return 0; }
static inline unsigned long micros() { return 0; }
