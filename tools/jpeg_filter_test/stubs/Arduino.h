#pragma once
// Host stand-in for Arduino.h -- the firmware's JPEG filter branch uses no
// Arduino API beyond stdint/stddef types, so provide those.
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
