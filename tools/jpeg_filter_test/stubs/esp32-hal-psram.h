#pragma once
#include <stdlib.h>
// On device this allocates from external PSRAM; the host test run only needs
// to observe the filter math, not allocation-failure behaviour, so plain
// malloc is the honest stand-in (host RAM has no 4 MB ceiling).
static inline void *ps_malloc(size_t n) { return malloc(n); }
