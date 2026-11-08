// Host test driver for the firmware capture filter.
//
// Compiles the REAL src/jpeg_filter.h JPEG_FILTER_SUPERIA_USM branch with
// thin host stubs (stubs/), so what runs here is the device's filter math,
// its LUTs, its RGB565 canvas and its byte order -- not a reimplementation.
//
// FILTER_MODE selects which part of the chain is active, by the same -D
// macros the firmware exposes (see platformio.ini):
//   0  as shipped            Superia 400 look + USM sharpen
//   1  Superia look only     -DSUPERIA_USM_AMOUNT=0
//   2  USM only              balance/contrast/WB/gamma/tint/vignette switched
//                            off by zeroing their macros
//
// Usage: host_test <in.jpg> <out.jpg> [quality]
// quality applies to the re-encode like on device -- default 10, the
// firmware's JPEG_QUALITY. Output JPEG codec is stb, the device's is jpge;
// see run_filter_test.py for the honest checklist of what this does and
// does not reproduce.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FILTER_MODE
#define FILTER_MODE 0
#endif
#define JPEG_FILTER JPEG_FILTER_SUPERIA_USM
#if FILTER_MODE == 1
#define SUPERIA_USM_AMOUNT 0.0f
#elif FILTER_MODE == 2
#define SUPERIA_AUTO_BALANCE 0.0f
#define SUPERIA_AUTO_CONTRAST 0.0f
#define SUPERIA_WB_R 1.0f
#define SUPERIA_WB_G 1.0f
#define SUPERIA_WB_B 1.0f
#define SUPERIA_MIDTONE_GAMMA 1.0f
#define SUPERIA_SHADOW_R 0.0f
#define SUPERIA_SHADOW_G 0.0f
#define SUPERIA_SHADOW_B 0.0f
#define SUPERIA_HIGHLIGHT_R 0.0f
#define SUPERIA_HIGHLIGHT_G 0.0f
#define SUPERIA_HIGHLIGHT_B 0.0f
#define SUPERIA_VIGNETTE 0
#endif
#include "jpeg_filter.h"

// The firmware's SD file sink: stdout-equivalent on host.
struct FileSink {
  FILE *f;
  size_t n;
};

static size_t fileSinkWrite(void *arg, const uint8_t *data, size_t len) {
  FileSink *s = (FileSink *)arg;
  const size_t w = fwrite(data, 1, len, s->f);
  s->n += w;
  return w;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <in.jpg> <out.jpg> [quality]\n", argv[0]);
    return 2;
  }
  const uint8_t quality = argc > 3 ? (uint8_t)atoi(argv[3]) : 10;

  FILE *fin = fopen(argv[1], "rb");
  if (!fin) {
    fprintf(stderr, "cannot open %s\n", argv[1]);
    return 2;
  }
  fseek(fin, 0, SEEK_END);
  const long len = ftell(fin);
  fseek(fin, 0, SEEK_SET);

  camera_fb_t fb;
  fb.len = (size_t)len;
  fb.buf = (uint8_t *)malloc(fb.len);
  fb.format = PIXFORMAT_JPEG;
  if (!fb.buf || fread(fb.buf, 1, fb.len, fin) != fb.len) {
    fprintf(stderr, "read failed\n");
    return 2;
  }
  fclose(fin);

  // jpegFilterStream() sizes its canvas from fb.width/height, so read the
  // dims the same way main.cpp learns them from the sensor's frame.
  int w, h, comp;
  if (!stbi_info_from_memory(fb.buf, (int)fb.len, &w, &h, &comp)) {
    fprintf(stderr, "not a JPEG: %s\n", argv[1]);
    return 2;
  }
  fb.width = (size_t)w;
  fb.height = (size_t)h;

  FILE *fout = fopen(argv[2], "wb");
  if (!fout) {
    fprintf(stderr, "cannot write %s\n", argv[2]);
    return 2;
  }
  FileSink sink = { fout, 0 };
  size_t written;
  uint16_t ow, oh;
  const bool ok = jpegFilterStream(&fb, quality, fileSinkWrite, &sink,
                                   &written, &ow, &oh);
  fclose(fout);
  free(fb.buf);

  printf("%s %s -> %s %ux%u %zu bytes\n", ok ? "OK " : "FAIL", argv[1],
         argv[2], ow, oh, sink.n);
  return ok ? 0 : 1;
}
