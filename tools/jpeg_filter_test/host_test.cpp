// Host test driver for the firmware capture filter.
//
// Compiles the REAL src/jpeg_filter.h with thin host stubs (stubs/), so what
// runs here is the device's filter math, its LUTs, its RGB565 canvas and its
// byte order -- not a reimplementation.
//
// FILTER_KIND selects the runtime look/border pair:
//   0  None                      (pass-through)
//   1  None + border             (WITH_BORDER)
//   2  Superia 400
//   3  Kodachrome 64
//   4  T-Max 400
//   5  Superia 400 + border
//   6  Kodachrome 64 + border
//   7  T-Max 400 + border
// FILTER_MODE selects which part of the chain is active, by the same -D
// macros the firmware exposes (see platformio.ini):
//   0  as shipped            look + USM sharpen
//   1  look only             -D..._USM_AMOUNT=0 (Superia)
//   2  USM only              balance/contrast/WB/gamma/tint/vignette switched
//                            off by zeroing their macros (Superia)
//   3  T-Max, S-curve off    -DTMAX_TONE_GAIN=1 -DTMAX_HIGHLIGHT_ROLLOFF=0,
//                            i.e. the pre-S-curve look, for an A/B against 0
//
// Usage: host_test <in.jpg> <out.jpg> [quality]
// quality applies to the re-encode like on device -- default 10, the
// firmware's JPEG_QUALITY. Output JPEG codec is stb, the device's is jpge;
// see run_filter_test.py for the honest checklist of what this does and
// does not reproduce.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FILTER_MODE
#define FILTER_MODE 0
#endif
#ifndef FILTER_KIND
#define FILTER_KIND 0
#endif
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
#elif FILTER_MODE == 3
// The T-Max look with the S-curve's two stages at identity, so mode 0 and mode
// 3 differ only by the curve (same look, same border, same sharpen).
#define TMAX_TONE_GAIN 1.0f
#define TMAX_HIGHLIGHT_ROLLOFF 0.0f
#endif
#include "jpeg_filter.h"

// FILTER_KIND is the test matrix's fixed numbering; the firmware itself uses
// the runtime look + border pair.
static void selectFilter(int kind) {
  FilterLook look = FILTER_LOOK_NONE;
  bool border = false;
  switch (kind) {
    case 0: look = FILTER_LOOK_NONE;       border = false; break;
    case 1: look = FILTER_LOOK_NONE;       border = true;  break;
    case 2: look = FILTER_LOOK_SUPERIA;    border = false; break;
    case 3: look = FILTER_LOOK_KODACHROME; border = false; break;
    case 4: look = FILTER_LOOK_TMAX;       border = false; break;
    case 5: look = FILTER_LOOK_SUPERIA;    border = true;  break;
    case 6: look = FILTER_LOOK_KODACHROME; border = true;  break;
    case 7: look = FILTER_LOOK_TMAX;       border = true;  break;
    default: break;
  }
  jpegFilterSetLook(look);
  jpegFilterSetBorder(border);
}

// The button-driven part of the runtime API: cycle order and border toggle.
static void selfCheck() {
  jpegFilterSetLook(FILTER_LOOK_NONE);
  jpegFilterSetBorder(false);
  assert(jpegFilterNextLook() == FILTER_LOOK_SUPERIA);
  assert(jpegFilterNextLook() == FILTER_LOOK_KODACHROME);
  assert(jpegFilterNextLook() == FILTER_LOOK_TMAX);
  assert(jpegFilterNextLook() == FILTER_LOOK_NONE);
  assert(jpegFilterToggleBorder() == true);
  assert(jpegFilterToggleBorder() == false);
}

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
  selfCheck();
  selectFilter(FILTER_KIND);

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

  // The harness exits 0 only if the output shape and the pass-through contract
  // are right; otherwise a corrupted border or pass-through would go unnoticed.
  const int border = (FILTER_KIND == 1 || FILTER_KIND >= 5)
                         ? JPEG_FILTER_INSET_PX
                         : 0;
  const uint16_t wantW = (uint16_t)(fb.width + 2 * border);
  const uint16_t wantH = (uint16_t)(fb.height + 2 * border);
  bool pass = ok && written == sink.n && sink.n > 0 && ow == wantW &&
              oh == wantH;
  if (pass && FILTER_KIND == 0) {
    FILE *fc = fopen(argv[2], "rb");
    if (!fc) {
      pass = false;
    } else {
      uint8_t *buf = (uint8_t *)malloc(fb.len);
      pass = buf && fread(buf, 1, fb.len, fc) == fb.len &&
             fgetc(fc) == EOF && memcmp(buf, fb.buf, fb.len) == 0;
      free(buf);
      fclose(fc);
    }
  }
  free(fb.buf);

  if (!pass) {
    fprintf(stderr, "FAIL: expected %ux%u, got %ux%u (%zu bytes)\n", wantW,
            wantH, ow, oh, sink.n);
  }
  printf("%s %s -> %s %ux%u %zu bytes\n", pass ? "OK " : "FAIL", argv[1],
         argv[2], ow, oh, sink.n);
  return pass ? 0 : 1;
}
