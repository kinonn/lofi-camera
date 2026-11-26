// Host test for the APP1 EXIF segment built by src/exif.h.
//
// The firmware inserts this segment after the JPEG SOI and later patches the
// 256-byte ImageDescription in place with the final stage timings. The offsets
// asserted here are part of that contract: a wrong APP1 length or TIFF value
// offset makes the tag unreadable, and a wrong file offset corrupts the JPEG
// header on the in-place patch.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "exif.h"

struct MemSink {
  uint8_t buf[600];
  size_t n;
  size_t limit;
};

static size_t memWrite(void* arg, const uint8_t* data, size_t len) {
  MemSink* s = (MemSink*)arg;
  if (s->n >= s->limit) {
    len = 0;
  } else if (len > s->limit - s->n) {
    len = s->limit - s->n;
  }
  memcpy(s->buf + s->n, data, len);
  s->n += len;
  return len;
}

int main(int argc, char** argv) {
  static_assert(EXIF_APP1_BYTES == 292, "APP1 size changed");
  static_assert(EXIF_DESC_SIZE == 256, "description size changed");
  static_assert(EXIF_DESC_APP1_OFFSET == 36, "description offset changed");
  static_assert(EXIF_DESC_FILE_OFFSET == 38, "file patch offset changed");

  uint8_t segment[EXIF_APP1_BYTES];
  exifBuild(segment);

  // JPEG APP1 marker and big-endian segment length (length includes itself).
  assert(segment[0] == 0xFF && segment[1] == 0xE1);
  assert(((segment[2] << 8) | segment[3]) == EXIF_APP1_BYTES - 2);
  assert(memcmp(segment + 4, "Exif\0\0", 6) == 0);

  // Little-endian TIFF header and one IFD0 entry.
  const uint8_t tiff[8] = {'I', 'I', 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00};
  assert(memcmp(segment + 10, tiff, sizeof(tiff)) == 0);
  assert(segment[18] == 1 && segment[19] == 0);

  // ImageDescription: tag 0x010E, ASCII, 256 bytes, value at TIFF offset 26.
  assert(segment[20] == 0x0E && segment[21] == 0x01);
  assert(segment[22] == 2 && segment[23] == 0);
  assert(segment[24] == 0 && segment[25] == 1 && segment[26] == 0 &&
         segment[27] == 0);
  assert(segment[28] == 26 && segment[29] == 0 && segment[30] == 0 &&
         segment[31] == 0);

  // No next IFD, and the description area starts zeroed for the first write.
  assert(segment[32] == 0 && segment[33] == 0 && segment[34] == 0 &&
         segment[35] == 0);
  for (size_t i = 0; i < EXIF_DESC_SIZE; i++) {
    assert(segment[EXIF_DESC_APP1_OFFSET + i] == 0);
  }

  ExifInfo info = {};
  info.look = "T-Max 400";
  info.border = true;
  info.inset = 24;
  info.srcW = 1280;
  info.srcH = 1024;
  info.outW = 1328;
  info.outH = 1072;
  info.quality = 80;
  info.srcBytes = 312345;
  info.fbMs = 120;
  info.decodeMs = 900;
  info.statsMs = 350;
  info.lookMs = 850;
  info.sharpenMs = 420;
  info.encodeMs = 2610;
  info.totalMs = 5700;

  char desc[EXIF_DESC_SIZE];
  exifFormatDescription(desc, sizeof(desc), info);
  assert(strcmp(desc,
                "lofi-camera look=T-Max 400 border=1 inset=24 "
                "in=1280x1024 out=1328x1072 q=80 src=312345 "
                "fb_ms=120 dec_ms=900 stats_ms=350 look_ms=850 "
                "sharp_ms=420 enc_ms=2610 total_ms=5700") == 0);

  // Simulate the sink: SOI + APP1 + JPEG body, then patch the description.
  uint8_t file[2 + EXIF_APP1_BYTES + 16];
  file[0] = 0xFF;
  file[1] = 0xD8;
  memcpy(file + 2, segment, EXIF_APP1_BYTES);
  memset(file + 2 + EXIF_APP1_BYTES, 0xAA, 16);
  memcpy(file + EXIF_DESC_FILE_OFFSET, desc, EXIF_DESC_SIZE);
  assert(memcmp(file + EXIF_DESC_FILE_OFFSET, desc, strlen(desc)) == 0);
  assert(memcmp(file + 2 + EXIF_DESC_APP1_OFFSET, desc, strlen(desc)) == 0);

  // First-write contract: SOI, APP1, then the rest of the chunk; the return
  // value counts only source bytes so the filter's length checks stay valid.
  uint8_t chunk[64];
  chunk[0] = 0xFF;
  chunk[1] = 0xD8;
  for (int i = 2; i < (int)sizeof(chunk); i++) chunk[i] = (uint8_t)i;

  MemSink ms = {};
  ms.limit = sizeof(ms.buf);
  bool failed = false;
  assert(exifWriteFirst(memWrite, &ms, chunk, sizeof(chunk), segment,
                        &failed) == sizeof(chunk));
  assert(!failed);
  assert(ms.n == 2 + EXIF_APP1_BYTES + sizeof(chunk) - 2);
  assert(ms.buf[0] == 0xFF && ms.buf[1] == 0xD8);
  assert(memcmp(ms.buf + 2, segment, EXIF_APP1_BYTES) == 0);
  assert(memcmp(ms.buf + 2 + EXIF_APP1_BYTES, chunk + 2,
                sizeof(chunk) - 2) == 0);

  // A first chunk that is just SOI still gets the header and returns 2.
  MemSink ms2 = {};
  ms2.limit = sizeof(ms2.buf);
  failed = false;
  assert(exifWriteFirst(memWrite, &ms2, chunk, 2, segment, &failed) == 2);
  assert(!failed);
  assert(ms2.n == 2 + EXIF_APP1_BYTES);

  // Not a JPEG start: no output and a reported failure.
  MemSink ms3 = {};
  ms3.limit = sizeof(ms3.buf);
  failed = false;
  assert(exifWriteFirst(memWrite, &ms3, chunk + 1, 4, segment, &failed) == 0);
  assert(failed);
  assert(ms3.n == 0);

  // Short inner write: failure is reported instead of a silent partial file.
  MemSink ms4 = {};
  ms4.limit = 3;  // SOI + one byte of APP1
  failed = false;
  assert(exifWriteFirst(memWrite, &ms4, chunk, sizeof(chunk), segment,
                        &failed) == 0);
  assert(failed);

  // Optional real-JPEG check: insert the segment after SOI, write the file,
  // then patch the description at the same offset the firmware uses. The
  // Python harness reopens it with Pillow to prove a real parser sees it.
  if (argc >= 3) {
    FILE* in = fopen(argv[1], "rb");
    if (!in) {
      fprintf(stderr, "cannot read %s\n", argv[1]);
      return 2;
    }
    fseek(in, 0, SEEK_END);
    const long len = ftell(in);
    fseek(in, 0, SEEK_SET);
    uint8_t* jpeg = (uint8_t*)malloc((size_t)(len > 0 ? len : 0));
    if (!jpeg || len < 2 || fread(jpeg, 1, (size_t)len, in) != (size_t)len) {
      fprintf(stderr, "cannot read %s\n", argv[1]);
      fclose(in);
      free(jpeg);
      return 2;
    }
    fclose(in);
    if (jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
      fprintf(stderr, "%s is not a JPEG\n", argv[1]);
      free(jpeg);
      return 2;
    }

    FILE* out = fopen(argv[2], "wb");
    if (!out || fwrite(jpeg, 1, 2, out) != 2 ||
        fwrite(segment, 1, sizeof(segment), out) != sizeof(segment) ||
        fwrite(jpeg + 2, 1, (size_t)len - 2, out) != (size_t)len - 2) {
      fprintf(stderr, "cannot write %s\n", argv[2]);
      if (out) fclose(out);
      free(jpeg);
      return 2;
    }
    fclose(out);
    free(jpeg);

    FILE* patch = fopen(argv[2], "r+b");
    if (!patch || fseek(patch, EXIF_DESC_FILE_OFFSET, SEEK_SET) != 0 ||
        fwrite(desc, 1, sizeof(desc), patch) != sizeof(desc)) {
      fprintf(stderr, "cannot patch %s\n", argv[2]);
      if (patch) fclose(patch);
      return 2;
    }
    fclose(patch);
  }

  printf("EXIF OK\n");
  return 0;
}
