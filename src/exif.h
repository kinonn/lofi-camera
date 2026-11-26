#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// APP1 EXIF header for every saved JPEG. The image data is never touched: the
// SD sink inserts this segment after SOI, and the description is patched in
// place after the encode because the encode timing is not known until then.
//
// The layout is deliberately fixed-size. EXIF_DESC_FILE_OFFSET is the byte
// offset of the description from the start of the file (2 bytes of SOI + the
// APP1 header/TIFF/IFD). Changing any header field changes that offset, so the
// static_asserts below are the contract between the builder and the patcher.
#ifndef EXIF_ENABLED
#define EXIF_ENABLED 1
#endif

static constexpr size_t EXIF_DESC_SIZE = 256;
static constexpr size_t EXIF_DESC_APP1_OFFSET = 36;  // from the FF E1 marker
static constexpr size_t EXIF_DESC_FILE_OFFSET = 2 + EXIF_DESC_APP1_OFFSET;
static constexpr size_t EXIF_APP1_BYTES =
    2 + 2 + 6 + 8 + 2 + 12 + 4 + EXIF_DESC_SIZE;  // marker, len, Exif, TIFF

static_assert(EXIF_APP1_BYTES == 292, "APP1 layout changed");
static_assert(EXIF_DESC_APP1_OFFSET == 2 + 2 + 6 + 8 + 2 + 12 + 4,
              "description offset changed");
static_assert(EXIF_DESC_APP1_OFFSET - 10 == 26,
              "ImageDescription TIFF value offset changed");

struct ExifInfo {
  const char* look;
  bool border;
  int inset;
  int srcW, srcH;
  int outW, outH;
  int quality;  // re-encode quality; 0 = pass-through
  size_t srcBytes;
  uint32_t fbMs, decodeMs, statsMs, lookMs, sharpenMs, encodeMs, totalMs;
};

// Build the fixed APP1 segment. The description area stays zeroed until the
// first write; the caller patches it with exifFormatDescription() afterwards.
static inline void exifBuild(uint8_t* out) {
  memset(out, 0, EXIF_APP1_BYTES);

  out[0] = 0xFF;  // APP1
  out[1] = 0xE1;
  const uint16_t segLen = (uint16_t)(EXIF_APP1_BYTES - 2);
  out[2] = (uint8_t)(segLen >> 8);  // JPEG segment lengths are big-endian
  out[3] = (uint8_t)(segLen & 0xFF);

  memcpy(out + 4, "Exif\0\0", 6);

  // Little-endian TIFF header, IFD0 at offset 8.
  const uint8_t tiff[8] = {'I', 'I', 0x2A, 0x00, 0x08, 0x00, 0x00, 0x00};
  memcpy(out + 10, tiff, sizeof(tiff));

  out[18] = 1;  // one IFD0 entry
  out[19] = 0;

  // ImageDescription (0x010E), ASCII, 256 bytes, value at TIFF offset 26.
  const uint16_t tag = 0x010E;
  const uint16_t type = 2;
  const uint32_t count = EXIF_DESC_SIZE;
  const uint32_t valueOffset = (uint32_t)(EXIF_DESC_APP1_OFFSET - 10);
  out[20] = (uint8_t)(tag & 0xFF);
  out[21] = (uint8_t)(tag >> 8);
  out[22] = (uint8_t)(type & 0xFF);
  out[23] = (uint8_t)(type >> 8);
  out[24] = (uint8_t)(count & 0xFF);
  out[25] = (uint8_t)((count >> 8) & 0xFF);
  out[26] = (uint8_t)((count >> 16) & 0xFF);
  out[27] = (uint8_t)((count >> 24) & 0xFF);
  out[28] = (uint8_t)(valueOffset & 0xFF);
  out[29] = (uint8_t)((valueOffset >> 8) & 0xFF);
  out[30] = (uint8_t)((valueOffset >> 16) & 0xFF);
  out[31] = (uint8_t)((valueOffset >> 24) & 0xFF);
  // out[32..35] is the next-IFD offset, already zero.
}

// Human-readable, machine-parseable one-liner. `look` may contain spaces, so
// it runs until the " border=" field; every other value is a single token.
static inline void exifFormatDescription(char* out, size_t cap,
                                         const ExifInfo& info) {
  if (cap == 0) {
    return;
  }
  memset(out, 0, cap);
  snprintf(out, cap,
           "lofi-camera look=%s border=%d inset=%d in=%dx%d out=%dx%d q=%d "
           "src=%lu fb_ms=%lu dec_ms=%lu stats_ms=%lu look_ms=%lu "
           "sharp_ms=%lu enc_ms=%lu total_ms=%lu",
           info.look ? info.look : "?", info.border ? 1 : 0, info.inset,
           info.srcW, info.srcH, info.outW, info.outH, info.quality,
           (unsigned long)info.srcBytes,
           (unsigned long)info.fbMs,
           (unsigned long)info.decodeMs,
           (unsigned long)info.statsMs,
           (unsigned long)info.lookMs,
           (unsigned long)info.sharpenMs,
           (unsigned long)info.encodeMs,
           (unsigned long)info.totalMs);
}

typedef size_t (*ExifWriteFn)(void* arg, const uint8_t* data, size_t len);

// Write the first JPEG chunk with the APP1 segment after SOI. `write` is the
// caller's sink, used for every part of the chunk. The return value counts
// only source bytes: the APP1 bytes are extra output, so jpegFilterStream's
// pass-through length check still sees the original JPEG length. A short
// write or a chunk that does not start with SOI sets *failed.
static inline size_t exifWriteFirst(ExifWriteFn write, void* arg,
                                    const uint8_t* data, size_t len,
                                    const uint8_t* app1, bool* failed) {
  if (len < 2 || data[0] != 0xFF || data[1] != 0xD8) {
    *failed = true;
    return 0;
  }
  if (write(arg, data, 2) != 2 ||
      write(arg, app1, EXIF_APP1_BYTES) != EXIF_APP1_BYTES) {
    *failed = true;
    return 0;
  }
  if (len == 2) {
    return 2;
  }
  const size_t n = write(arg, data + 2, len - 2);
  if (n != len - 2) {
    *failed = true;
    return 2 + n;
  }
  return len;
}
