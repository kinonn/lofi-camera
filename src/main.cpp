#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include "preview.h"
#include "jpeg_filter.h"
#include "img_converters.h"
#include "esp32-hal-psram.h"
#include "esp_system.h"

// ---------- User config ----------
// Verified pin map (AI Thinker ESP32-CAM, SD in 1-bit mode):
//   Camera (fixed, do not reuse): D0-D7 = 5,18,19,21,36,39,34,35;
//     XCLK=0, PCLK=22, VSYNC=25, HREF=23, SCCB SDA=26, SCL=27, PWDN=32.
//   SD_MMC 1-bit (fixed): CLK=14, CMD=15, DATA0=2. (GPIO4/12/13 freed
//     by 1-bit mode; GPIO4 left alone to avoid flash-LED interference.)
//   DO NOT USE: GPIO16 (PSRAM chip-select, crashes after photos),
//     GPIO33 (internal red LED only, NOT on the header),
//     GPIO0/5/18/19/21/22/23/25/26/27/32/34/35/36/39 (camera),
//     GPIO2/14/15 (SD), GPIO12 (MTDI strapping, must be LOW at boot),
//     GPIO4 (white flash LED -- avoided for clean OLED I2C).
//   Wiring (use silkscreen labels on the header):
//   OLED SDA -> IO13, OLED SCL -> U0T (GPIO1), VCC -> 3V3, GND -> GND.
//     GPIO1 is the serial TX line: Serial.print debug output is
//     disabled in this config, and the OLED must be disconnected
//     for upload via the MB adapter.
//   Button: one side -> U0R (GPIO3), other side -> GND (internal
//     pull-up, active LOW; 1k series resistor recommended while the
//     MB adapter is attached). Do not press during upload.
//   SD card: FAT32 microSD in the onboard slot.
#define OLED_SDA 13
#define OLED_SCL 1
#define BUTTON_PIN 3
#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Serial TX is GPIO1, which this build also uses for OLED SCL. Writing
// to Serial therefore glitches the I2C clock, so debug logging is
// compiled out whenever the OLED owns GPIO1.
#if (OLED_SDA == 1) || (OLED_SCL == 1)
#define DBG_ENABLED 0
#else
#define DBG_ENABLED 1
#endif
#define DBG(...) do { if (DBG_ENABLED) Serial.printf(__VA_ARGS__); } while (0)

// Auto-exposure target bias: -2 (darker) .. +2 (brighter), 0 = neutral.
// The 1-bit dithered preview is very high contrast, so -1 often keeps
// highlights from blowing out; the JPEG stills use the same setting.
#define AE_LEVEL 0

// Still JPEG quality, esp32-camera scale 0-63 (lower = better and larger).
// Also the quality used when a JPEG_FILTER re-encodes the image.
#define JPEG_QUALITY 10

// Camera orientation. 1 = flip. Setting both to 1 rotates the image 180
// degrees (for a module mounted upside down in its housing); set both to 0
// for a normally oriented module. Applies to the preview and the stills.
#define CAM_VFLIP 1
#define CAM_HMIRROR 1

// Viewfinder recovery. A few dropped frames are normal (mode switches, SD
// activity, a slow AGC step), so a restart only happens once the camera has
// produced nothing for CAM_STALL_MS. Failed restarts back off so the unit
// cannot thrash -- an unthrottled retry loop flickers between two OLED
// messages and hammers the sensor with init cycles.
#define CAM_STALL_MS 1500
#define CAM_BACKOFF_MIN_MS 1000
#define CAM_BACKOFF_MAX_MS 8000

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---------- Globals ----------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long frames = 0;
unsigned long lastFpsT = 0;
bool sdReady = false;
bool oledReady = false;
bool btnPrev = true;  // pull-up idle HIGH
unsigned long lastBtnT = 0;
int sdFailStreak = 0;
unsigned long lastSdCheck = 0;
unsigned long lastFrameT = 0;  // time of the last good viewfinder frame

void showText(const char* l1, const char* l2 = nullptr, const char* l3 = nullptr) {
  DBG("[OLED] %s %s %s\n", l1, l2 ? l2 : "", l3 ? l3 : "");
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(l1);
  if (l2) display.println(l2);
  if (l3) display.println(l3);
  display.display();
}

// Short name for why the chip last restarted. A repeated panic and a brownout
// look identical from the outside -- both simply reset -- so this is the only
// way to tell them apart without a serial console.
static const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "int-wdt";
    case ESP_RST_TASK_WDT:  return "task-wdt";
    case ESP_RST_SW:        return "sw-restart";
    case ESP_RST_EXT:       return "ext-reset";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    default:                return "unknown";
  }
}

// The camera runs ONE continuous stream that serves both the viewfinder and
// the stills: SXGA JPEG. Because preview and capture share the configuration,
// a capture needs no deinit/init cycle and no AWB/AEC settle -- the next frame
// is simply written to the card. That removes ~1.1 s of latency per shot.
// The trade-off is that the viewfinder must decode each frame (below).
#define CAM_FRAMESIZE FRAMESIZE_SXGA
// One frame buffer: halves the PSRAM the stream reserves at SXGA and avoids the
// double-buffered JPEG path while we are chasing this crash.
#define CAM_FB_COUNT 1

// Fallback ladder. A large JPEG frame needs two PSRAM buffers and not every
// module -- or supply -- copes with the biggest one. Each entry is tried in
// turn and the OLED reports which one stuck, so the usable ceiling is visible
// on-device.
struct CamMode {
  framesize_t size;
  int fbCount;
};

static const CamMode CAM_MODES[] = {
  { CAM_FRAMESIZE, CAM_FB_COUNT },
  { FRAMESIZE_XGA, 2 },
  { FRAMESIZE_SVGA, 2 },
  { FRAMESIZE_VGA, 1 },
};

// Label for the OLED, derived from the mode itself so changing CAM_FRAMESIZE
// cannot leave a stale name behind.
static const char* framesizeName(framesize_t s) {
  switch (s) {
    case FRAMESIZE_SXGA: return "SXGA";
    case FRAMESIZE_XGA:  return "XGA";
    case FRAMESIZE_SVGA: return "SVGA";
    case FRAMESIZE_VGA:  return "VGA";
    default:             return "??";
  }
}

static bool cameraBegin(framesize_t size, int fbCount) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.frame_size = size;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = fbCount;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DBG("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    // Auto exposure / gain / white balance, set explicitly. These match the
    // OV2640 driver defaults, but stating them keeps the intent obvious and
    // guards against driver default changes.
    s->set_exposure_ctrl(s, 1);  // auto exposure (AEC)
    s->set_aec2(s, 1);           // DSP auto exposure
    s->set_gain_ctrl(s, 1);      // auto gain (AGC)
    s->set_whitebal(s, 1);       // auto white balance (AWB)
    s->set_awb_gain(s, 1);       // AWB gain
    s->set_ae_level(s, AE_LEVEL);

    // Colour-processing blocks. All of these are on by default; restating
    // them makes the expected pipeline explicit.
    s->set_special_effect(s, 0);  // 0 = no special effect
    s->set_bpc(s, 1);             // black pixel correction
    s->set_wpc(s, 1);             // white pixel correction
    s->set_lenc(s, 1);            // lens shading correction

    // Orientation, applied at the sensor so the OLED preview and the saved
    // JPEGs agree. A 180 degree rotation needs both flips.
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);

    // Neutral tone: this single stream feeds both the dithered preview and the
    // saved JPEG, so per-mode contrast is no longer possible. The contrast the
    // 1-bit panel wants is applied in the dither renderer instead.
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 1);
  }
  return true;
}

// Try each mode in turn. The driver is always torn down first: calling
// esp_camera_init() again on a half-initialised driver is what turned a failed
// start into a reboot loop.
static bool cameraStart(const char** chosenName) {
  for (const CamMode& m : CAM_MODES) {
    esp_camera_deinit();
    if (cameraBegin(m.size, m.fbCount)) {
      if (chosenName) *chosenName = framesizeName(m.size);
      return true;
    }
    DBG("Camera mode %s failed\n", framesizeName(m.size));
  }
  return false;
}

// Bisect switch. Set to 0 to skip the viewfinder decode entirely (the OLED
// then shows only status text, no image). If the board stops resetting, the
// JPEG decoder is implicated; if it still resets, the fault is power or the
// camera stream itself.
#ifndef PREVIEW_DECODE_ENABLED
#define PREVIEW_DECODE_ENABLED 1
#endif

// --- Viewfinder decode -----------------------------------------------------
// SXGA JPEG -> 1/8 scale (160x128) grayscale, using the library's
// jpg2rgb565() helper -- the path the official examples use -- instead of a
// hand-rolled esp_jpg_decode() callback pair.
#define PREVIEW_DECODE_W 160
#define PREVIEW_DECODE_H 128

static uint8_t previewGray[PREVIEW_DECODE_W * PREVIEW_DECODE_H];
static uint8_t* previewRgb = nullptr;  // RGB565 scratch, 40 KB, allocated once
static int previewGrayW = 0;
static int previewGrayH = 0;

static bool decodePreviewGray(const camera_fb_t* fb) {
  const int w = (int)fb->width / 8;
  const int h = (int)fb->height / 8;
  if (w <= 0 || h <= 0) return false;
  if (w > PREVIEW_DECODE_W || h > PREVIEW_DECODE_H) return false;

  if (!previewRgb) {
    previewRgb = (uint8_t*)ps_malloc((size_t)PREVIEW_DECODE_W *
                                     PREVIEW_DECODE_H * sizeof(uint16_t));
    if (!previewRgb) return false;
  }

  if (!jpg2rgb565(fb->buf, fb->len, previewRgb, JPG_SCALE_8X)) return false;

  // The helper emits RGB565 in an LCD-oriented byte order, so derive luma from
  // the two bytes directly: that is byte-order independent and more than good
  // enough for a 1-bit preview.
  const uint16_t* src = (const uint16_t*)previewRgb;
  for (int y = 0; y < h; y++) {
    uint8_t* drow = previewGray + (size_t)y * w;
    const uint16_t* srow = src + (size_t)y * w;
    for (int x = 0; x < w; x++) {
      const uint16_t v = srow[x];
      drow[x] = (uint8_t)((((v >> 8) & 0xFF) + (v & 0xFF)) >> 1);
    }
  }

  previewGrayW = w;
  previewGrayH = h;
  return true;
}

// Draw one decoded frame on the OLED. Downscaling and letterboxing are shared
// by every mode; the image processing itself is selected at compile time via
// PREVIEW_MODE (see preview.h).
void drawPreview(const uint8_t* img, int w, int h) {
  if (!oledReady) return;
  static PreviewFrame frame;  // ~8 KB scratch, kept off the stack
  previewPrepare(frame, img, w, h, SCREEN_WIDTH, SCREEN_HEIGHT);
  if (frame.w <= 0 || frame.h <= 0) return;
  display.clearDisplay();
  previewDraw(display, frame);
  display.display();
}

// (drawTemp removed: temp-monitor mode replaced by camera stream)

bool initSD() {
  // 1-bit mode keeps GPIO4 (flash LED) free; SD uses 14/15/2 internally.
  if (!SD_MMC.begin("/sdcard", true)) {
    DBG("SD_MMC mount failed\n");
    return false;
  }
  uint8_t ct = SD_MMC.cardType();
  if (ct == CARD_NONE) {
    DBG("No SD card attached\n");
    return false;
  }
  DBG("SD card: %llu MB\n", SD_MMC.cardSize() / (1024ULL * 1024ULL));
  return true;
}

// Re-try mounting after a card is inserted at runtime.
bool retrySD() {
  SD_MMC.end();
  delay(50);
  return initSD();
}

// Liveness probe for the mounted card. SD_MMC.cardType() is cached at mount
// time and keeps reporting the old type after the card is pulled, so a stat
// on the mounted root is used instead -- it actually hits the medium.
bool sdAlive() {
  return sdReady && SD_MMC.exists("/");
}

// Block until a card is mounted, showing status on the OLED. Used at boot and
// again whenever a card is pulled at runtime.
void waitForSD() {
  showText("Checking SD...", "FAT32 microSD");
  sdReady = initSD();
  unsigned int waits = 0;
  while (!sdReady) {
    char line[20];
    snprintf(line, sizeof(line), "waiting %us", waits * 2);
    showText("NO SD CARD", (waits >= 5) ? "FAT32 formatted?" : "Insert microSD",
             line);
    DBG("Waiting for SD card...\n");
    delay(2000);
    waits++;
    sdReady = retrySD();
  }
  sdFailStreak = 0;
  showText("SD card OK", "Ready to capture");
  DBG("SD card mounted\n");
  delay(600);
}

// Restart the stream, retrying a few times. The camera is no longer torn down
// for captures, so this only runs when the sensor itself wedges.
bool restartCamera() {
  for (int attempt = 0; attempt < 3; attempt++) {
    const char* mode = nullptr;
    if (cameraStart(&mode)) {
      lastFrameT = millis();
      frames = 0;
      lastFpsT = millis();
      return true;
    }
    delay(200);
  }
  return false;
}

// Find the next free /img_NNNN.jpg and write it into `out`. Returns false when
// all slots are taken. The scan remembers where it stopped, so a full card is
// not re-walked from index 0 on every shot.
static unsigned int lastPhotoIdx = 0;

bool nextPhotoPath(char* out, size_t outLen) {
  for (unsigned int i = 0; i < 10000; i++) {
    unsigned int n = (lastPhotoIdx + i) % 10000;
    snprintf(out, outLen, "/img_%04u.jpg", n);
    if (!SD_MMC.exists(out)) {
      lastPhotoIdx = n;
      return true;
    }
  }
  return false;
}

// Adapter so the JPEG pipeline can stream straight into an open SD file.
struct FileSink {
  File* file;
  bool failed;
};

static size_t fileSinkWrite(void* arg, const uint8_t* data, size_t len) {
  FileSink* s = (FileSink*)arg;
  const size_t n = s->file->write(data, len);
  if (n != len) s->failed = true;
  return n;
}

// Outcome of a still capture, so each failure gets an honest message.
enum PhotoResult {
  PHOTO_OK,
  PHOTO_CAM_FAIL,
  PHOTO_NO_SPACE,
  PHOTO_INDEX_FULL,
  PHOTO_WRITE_FAIL,
  PHOTO_FILTER_FAIL,
};

// Capture one JPEG and store it on the card. Fills `name` with the path used.
// The viewfinder stream is already SXGA JPEG, so a still is just the next
// frame: no re-init and no AWB/AEC settle.
PhotoResult captureStill(char* name, size_t nameLen) {
  name[0] = '\0';

  PhotoResult result = PHOTO_CAM_FAIL;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb || fb->format != PIXFORMAT_JPEG || fb->len == 0) {
    DBG("Still capture failed (fmt=%d len=%u)\n",
        fb ? (int)fb->format : -1, fb ? (unsigned)fb->len : 0u);
    result = PHOTO_CAM_FAIL;
  } else {
    // The pipeline streams its output straight into the file, so the encoded
    // JPEG is never held in RAM -- peak PSRAM stays low enough for a 4 MB
    // module. With JPEG_FILTER_NONE this is a single write of the camera
    // buffer, with no decode or re-encode at all.
    char path[24];
    if (!nextPhotoPath(path, sizeof(path))) {
      DBG("Photo index exhausted\n");
      result = PHOTO_INDEX_FULL;
    } else {
      // The exact size is unknown before encoding, so reserve against an
      // upper bound rather than discovering a full card mid-write.
      const size_t worst = jpegFilterWorstCaseBytes(fb);
      const uint64_t freeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
      if (freeBytes < (uint64_t)worst + 4096) {
        DBG("Not enough space: %llu free, %u needed\n", freeBytes, (unsigned)worst);
        result = PHOTO_NO_SPACE;
      } else {
        File f = SD_MMC.open(path, FILE_WRITE);
        if (!f) {
          DBG("Open %s failed\n", path);
          result = PHOTO_WRITE_FAIL;
        } else {
          FileSink sink = { &f, false };
          size_t written = 0;
          uint16_t ow = 0, oh = 0;
          const bool okFilter = jpegFilterStream(fb, JPEG_QUALITY, fileSinkWrite,
                                                 &sink, &written, &ow, &oh);
          f.close();
          if (!okFilter) {
            SD_MMC.remove(path);
            DBG("JPEG filter failed\n");
            result = PHOTO_FILTER_FAIL;
          } else if (sink.failed || written == 0) {
            // Never leave a truncated JPEG behind: it would look like a valid
            // photo and permanently consume a name slot.
            SD_MMC.remove(path);
            DBG("Short write on %s (%u bytes) -- removed\n", path, (unsigned)written);
            result = PHOTO_WRITE_FAIL;
          } else {
            snprintf(name, nameLen, "%s", path);
            DBG("Saved %s (%u bytes, %ux%u)\n", path, (unsigned)written,
                (unsigned)ow, (unsigned)oh);
            result = PHOTO_OK;
          }
        }
      }
    }
  }
  if (fb) esp_camera_fb_return(fb);
  return result;
}

void savePhoto() {
  // Cheap pre-checks first: a card that vanished a moment ago should not cost
  // a camera teardown plus an 800 ms warm-up before finding out.
  if (!sdReady || !sdAlive()) {
    DBG("savePhoto: SD unavailable\n");
    SD_MMC.end();
    sdReady = false;
    waitForSD();
    return;
  }

  showText("Capturing...", "SXGA JPEG");

  char name[24];
  PhotoResult result = captureStill(name, sizeof(name));

  switch (result) {
    case PHOTO_OK:         showText("Saved to SD", name); break;
    case PHOTO_NO_SPACE:   showText("Save FAILED", "SD card full"); break;
    case PHOTO_INDEX_FULL: showText("Save FAILED", "10000 files"); break;
    case PHOTO_WRITE_FAIL: showText("Save FAILED", "SD write error"); break;
    case PHOTO_FILTER_FAIL: showText("Save FAILED", "filter error"); break;
    case PHOTO_CAM_FAIL:
    default:               showText("Capture FAILED", "camera error"); break;
  }
  delay(result == PHOTO_OK ? 900 : 1200);

  // A failed grab is left to recoverCamera() on the next loop iteration.
}

// Recover a camera that has stopped delivering frames. Only acts once the
// stall exceeds CAM_STALL_MS, and backs off between successive attempts.
bool recoverCamera() {
  if (millis() - lastFrameT < CAM_STALL_MS) return false;

  static unsigned int restarts = 0;
  char line[20];
  snprintf(line, sizeof(line), "attempt %u", restarts + 1);
  DBG("Viewfinder stalled -- restarting camera (attempt %u)\n", restarts + 1);
  showText("Camera restart", line);

  if (restartCamera()) {
    restarts = 0;
    lastFrameT = millis();
    return true;
  }

  restarts++;
  unsigned long backoff = CAM_BACKOFF_MIN_MS << (restarts > 4 ? 3 : restarts - 1);
  if (backoff > CAM_BACKOFF_MAX_MS) backoff = CAM_BACKOFF_MAX_MS;
  snprintf(line, sizeof(line), "retry %lus", backoff / 1000);
  DBG("Camera restart failed -- backing off %lu ms\n", backoff);
  showText("Camera offline", line);
  delay(backoff);
  return false;
}

void streamFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    recoverCamera();
    delay(5);
    return;
  }
  lastFrameT = millis();
  if (fb->format != PIXFORMAT_JPEG) {
    DBG("Unexpected format %d\n", (int)fb->format);
    esp_camera_fb_return(fb);
    return;
  }
  // Decode at 1/8 scale and draw -- tens of ms per frame on a 240 MHz core.
#if PREVIEW_DECODE_ENABLED
  // Marker: if the panel shows this and then resets, the frame grab worked and
  // the decoder is the culprit.
  static bool firstFrameShown = false;
  if (!firstFrameShown) {
    firstFrameShown = true;
    showText("Frame OK", "decoding...");
  }
  if (decodePreviewGray(fb)) {
    drawPreview(previewGray, previewGrayW, previewGrayH);
  }
#endif
  esp_camera_fb_return(fb);

  frames++;
  unsigned long now = millis();
  if (now - lastFpsT >= 5000) {
    float fps = (float)frames * 1000.0f / (float)(now - lastFpsT);
    DBG("OLED viewfinder: %.1f fps\n", fps);
    frames = 0;
    lastFpsT = now;
  }
}

// Never give up on the camera: an init failure is usually transient (brownout
// or slow sensor power-up), so retry with a visible counter instead of
// halting and forcing a manual reset.
void waitForCamera() {
  unsigned int attempt = 0;
  for (;;) {
    char line[20];
    snprintf(line, sizeof(line), "attempt %u", attempt + 1);
    showText(attempt == 0 ? "Init camera..." : "Retry camera...", line);
    const char* mode = nullptr;
    if (cameraStart(&mode)) {
      showText("Camera OK", mode);
      DBG("Camera ready at %s\n", mode);
      delay(400);
      return;
    }
    DBG("Camera init failed (attempt %u)\n", attempt + 1);
    attempt++;
    delay(1000);
  }
}

void setup() {
  // U0TXD (GPIO1) doubles as OLED SCL in this build, so the UART is only
  // started when logging is actually enabled -- otherwise the UART would
  // hold the same pad as the I2C clock.
#if DBG_ENABLED
  Serial.begin(115200);
  delay(200);
#endif

  Wire.begin(OLED_SDA, OLED_SCL);

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR) ||
      display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    oledReady = true;
  } else {
    DBG("SSD1306 not found -- continuing without OLED\n");
  }

  char info[24];
  snprintf(info, sizeof(info), "PSRAM %u KB",
           (unsigned)(ESP.getPsramSize() / 1024));
  showText("ESP32-CAM OLED", info, resetReasonStr());
  delay(1500);  // long enough to actually read the two diagnostic lines

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ---- SD card is required: block here until one is inserted ----
  waitForSD();

  waitForCamera();

  lastFrameT = millis();
  lastFpsT = millis();
}

void loop() {
  bool btn = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // Runtime card monitoring. A pulled card stops the viewfinder and waits,
  // matching the boot behaviour, then auto-recovers when it is reinserted.
  // Two consecutive failures are required so a transient I/O hiccup does not
  // trigger a needless remount.
  if ((now - lastSdCheck) > 1000) {
    lastSdCheck = now;
    if (sdAlive()) {
      sdFailStreak = 0;
    } else if (++sdFailStreak >= 2) {
      sdFailStreak = 0;
      DBG("SD card removed -- waiting for reinsertion\n");
      SD_MMC.end();
      sdReady = false;
      waitForSD();
      frames = 0;
      lastFpsT = millis();
    }
  }

  // Active-low button with 50 ms debounce; capture on press edge.
  if (!btn && btnPrev && (now - lastBtnT) > 50) {
    lastBtnT = now;
    DBG("Button pressed -> capture\n");
    savePhoto();
  }
  btnPrev = btn;
  streamFrame();
}
