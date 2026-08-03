#include "display.h"

#include "config.h"
#include "zones.h"

#if !DISPLAY_ENABLED

namespace display {
bool begin() { return false; }
void message(const char *, const char *) {}
void startTask() {}
void publish(const LedPattern &, float, float) {}
uint32_t lastFillUs() { return 0; }
bool available() { return false; }
}  // namespace display

#else

#include <Arduino_GFX_Library.h>

namespace display {

namespace {

Arduino_DataBus *g_bus = nullptr;
Arduino_GFX     *g_gfx = nullptr;
bool             g_ok  = false;
uint32_t         g_fillUs = 0;

// Panel geometry after rotation.
int16_t g_w = 0, g_h = 0;

// Change tracking — the panel is slow, so we only redraw what actually moved.
uint16_t g_lastBg    = 0xDEAD;
int32_t  g_lastAlt   = INT32_MIN;
int32_t  g_lastVs    = INT32_MIN;
uint8_t  g_lastAltSz = 0;

// State handed over from the sample loop on the other core. Small enough that
// a spinlock costs nothing; a mutex would risk blocking the sample loop.
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
struct Shared
{
  Rgb      color    = {0, 0, 0};
  uint16_t periodMs = 0;
  float    altM     = 0.0f;
  float    vsMps    = 0.0f;
};
Shared g_shared;

// The JD9853 register sequence, straight from Waveshare's own Arduino example.
// Arduino_GFX has no JD9853 driver in any released version (checked 1.5.9 and
// 1.6.7), so we drive it with the ST7789 class — the addressing and pixel
// commands are compatible — and push the panel-specific setup by hand.
// 0xDF 0x98 0x53 is the JD9853 command unlock; 0x21 turns on inversion, which
// this panel needs or every colour comes out complemented.
const uint8_t kJd9853Init[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,          // sleep out
    END_WRITE,
    DELAY, 120,

    BEGIN_WRITE,
    WRITE_C8_D16, 0xDF, 0x98, 0x53, // unlock vendor commands
    WRITE_C8_D8,  0xB2, 0x23,

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 4, 0x00, 0x47, 0x00, 0x6F,

    WRITE_COMMAND_8, 0xBB,
    WRITE_BYTES, 6, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,

    WRITE_C8_D16, 0xC0, 0x44, 0xA4,
    WRITE_C8_D8,  0xC1, 0x16,

    WRITE_COMMAND_8, 0xC3,
    WRITE_BYTES, 8, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,

    WRITE_COMMAND_8, 0xC4,
    WRITE_BYTES, 12, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,

    WRITE_COMMAND_8, 0xC8,
    WRITE_BYTES, 32,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
    0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,

    WRITE_COMMAND_8, 0xD0,
    WRITE_BYTES, 5, 0x04, 0x06, 0x6B, 0x0F, 0x00,

    WRITE_C8_D16, 0xD7, 0x00, 0x30,
    WRITE_C8_D8,  0xE6, 0x14,
    WRITE_C8_D8,  0xDE, 0x01,

    WRITE_COMMAND_8, 0xB7,
    WRITE_BYTES, 5, 0x03, 0x13, 0xEF, 0x35, 0x35,

    WRITE_COMMAND_8, 0xC1,
    WRITE_BYTES, 3, 0x14, 0x15, 0xC0,

    WRITE_C8_D16, 0xC2, 0x06, 0x3A,
    WRITE_C8_D16, 0xC4, 0x72, 0x12,
    WRITE_C8_D8,  0xBE, 0x00,
    WRITE_C8_D8,  0xDE, 0x02,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3, 0x00, 0x02, 0x00,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3, 0x01, 0x02, 0x00,

    WRITE_C8_D8, 0xDE, 0x00,
    WRITE_C8_D8, 0x35, 0x00,
    WRITE_C8_D8, 0x3A, 0x05,

    WRITE_COMMAND_8, 0x2A,
    WRITE_BYTES, 4, 0x00, 0x22, 0x00, 0xCD,

    WRITE_COMMAND_8, 0x2B,
    WRITE_BYTES, 4, 0x00, 0x00, 0x01, 0x3F,

    WRITE_C8_D8, 0xDE, 0x02,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 3, 0x00, 0x02, 0x00,

    WRITE_C8_D8, 0xDE, 0x00,
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_COMMAND_8, 0x21,          // inversion on
    END_WRITE,

    DELAY, 10,

    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x29,          // display on
    END_WRITE,
};

// The LED module already decides what the device should be showing for every
// state, including the landing ladder and the fault pattern. Reusing its
// LedPattern keeps one source of truth instead of a second colour table that
// could drift out of step.
uint16_t rgb565(const Rgb &c)
{
  return ((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3);
}

// Pick black or white text from the background's luminance, so yellow and
// green stay readable instead of glowing white-on-white.
uint16_t inkFor(uint16_t bg565)
{
  const uint8_t r = ((bg565 >> 11) & 0x1F) << 3;
  const uint8_t g = ((bg565 >> 5) & 0x3F) << 2;
  const uint8_t b = (bg565 & 0x1F) << 3;
  const uint16_t luma = (r * 77 + g * 150 + b * 29) >> 8;
  return (luma > 140) ? RGB565_BLACK : RGB565_WHITE;
}

void fillAll(uint16_t c)
{
  const uint32_t t0 = micros();
  g_gfx->fillScreen(c);
  g_fillUs = micros() - t0;
}

}  // namespace

bool available() { return g_ok; }
uint32_t lastFillUs() { return g_fillUs; }

bool begin()
{
  // Panel reset is done by hand on LCD_RST before the driver starts. Waveshare
  // pass GPIO47 as the driver's reset pin as well, which is a touch-controller
  // pin; we leave that alone since we do not use touch and the manual pulse
  // below is the reset that matters.
  pinMode(PIN_LCD_RST, OUTPUT);
  digitalWrite(PIN_LCD_RST, LOW);
  delay(10);
  digitalWrite(PIN_LCD_RST, HIGH);
  delay(10);

  g_bus = new Arduino_ESP32SPI(PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_SCK, PIN_LCD_MOSI);
  g_gfx = new Arduino_ST7789(g_bus, GFX_NOT_DEFINED, 0 /* rotation */,
                             false /* IPS */, 172, 320,
                             34 /* col offset 1 */, 0 /* row offset 1 */,
                             34 /* col offset 2 */, 0 /* row offset 2 */);

  if (!g_gfx->begin())
  {
    Serial.println(F("display: gfx begin() failed"));
    return false;
  }

  g_bus->batchOperation(kJd9853Init, sizeof(kJd9853Init));
  g_gfx->setRotation(DISPLAY_ROTATION);

  g_w = g_gfx->width();
  g_h = g_gfx->height();

  fillAll(RGB565_BLACK);

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  g_ok = true;
  Serial.printf("display: JD9853 %dx%d ready (full fill %lu us)\n",
                g_w, g_h, (unsigned long)g_fillUs);
  return true;
}

void message(const char *line1, const char *line2)
{
  if (!g_ok) return;
  fillAll(RGB565_BLACK);
  g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  g_gfx->setTextSize(2, 2, 0);
  g_gfx->setCursor(8, 40);
  g_gfx->print(line1 ? line1 : "");
  g_gfx->setCursor(8, 70);
  g_gfx->print(line2 ? line2 : "");
  g_lastBg = 0xDEAD;          // force a full repaint on the next frame
  g_lastAlt = INT32_MIN;
  g_lastVs = INT32_MIN;
  g_lastAltSz = 0;
}

namespace {

// Draw one frame from an already-copied snapshot. Runs only on the display
// task, so it is free to take as long as it likes.
void renderFrame(const Shared &st, uint32_t nowMs)
{
  // Blink by alternating the background to black. The number is redrawn on
  // top of whichever colour is showing, so it stays readable throughout.
  bool on = true;
  if (st.periodMs > 0) on = (nowMs % st.periodMs) < (st.periodMs / 2u);

  const uint16_t bg  = on ? rgb565(st.color) : RGB565_BLACK;
  const uint16_t ink = inkFor(bg);

  if (bg != g_lastBg)
  {
    fillAll(bg);
    g_lastBg    = bg;
    g_lastAlt   = INT32_MIN;   // everything on top must be redrawn
    g_lastVs    = INT32_MIN;
    g_lastAltSz = 0;
  }

  // Altitude in whole metres, as large as will fit. The label that used to sit
  // at the top is gone — the background colour already says which zone we are
  // in, so the text was redundant and the digits get the whole screen.
  const int32_t alt = (int32_t)lrintf(st.altM);
  char buf[8];
  snprintf(buf, sizeof(buf), "%ld", (long)alt);
  const uint8_t len = strlen(buf);

  // Widest size that fits both the width (for this many digits) and the height
  // of the band above the vertical-speed line.
  const int16_t band = g_h - ALT_BOTTOM_BAND;
  uint8_t sz = (g_w - 12) / (len * 6);
  const uint8_t szByHeight = band / 8;
  if (sz > szByHeight) sz = szByHeight;
  if (sz > ALT_TEXT_SIZE_MAX) sz = ALT_TEXT_SIZE_MAX;
  if (sz < 1) sz = 1;

  if (alt != g_lastAlt || sz != g_lastAltSz)
  {
    // A shorter number, or a smaller size, would leave the old glyphs behind.
    // Opaque text only covers its own cells, so clear the band on any change
    // of geometry. Digit-count changes are rare, so this is cheap in practice.
    if (sz != g_lastAltSz) g_gfx->fillRect(0, 0, g_w, band, bg);

    const int16_t tw = len * 6 * sz;
    const int16_t th = 8 * sz;
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(sz, sz, 0);
    g_gfx->setCursor((g_w - tw) / 2, (band - th) / 2);
    g_gfx->print(buf);

    g_lastAlt   = alt;
    g_lastAltSz = sz;
  }

  // Vertical speed, quantised to 0.1 m/s so it is not redrawn every frame.
  const int32_t vs = (int32_t)lrintf(st.vsMps * 10.0f);
  if (vs != g_lastVs)
  {
    char vbuf[20];
    snprintf(vbuf, sizeof(vbuf), "%+6.1f m/s", vs / 10.0f);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(2, 2, 0);
    g_gfx->setCursor(8, g_h - 20);
    g_gfx->print(vbuf);
    g_lastVs = vs;
  }
}

void displayTask(void *)
{
  for (;;)
  {
    Shared st;
    portENTER_CRITICAL(&g_mux);
    st = g_shared;
    portEXIT_CRITICAL(&g_mux);

    renderFrame(st, millis());
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
  }
}

}  // namespace

void publish(const LedPattern &p, float altitudeM, float vspeedMps)
{
  portENTER_CRITICAL(&g_mux);
  g_shared.color    = p.color;
  g_shared.periodMs = p.periodMs;
  g_shared.altM     = altitudeM;
  g_shared.vsMps    = vspeedMps;
  portEXIT_CRITICAL(&g_mux);
}

void startTask()
{
  if (!g_ok) return;
  // Pinned to the core the Arduino loop does NOT run on, so a 23.7 ms
  // full-screen fill cannot stall the 25 ms sample loop.
  xTaskCreatePinnedToCore(displayTask, "display", DISPLAY_TASK_STACK,
                          nullptr, DISPLAY_TASK_PRIO, nullptr,
                          DISPLAY_TASK_CORE);
  Serial.printf("display: render task on core %d (loop on core %d)\n",
                DISPLAY_TASK_CORE, xPortGetCoreID());
}

}  // namespace display

#endif  // DISPLAY_ENABLED
