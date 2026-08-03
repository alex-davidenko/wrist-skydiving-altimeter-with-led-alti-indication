#include "display.h"

#include "config.h"
#include "zones.h"

#if !DISPLAY_ENABLED

namespace display {
bool begin() { return false; }
void message(const char *, const char *) {}
void update(float, float, uint8_t, const char *, bool, uint32_t) {}
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
uint16_t g_lastBg   = 0xDEAD;
int32_t  g_lastAlt  = INT32_MIN;
int32_t  g_lastVs   = INT32_MIN;
char     g_lastLabel[16] = {0};

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

uint16_t backgroundFor(uint8_t zone, bool fault)
{
  if (fault) return RGB565_MAGENTA;
  switch (zone)
  {
    case ZONE_BLINK_RED:
    case ZONE_RED:    return RGB565_RED;
    case ZONE_YELLOW: return RGB565_ORANGE;
    case ZONE_GREEN:  return RGB565_GREEN;
    case ZONE_ABOVE:  return RGB565_BLUE;
    case ZONE_OFF:
    default:          return RGB565_BLACK;
  }
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
  g_lastBg = 0xDEAD;          // force a repaint on the next update()
  g_lastAlt = INT32_MIN;
  g_lastVs = INT32_MIN;
  g_lastLabel[0] = '\0';
}

void update(float altitudeM, float vspeedMps, uint8_t zone,
            const char *label, bool fault, uint32_t nowMs)
{
  if (!g_ok) return;
  (void)nowMs;

  const uint16_t bg  = backgroundFor(zone, fault);
  const uint16_t ink = inkFor(bg);

  // A full fill is the expensive operation on this panel, so it happens only
  // when the zone colour genuinely changes — not every frame.
  const bool repaint = (bg != g_lastBg);
  if (repaint)
  {
    fillAll(bg);
    g_lastBg  = bg;
    g_lastAlt = INT32_MIN;    // everything on top must be redrawn
    g_lastVs  = INT32_MIN;
    g_lastLabel[0] = '\0';
  }

  // Altitude in whole metres, right-aligned in a fixed-width field. Fixed width
  // plus an opaque text background means the previous value is overwritten
  // in place, with no erase step and therefore no flicker.
  const int32_t alt = (int32_t)lrintf(altitudeM);
  if (alt != g_lastAlt)
  {
    char buf[8];
    snprintf(buf, sizeof(buf), "%5ld", (long)alt);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(ALT_TEXT_SIZE, ALT_TEXT_SIZE, 0);
    const int16_t tw = 5 * 6 * ALT_TEXT_SIZE;
    const int16_t th = 8 * ALT_TEXT_SIZE;
    g_gfx->setCursor((g_w - tw) / 2, (g_h - th) / 2);
    g_gfx->print(buf);
    g_lastAlt = alt;
  }

  if (label && strncmp(label, g_lastLabel, sizeof(g_lastLabel) - 1) != 0)
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "%-11s", label);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(2, 2, 0);
    g_gfx->setCursor(8, 8);
    g_gfx->print(buf);
    strncpy(g_lastLabel, label, sizeof(g_lastLabel) - 1);
    g_lastLabel[sizeof(g_lastLabel) - 1] = '\0';
  }

  // Vertical speed, quantised to 0.1 m/s so it is not redrawn every sample.
  const int32_t vs = (int32_t)lrintf(vspeedMps * 10.0f);
  if (vs != g_lastVs)
  {
    char buf[20];
    snprintf(buf, sizeof(buf), "%+6.1f m/s", vs / 10.0f);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(2, 2, 0);
    g_gfx->setCursor(8, g_h - 24);
    g_gfx->print(buf);
    g_lastVs = vs;
  }
}

}  // namespace display

#endif  // DISPLAY_ENABLED
