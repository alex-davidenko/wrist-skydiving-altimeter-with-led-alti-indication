#include "touch.h"

#include "config.h"

#if !TOUCH_ENABLED

namespace touch {
bool begin() { return false; }
bool available() { return false; }
Point takeTouch() { return {0, 0, false}; }
void rawLast(int16_t *x, int16_t *y) { *x = 0; *y = 0; }
}  // namespace touch

#else

#include <Wire.h>

namespace touch {

namespace {

constexpr uint8_t kAddr        = 0x63;
constexpr uint8_t kRegId       = 0x08;
constexpr uint8_t kRegTouch    = 0x01;

bool g_ok = false;
volatile bool g_intFlag = false;

uint32_t g_lastEventMs = 0;
int16_t g_rawX = 0, g_rawY = 0;
int16_t g_pendX = 0, g_pendY = 0;
volatile bool g_pending = false;

void IRAM_ATTR onTouchInt() { g_intFlag = true; }

bool readReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  // NOTE the STOP: endTransmission() with no argument, not endTransmission(false).
  // A repeated start does not work with this controller — Waveshare's driver
  // sends a full stop between the register write and the read, and copying that
  // exactly is what made it respond.
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)kAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// The controller reports in the panel's native 172x320 portrait frame. The
// display runs rotated, so the axes have to be remapped — and which way round
// depends on how the panel is mounted, which no datasheet will tell you.
// Hence the config switches plus the `T` calibration dump.
void mapToDisplay(int16_t rx, int16_t ry, int16_t *dx, int16_t *dy)
{
  int16_t a = rx, b = ry;
#if TOUCH_SWAP_XY
  a = ry;
  b = rx;
#endif
#if TOUCH_FLIP_X
  a = (DISPLAY_W - 1) - a;
#endif
#if TOUCH_FLIP_Y
  b = (DISPLAY_H - 1) - b;
#endif
  *dx = a;
  *dy = b;
}

}  // namespace

bool available() { return g_ok; }

void rawLast(int16_t *x, int16_t *y) { *x = g_rawX; *y = g_rawY; }

bool begin()
{
  pinMode(PIN_TOUCH_RST, OUTPUT);
  digitalWrite(PIN_TOUCH_RST, LOW);
  delay(200);
  digitalWrite(PIN_TOUCH_RST, HIGH);
  delay(300);

  // Presence is an address ACK, nothing more. Do NOT validate the ID register:
  // Waveshare's own driver reads it and only prints it when non-zero, never
  // treating zeros as failure — this chip appears to return zeros there, and
  // checking it rejected a perfectly working controller.
  Wire.beginTransmission(kAddr);
  if (Wire.endTransmission() != 0)
  {
    Serial.println(F("touch: no ACK at 0x63"));
    return false;
  }

  uint8_t id[3] = {0};
  readReg(kRegId, id, 3);   // informational only

  pinMode(PIN_TOUCH_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TOUCH_INT), onTouchInt, FALLING);

  g_ok = true;
  Serial.printf("touch: AXS5106L ready (id %02X %02X %02X)\n", id[0], id[1], id[2]);
  return true;
}

Point takeTouch()
{
  Point p{0, 0, false};
  if (!g_ok) return p;

  if (g_intFlag)
  {
    g_intFlag = false;
    uint8_t d[14] = {0};
    if (readReg(kRegTouch, d, sizeof(d)) && d[1] > 0)
    {
      // The controller keeps reporting while a finger is down — one tap
      // produced ~20 identical events in testing. Collapse a press into a
      // single event, or every menu button would fire many times per touch.
      const uint32_t now = millis();
      if (static_cast<uint32_t>(now - g_lastEventMs) < TOUCH_DEBOUNCE_MS)
      {
        return p;
      }
      g_lastEventMs = now;
      g_rawX = (int16_t)((((uint16_t)(d[2] & 0x0F)) << 8) | d[3]);
      g_rawY = (int16_t)((((uint16_t)(d[4] & 0x0F)) << 8) | d[5]);
      mapToDisplay(g_rawX, g_rawY, &g_pendX, &g_pendY);
      g_pending = true;
    }
  }

  if (g_pending)
  {
    g_pending = false;
    p.x = g_pendX;
    p.y = g_pendY;
    p.valid = true;
  }
  return p;
}

}  // namespace touch

#endif  // TOUCH_ENABLED
