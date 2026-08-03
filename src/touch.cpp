#include "touch.h"

#include "config.h"

#if !TOUCH_ENABLED

namespace touch {
bool begin() { return false; }
bool available() { return false; }
Event takeEvent() { return {EV_NONE, 0, 0}; }
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

int16_t g_rawX = 0, g_rawY = 0;

// Contact in progress: where it started, where it is now, and when we last
// heard from it. Absence of reports is what stands in for a release event.
bool     g_active  = false;
int16_t  g_startX = 0, g_startY = 0;
int16_t  g_lastX  = 0, g_lastY  = 0;
uint32_t g_lastMs = 0;

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

Event takeEvent()
{
  Event e{EV_NONE, 0, 0};
  if (!g_ok) return e;

  if (g_intFlag)
  {
    g_intFlag = false;
    uint8_t d[14] = {0};
    if (readReg(kRegTouch, d, sizeof(d)) && d[1] > 0)
    {
      g_rawX = (int16_t)((((uint16_t)(d[2] & 0x0F)) << 8) | d[3]);
      g_rawY = (int16_t)((((uint16_t)(d[4] & 0x0F)) << 8) | d[5]);
      int16_t x, y;
      mapToDisplay(g_rawX, g_rawY, &x, &y);

      if (!g_active)
      {
        g_active = true;
        g_startX = x;
        g_startY = y;
      }
      g_lastX = x;
      g_lastY = y;
      g_lastMs = millis();
    }
  }

  // The contact has gone quiet for long enough to call it a release.
  if (g_active && static_cast<uint32_t>(millis() - g_lastMs) > TOUCH_RELEASE_MS)
  {
    g_active = false;
    const int16_t dx = g_lastX - g_startX;
    const int16_t dy = g_lastY - g_startY;

    e.x = g_startX;
    e.y = g_startY;
    if (abs(dx) >= TOUCH_SWIPE_MIN_PX && abs(dx) > abs(dy))
      e.type = (dx < 0) ? EV_SWIPE_LEFT : EV_SWIPE_RIGHT;
    else
      e.type = EV_TAP;
  }
  return e;
}

}  // namespace touch

#endif  // TOUCH_ENABLED
