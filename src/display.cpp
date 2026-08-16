#include "display.h"

#include "config.h"
#include "zones.h"

#if !DISPLAY_ENABLED

namespace display {
bool begin() { return false; }
void message(const char *, const char *) {}
void bootFace() {}
void bootFaceOut() {}
void bannerIn(const char *, const char *) {}
void bannerLine1(const char *) {}
void bannerLine2In(const char *) {}
void bannerOut() {}
void sleepFace() {}
void startTask() {}
void publish(const LedPattern &, float, float) {}
void setBattery(float) {}
void setTopText(const char *) {}
void setUnitsFeet(bool) {}
bool unitsFeet() { return false; }
void setScreen(uint8_t) {}
uint8_t screen() { return UI_ALT; }
void setBanner(const char *, const char *) {}
void setClockEdit(const int16_t *, uint8_t) {}
uint8_t hitTest(int16_t, int16_t) { return ACT_NONE; }
void sleep() {}
void wake() {}
uint32_t lastFillUs() { return 0; }
bool available() { return false; }
}  // namespace display

#else

#include <Arduino_GFX_Library.h>

#include "font_alt.h"

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
int32_t  g_lastBatt  = INT32_MIN;
int32_t  g_lastAltN  = 0;             // last displayed value, in display steps
char     g_lastTop[16] = {1, 0};      // deliberately not a string we ever set
uint8_t  g_lastUnit  = 0xFF;
const AltFont *g_lastAltFont = nullptr;
char     g_lastStr[12] = {0};

// Scratch cell for compositing one digit off-screen. Big enough for the widest
// advance at the tallest ink; lives in PSRAM because internal RAM is precious.
uint16_t *g_cell = nullptr;

// State handed over from the sample loop on the other core. Small enough that
// a spinlock costs nothing; a mutex would risk blocking the sample loop.
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
struct Shared
{
  Rgb      color    = {0, 0, 0};
  uint16_t periodMs = 0;
  float    altM     = 0.0f;
  float    vsMps    = 0.0f;
  uint8_t  screen   = UI_ALT;
  float    battV    = 0.0f;
  char     top[16]  = {0};
  bool     feet     = UNITS_FEET_DEFAULT;
  char     l1[20]   = {0};
  char     l2[20]   = {0};
};
Shared g_shared;
uint8_t g_lastScreen = 0xFF;
volatile bool g_suspended = false;

// Backlight state, cached so the task does not write the GPIO every tick.
// It lives here rather than inside the task because sleep() and wake() also
// drive the pin: a copy local to the task went stale across a light sleep, and
// the backlight stayed off after waking while the panel was drawn to normally.
bool g_blOn = true;

// Button rectangles. hitTest() and the renderer both read these, so a button
// can never be drawn somewhere other than where it responds.
struct Rect { int16_t x, y, w, h; };
constexpr Rect kBtnLeft  = { 20, 40, 132,  95};
constexpr Rect kBtnRight = {168, 40, 132,  95};
constexpr Rect kBtnWide  = { 60, 40, 200,  95};   // single-button pages

// Clock editor: three controls along the bottom, value above them.
// Three list rows, then a footer with prev/next. Rows are wide and tall
// because they are tapped with a gloved thumb, not a stylus.
constexpr Rect kBtnReplay = { 196, 140, 116,  28};   // detail page, bottom right
constexpr Rect kLbRow[3] = {{ 10, 30, 300, 34},
                            { 10, 66, 300, 34},
                            { 10,102, 300, 34}};
constexpr Rect kLbPrev = {  6,140,  90, 28};
constexpr Rect kLbNext = {224,140,  90, 28};

constexpr Rect kClkDown = { 20, 96,  80, 54};
constexpr Rect kClkNext = {120, 96,  80, 54};
constexpr Rect kClkUp   = {220, 96,  80, 54};

// Published from the loop. Guarded by the same spinlock as the sample state.
int16_t g_clkF[5]   = {2026, 1, 1, 0, 0};

// Logbook page, formatted by main.cpp and copied here under the spinlock.
char     g_lbRow[3][32] = {{0}};
uint8_t  g_lbPage = 0, g_lbPages = 0;
uint16_t g_lbTotal = 0;
char     g_dtTitle[24] = {0};
char     g_dtLine[5][32] = {{0}};
uint8_t g_clkActive = 0;

bool inside(const Rect &r, int16_t x, int16_t y)
{
  return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

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

// Digits are BLACK on every colour the device shows, and white only on a dark
// background where black would be invisible.
//
// This is a daylight decision, not an aesthetic one. Outdoors the panel cannot
// outshine the sun, so emitted white washes out — but black pixels stay black,
// because they are simply not emitting. Dark-on-bright is the only contrast
// that survives in direct sunlight, which is the environment this thing has to
// work in. Indoors white-on-colour looks nicer; outdoors it disappears.
uint16_t inkFor(uint16_t bg565)
{
  const uint8_t r = ((bg565 >> 11) & 0x1F) << 3;
  const uint8_t g = ((bg565 >> 5) & 0x3F) << 2;
  const uint8_t b = (bg565 & 0x1F) << 3;
  const uint16_t luma = (r * 77 + g * 150 + b * 29) >> 8;
  return (luma > 30) ? RGB565_BLACK : RGB565_WHITE;
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

  const size_t cellPx = (size_t)kFontAltBig.advance *
                        (kFontAltBig.inkBottom - kFontAltBig.inkTop);
  g_cell = (uint16_t *)ps_malloc(cellPx * sizeof(uint16_t));
  if (!g_cell) g_cell = (uint16_t *)malloc(cellPx * sizeof(uint16_t));

  g_ok = true;
  Serial.printf("display: JD9853 %dx%d ready (full fill %lu us)\n",
                g_w, g_h, (unsigned long)g_fillUs);
  return true;
}

// The render task caches what it last drew so it can redraw only what changed.
// Anything that paints the panel behind its back has to say so, or the next
// frame will leave whatever we drew sitting underneath.
static void forceRepaint()
{
  g_lastBg = 0xDEAD;
  g_lastAlt = INT32_MIN;
  g_lastVs = INT32_MIN;
  g_lastBatt = INT32_MIN;
  g_lastUnit = 0xFF;
  g_lastAltFont = nullptr;
  g_lastStr[0] = '\0';
}

#if BOOT_FACE_ENABLED
namespace {

// Eye geometry, in the 320x172 landscape frame.
//
// EVE's eyes are elongated along their length and tilted, inner ends low and
// outer ends high, nearly meeting in the middle — not the upright ovals this
// started as. fillEllipse cannot rotate, but composeEyes() rasterises its own
// scanlines, so the rotation is just a change of basis in the membership test.
constexpr int16_t  kEyeDx   = 72;      // half the gap between the two centres
constexpr int16_t  kEyeRx   = 62;      // along the eye's own long axis
constexpr int16_t  kEyeRy   = 26;      // across it
constexpr float    kEyeTilt = 0.31f;   // ~18 deg, outer end high, inner end low
constexpr uint16_t kEyeBlue = 0x3DBF;  // ~#38B6FF

// Half-extents of the rotated ellipse, which is what the cell has to cover.
constexpr float kCosT = 0.9511f, kSinT = 0.3090f;
constexpr int16_t kEyeHalfW = 60;      // sqrt((rx*cos)^2 + (ry*sin)^2), rounded up
constexpr int16_t kEyeHalfH = 32;      // sqrt((rx*sin)^2 + (ry*cos)^2), rounded up

// The smile: how thin the eye ends up, and how far it sinks while closing.
// Sink is 0 deliberately. Drifting the blue down as it closed looked smoother
// but it is the wrong physics: when you smile the upper lid barely moves and
// only the lower one rises, so the arc should stay where the top of the eye
// was and the black should do all the travelling. Left as a constant because
// it is the obvious thing to reach for again.
constexpr int16_t kSmileThin = 9;
constexpr int16_t kSmileSink = 0;

// One PSRAM cell covering both eyes, composited off-screen and pushed as a
// single blit. Drawing black to the panel and then the eye on top leaves the
// region blank for milliseconds, and at this frame rate that reads as flicker —
// the same fault the altitude digits had, and the same fix.
constexpr int16_t kFaceX = DISPLAY_W / 2 - (kEyeDx + kEyeHalfW) - 2;
constexpr int16_t kFaceY = DISPLAY_H / 2 - kEyeHalfH - 2;
constexpr int16_t kFaceW = 2 * (kEyeDx + kEyeHalfW + 2);
constexpr int16_t kFaceH = 2 * (kEyeHalfH + 2);
uint16_t *g_faceBuf = nullptr;

uint16_t dim(uint16_t c, int num, int den)
{
  const uint8_t r = ((c >> 11) & 0x1F) * num / den;
  const uint8_t g = ((c >> 5) & 0x3F) * num / den;
  const uint8_t b = (c & 0x1F) * num / den;
  return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

// True if (dx,dy) from an eye centre falls inside that eye, tilted by `sinT`.
inline bool inEye(float dx, float dy, float sinT, float ry)
{
  const float u =  dx * kCosT + dy * sinT;
  const float v = -dx * sinT  + dy * kCosT;
  const float a = u / kEyeRx, b = v / ry;
  return a * a + b * b < 1.0f;
}

// One frame, rasterised into the cell and blitted once.
//
// Each eye is the tilted ellipse minus a second copy of itself shifted `carve`
// further down the screen. Two identical shapes offset purely vertically leave
// a band whose thickness is exactly `carve` at every column, so carve at or
// above the eye's full height leaves it whole, and shrinking carve closes the
// eye from its lower edge upward. That is how a smile closes an eye: the cheek
// lifts the lower lid while the upper barely moves, which is also why the blue
// must not slide down as it thins.
void composeEyes(int16_t ry, int16_t sink, int16_t carve, uint16_t colour)
{
  if (!g_faceBuf) return;
  memset(g_faceBuf, 0, static_cast<size_t>(kFaceW) * kFaceH * sizeof(uint16_t));
  if (ry < 1) ry = 1;

  const int16_t cx = DISPLAY_W / 2, cy = DISPLAY_H / 2;

  for (int side = -1; side <= 1; side += 2)
  {
    const int16_t ex = cx + side * kEyeDx;
    const int16_t ey = cy + sink;
    // Outer end high, inner end low, mirrored about the centre line.
    const float sinT = -side * kSinT;

    for (int16_t py = 0; py < kFaceH; py++)
    {
      const float dy = static_cast<float>(kFaceY + py - ey);
      if (dy < -kEyeHalfH || dy > kEyeHalfH + carve) continue;
      uint16_t *row = g_faceBuf + static_cast<size_t>(py) * kFaceW;

      for (int16_t px = 0; px < kFaceW; px++)
      {
        const float dx = static_cast<float>(kFaceX + px - ex);
        if (dx < -kEyeHalfW || dx > kEyeHalfW) continue;
        if (!inEye(dx, dy, sinT, ry)) continue;
        if (inEye(dx, dy - carve, sinT, ry)) continue;   // carved away
        row[px] = colour;
      }
    }
  }
  g_gfx->draw16bitRGBBitmap(kFaceX, kFaceY, g_faceBuf, kFaceW, kFaceH);
}

// Carve this large removes nothing, so it is the "eye fully open" value.
constexpr int16_t kEyeOpen = 2 * kEyeHalfH + 4;

void blinkOnce(int downMs, int upMs)
{
  for (int r = kEyeRy; r >= 3; r -= 4) { composeEyes(r, 0, kEyeOpen, kEyeBlue); delay(downMs); }
  for (int r = 3; r <= kEyeRy; r += 4) { composeEyes(r, 0, kEyeOpen, kEyeBlue); delay(upMs); }
  composeEyes(kEyeRy, 0, kEyeOpen, kEyeBlue);
}

bool faceBegin()
{
  g_faceBuf = static_cast<uint16_t *>(
      ps_malloc(static_cast<size_t>(kFaceW) * kFaceH * sizeof(uint16_t)));
  return g_faceBuf != nullptr;
}

void faceEnd()
{
  free(g_faceBuf);
  g_faceBuf = nullptr;
}

}  // namespace
#endif

void bootFace()
{
#if BOOT_FACE_ENABLED
  if (!g_ok) return;

  if (!faceBegin()) return;             // cosmetic only; never block the boot

  fillAll(RGB565_BLACK);
  delay(180);

  for (int i = 1; i <= 12; i++)           // eyes fade up
  {
    composeEyes(kEyeRy, 0, kEyeOpen, dim(kEyeBlue, i, 12));
    delay(28);
  }
  delay(340);

  blinkOnce(12, 14);
  delay(150);
  blinkOnce(12, 14);
  delay(500);

  // The smile snaps rather than slides: three frames, which at one 156x88 blit
  // apiece is around 25 ms. Enough motion that it is not a hard cut, too fast
  // to read as travel. Easing it in looked like the face was thinking about it.
  const int kSteps = 3;
  for (int i = 1; i <= kSteps; i++)
  {
    const int16_t carve = kEyeOpen - (kEyeOpen - kSmileThin) * i / kSteps;
    const int16_t sink  = kSmileSink * i / kSteps;
    composeEyes(kEyeRy, sink, carve, kEyeBlue);
  }
  delay(400);
  // Deliberately ends here, with the smile still up and the cell still
  // allocated. Whatever the caller does next — the boot zero takes about two
  // seconds — happens while the face is smiling, instead of over a black
  // screen that looks like a hang. bootFaceOut() finishes the job.
#endif
}

void bootFaceOut()
{
#if BOOT_FACE_ENABLED
  if (!g_ok || !g_faceBuf) return;

  for (int i = 11; i >= 0; i--)
  {
    composeEyes(kEyeRy, kSmileSink, kSmileThin, dim(kEyeBlue, i, 12));
    delay(24);
  }
  fillAll(RGB565_BLACK);
  delay(120);

  faceEnd();
  forceRepaint();
#endif
}

void sleepFace()
{
#if BOOT_FACE_ENABLED
  if (!g_ok) return;

  // Unlike bootFace(), this runs with the renderer already live on the other
  // core, so take the panel first — the same handshake sleep() uses.
  g_suspended = true;
  delay(80);                            // let any in-flight draw finish

  if (!faceBegin()) return;

  fillAll(RGB565_BLACK);
  delay(120);

  for (int i = 1; i <= 10; i++)         // eyes fade up
  {
    composeEyes(kEyeRy, 0, kEyeOpen, dim(kEyeBlue, i, 10));
    delay(30);
  }
  delay(320);

  blinkOnce(26, 30);                    // two slow, sleepy blinks
  delay(380);
  blinkOnce(30, 34);
  delay(300);

  for (int r = kEyeRy; r >= 1; r -= 3)  // and the last one does not open again
  {
    composeEyes(r, 0, kEyeOpen, kEyeBlue);
    delay(34);
  }
  for (int i = 9; i >= 0; i--)
  {
    composeEyes(1, 0, kEyeOpen, dim(kEyeBlue, i, 10));
    delay(26);
  }
  fillAll(RGB565_BLACK);
  delay(150);

  faceEnd();
#endif
}

// Two-line banner, drawn straight to the panel. Safe to fade without an
// off-screen cell because the text is drawn with an opaque background: each
// pass repaints its own glyph boxes, so there is never a clear step leaving it
// blank. That absence is the whole reason the eyes needed a PSRAM cell.
//
// A line that can SHRINK must be space-padded by the caller to a constant
// width, or the tail of the longer previous string is never painted over.
namespace {

char g_bL1[40] = {0};
char g_bL2[40] = {0};

uint16_t grey(int num, int den)
{
  const uint8_t r = 31 * num / den, g = 63 * num / den, b = 31 * num / den;
  return static_cast<uint16_t>((r << 11) | (g << 5) | b);
}

void paintBanner(uint16_t fg1, uint16_t fg2)
{
  g_gfx->setTextSize(2, 2, 0);
  g_gfx->setTextColor(fg1, RGB565_BLACK);
  g_gfx->setCursor(4, 58);
  g_gfx->print(g_bL1);
  g_gfx->setTextColor(fg2, RGB565_BLACK);
  g_gfx->setCursor(4, 96);
  g_gfx->print(g_bL2);
}

}  // namespace

void bannerIn(const char *line1, const char *line2)
{
  if (!g_ok) return;
  snprintf(g_bL1, sizeof(g_bL1), "%s", line1 ? line1 : "");
  snprintf(g_bL2, sizeof(g_bL2), "%s", line2 ? line2 : "");
  fillAll(RGB565_BLACK);
  for (int i = 1; i <= 10; i++) { paintBanner(grey(i, 10), grey(i, 10)); delay(24); }
  forceRepaint();
}

void bannerLine1(const char *line1)
{
  if (!g_ok) return;
  snprintf(g_bL1, sizeof(g_bL1), "%s", line1 ? line1 : "");
  paintBanner(0xFFFF, g_bL2[0] ? 0xFFFF : RGB565_BLACK);
  forceRepaint();
}

void bannerLine2In(const char *line2)
{
  if (!g_ok) return;
  snprintf(g_bL2, sizeof(g_bL2), "%s", line2 ? line2 : "");
  for (int i = 1; i <= 10; i++) { paintBanner(0xFFFF, grey(i, 10)); delay(26); }
  forceRepaint();
}

void bannerOut()
{
  if (!g_ok) return;
  for (int i = 9; i >= 0; i--) { paintBanner(grey(i, 10), grey(i, 10)); delay(24); }
  fillAll(RGB565_BLACK);
  forceRepaint();
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
  forceRepaint();
}

namespace {

// Draw one frame from an already-copied snapshot. Runs only on the display
// task, so it is free to take as long as it likes.
void drawButton(const Rect &r, uint16_t fill, const char *l1, const char *l2)
{
  g_gfx->fillRect(r.x, r.y, r.w, r.h, fill);
  g_gfx->drawRect(r.x, r.y, r.w, r.h, RGB565_WHITE);
  const uint16_t ink = inkFor(fill);
  g_gfx->setTextColor(ink, fill);
  g_gfx->setTextSize(2, 2, 0);
  const int16_t cy = r.y + r.h / 2 - (l2 && l2[0] ? 18 : 8);
  g_gfx->setCursor(r.x + (r.w - (int16_t)strlen(l1) * 12) / 2, cy);
  g_gfx->print(l1);
  if (l2 && l2[0])
  {
    g_gfx->setCursor(r.x + (r.w - (int16_t)strlen(l2) * 12) / 2, cy + 22);
    g_gfx->print(l2);
  }
}

// Two dots at the bottom, filled for the current page, plus an arrow on the
// side that has somewhere to go. Without this there is nothing to tell you a
// second page exists.
void drawPager(uint8_t page, uint8_t pages)
{
  const int16_t cx = DISPLAY_W / 2;
  const int16_t y  = DISPLAY_H - 13;
  for (uint8_t i = 0; i < pages; i++)
  {
    const int16_t x = cx + (int16_t)(i * 18) - (int16_t)((pages - 1) * 9);
    if (i == page) g_gfx->fillCircle(x, y, 5, RGB565_WHITE);
    else           g_gfx->drawCircle(x, y, 5, RGB565_WHITE);
  }
  g_gfx->setTextSize(1, 1, 0);
  g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  if (page + 1 < pages)
  {
    g_gfx->setCursor(DISPLAY_W - 66, y - 3);
    g_gfx->print("swipe <");
  }
  if (page > 0)
  {
    g_gfx->setCursor(8, y - 3);
    g_gfx->print("> swipe");
  }
}

void renderUi(const Shared &st)
{
  g_gfx->fillScreen(RGB565_BLACK);
  g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  g_gfx->setTextSize(2, 2, 0);

  switch (st.screen)
  {
    case UI_MENU:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnLeft,  RGB565_GREEN,  "ZERO",  "HERE");
      drawButton(kBtnRight, RGB565_ORANGE, "POWER", "OFF");
      drawPager(0, 4);
      break;

    case UI_MENU3:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnLeft,  RGB565_BLUE,   "USB",   "DRIVE");
      drawButton(kBtnRight, RGB565_ORANGE, "UNITS",
                 st.feet ? "ft -> m" : "m -> ft");
      drawPager(1, 4);
      break;

    case UI_MENU4:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnLeft,  RGB565_BLUE, "SET", "CLOCK");
      drawButton(kBtnRight, RGB565_BLUE, "JUMP", "No.");
      drawPager(2, 4);
      break;
    case UI_MENU5:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnWide, RGB565_GREEN, "LOG", "BOOK");
#if LED_TEST_ENABLED
      drawButton(kBtnLeft,  RGB565_BLUE,  "LOG",  "BOOK");
      drawButton(kBtnRight, RGB565_GREEN, "LED",  "TEST");
#endif
      drawPager(3, 4);
      break;

    case UI_LOGBOOK:
    {
      char rows[3][32]; uint8_t pg, pgs; uint16_t tot;
      portENTER_CRITICAL(&g_mux);
      // Fixed-size copy between identical buffers. snprintf here made the
      // compiler assume the source might not terminate inside its own row.
      for (int i = 0; i < 3; i++)
      {
        memcpy(rows[i], g_lbRow[i], sizeof(rows[i]));
        rows[i][sizeof(rows[i]) - 1] = '\0';
      }
      pg = g_lbPage; pgs = g_lbPages; tot = g_lbTotal;
      portEXIT_CRITICAL(&g_mux);

      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(10, 12);
      char hdr[32];
      snprintf(hdr, sizeof(hdr), "LOGBOOK  %u jump%s", tot, tot == 1 ? "" : "s");
      g_gfx->print(hdr);

      for (int i = 0; i < 3; i++)
      {
        if (!rows[i][0]) continue;
        g_gfx->fillRoundRect(kLbRow[i].x, kLbRow[i].y, kLbRow[i].w, kLbRow[i].h,
                             4, RGB565_DARKGREY);
        g_gfx->setTextSize(2, 2, 0);
        g_gfx->setTextColor(RGB565_WHITE, RGB565_DARKGREY);
        g_gfx->setCursor(kLbRow[i].x + 8, kLbRow[i].y + 10);
        g_gfx->print(rows[i]);
      }

      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      if (pgs > 1)
      {
        if (pg > 0)        { g_gfx->setCursor(kLbPrev.x + 8, kLbPrev.y + 10); g_gfx->print("< NEWER"); }
        if (pg + 1 < pgs)  { g_gfx->setCursor(kLbNext.x + 8, kLbNext.y + 10); g_gfx->print("OLDER >"); }
        char f[16]; snprintf(f, sizeof(f), "%u/%u", pg + 1, pgs);
        g_gfx->setCursor(150, kLbPrev.y + 10); g_gfx->print(f);
      }
      if (!tot)
      {
        g_gfx->setTextSize(2, 2, 0);
        g_gfx->setCursor(40, 70);
        g_gfx->print("no jumps yet");
      }
      break;
    }
    case UI_JUMP_DETAIL:
    {
      char title[24], lines[5][32];
      portENTER_CRITICAL(&g_mux);
      snprintf(title, sizeof(title), "%s", g_dtTitle);
      for (int i = 0; i < 5; i++)
      {
        memcpy(lines[i], g_dtLine[i], sizeof(lines[i]));
        lines[i][sizeof(lines[i]) - 1] = '\0';
      }
      portEXIT_CRITICAL(&g_mux);

      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
      g_gfx->setCursor(10, 10);
      g_gfx->print(title);
      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      for (int i = 0; i < 5; i++)
      {
        g_gfx->setCursor(14, 44 + i * 22);
        g_gfx->print(lines[i]);
      }
      g_gfx->setCursor(14, 158);
      g_gfx->print("tap to go back");
      drawButton(kBtnReplay, RGB565_GREEN, "REPLAY", "");
      break;
    }
    case UI_LED_TEST:
    {
      char nm[32]; int16_t br; uint8_t act;
      portENTER_CRITICAL(&g_mux);
      snprintf(nm, sizeof(nm), "%s", g_lbRow[0]);
      br = g_clkF[0]; act = g_clkActive;
      portEXIT_CRITICAL(&g_mux);

      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setTextColor(act == 0 ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(10, 16);
      char line[24];
      snprintf(line, sizeof(line), "%-14s", nm);
      g_gfx->print(line);

      g_gfx->setTextColor(act == 1 ? RGB565_YELLOW : RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(10, 48);
      snprintf(line, sizeof(line), "bright %3d    ", br);
      g_gfx->print(line);

      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(10, 78);
      g_gfx->print("NEXT switches field, then DONE");
      drawButton(kClkDown, RGB565_BLACK, "-", "");
      drawButton(kClkNext, RGB565_BLUE, act < 1 ? "NEXT" : "DONE", "");
      drawButton(kClkUp,   RGB565_BLACK, "+", "");
      break;
    }
    case UI_SET_JUMPNO:
    {
      int16_t d[4];
      uint8_t act;
      portENTER_CRITICAL(&g_mux);
      for (int i = 0; i < 4; i++) d[i] = g_clkF[i];
      act = g_clkActive;
      portEXIT_CRITICAL(&g_mux);

      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(40, 18);
      g_gfx->print("JUMPS SO FAR");

      char buf[8];
      // Each is 0-9 by construction, but the type says int16_t. Reduce to a
      // char so the bound is in the code rather than only in the invariant.
      for (int i = 0; i < 4; i++) buf[i] = (char)('0' + (d[i] < 0 ? 0 : d[i] % 10));
      buf[4] = '\0';
      constexpr int16_t kX = 112, kY = 44, kAdv = 24;
      g_gfx->setTextSize(4, 4, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(kX, kY);
      g_gfx->print(buf);
      if (act < 4)
      {
        g_gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
        g_gfx->setCursor(kX + act * kAdv, kY);
        char one[2] = {buf[act], 0};
        g_gfx->print(one);
      }
      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(20, 84);
      g_gfx->print("hold +/- to run   swipe to cancel");
      drawButton(kClkDown, RGB565_BLACK, "-", "");
      drawButton(kClkNext, RGB565_BLUE, act < 3 ? "NEXT" : "SET", "");
      drawButton(kClkUp,   RGB565_BLACK, "+", "");
      break;
    }
    case UI_SET_CLOCK:
    {
      int16_t f[5];
      uint8_t act;
      portENTER_CRITICAL(&g_mux);
      memcpy(f, g_clkF, sizeof(f));
      act = g_clkActive;
      portEXIT_CRITICAL(&g_mux);

      char buf[40];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d",
               f[0], f[1], f[2], f[3], f[4]);

      // Whole string in white, then the field being edited again in yellow.
      // The built-in font advances a fixed 12 px at size 2, so a character
      // index is an x offset — no measuring needed.
      constexpr int16_t kX = 64, kY = 44, kAdv = 12;
      static const uint8_t kOff[5] = {0, 5, 8, 11, 14};
      static const uint8_t kLen[5] = {4, 2, 2, 2, 2};
      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(kX, kY);
      g_gfx->print(buf);
      if (act < 5)
      {
        char part[6];
        snprintf(part, sizeof(part), "%.*s", kLen[act], buf + kOff[act]);
        g_gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
        g_gfx->setCursor(kX + kOff[act] * kAdv, kY);
        g_gfx->print(part);
      }
      g_gfx->setTextSize(1, 1, 0);
      g_gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      g_gfx->setCursor(20, 76);
      g_gfx->print("hold +/- to run   swipe to cancel");

      drawButton(kClkDown, RGB565_BLACK, "-", "");
      drawButton(kClkNext, RGB565_BLUE,  act < 4 ? "NEXT" : "SET", "");
      drawButton(kClkUp,   RGB565_BLACK, "+", "");
      break;
    }

    case UI_CONFIRM_ZERO:
    case UI_CONFIRM_UNMOUNT:
    case UI_CONFIRM_USB:
    case UI_CONFIRM_POWER:
      g_gfx->setCursor(20, 12);
      g_gfx->print(st.screen == UI_CONFIRM_UNMOUNT ? "UNMOUNT CARD?"
                 : st.screen == UI_CONFIRM_POWER   ? "POWER OFF?"
                 : st.screen == UI_CONFIRM_USB     ? "REBOOT TO USB?"
                                                   : "SET ZERO HERE?");
      drawButton(kBtnLeft,  RGB565_BLACK, "CANCEL", "");
      drawButton(kBtnRight, RGB565_RED,   "YES",    "");
      break;

    case UI_BANNER:
    default:
      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setCursor(20, 60);
      g_gfx->print(st.l1);
      g_gfx->setCursor(20, 95);
      g_gfx->print(st.l2);
      break;
  }
}

// Rounding that sticks. The displayed integer only changes once the value has
// cleared the halfway boundary by ALT_DISPLAY_HYST, so a reading sitting on
// that boundary does not flicker between two numbers on sensor noise.
int32_t roundSticky(float v, int32_t last)
{
  if (v > (float)last + 0.5f + ALT_DISPLAY_HYST ||
      v < (float)last - 0.5f - ALT_DISPLAY_HYST)
    return (int32_t)floorf(v + 0.5f);
  return last;
}

// Composite one tabular digit cell off-screen and push it in a single
// transaction.
//
// Drawing text straight to the panel meant erasing the old glyph and then
// drawing the new one, leaving the digit genuinely blank for several
// milliseconds in between — at 20 Hz that reads as constant flicker. Building
// the cell in RAM and blitting it once removes the intermediate state, and
// also sidesteps Arduino_GFX's custom-font path entirely, whose background
// fill and baseline are both unreliable (see the notes in renderFrame).
// Advance of one glyph. Digits share a uniform tabular advance so a live number
// does not jitter; '.' and the 'K' suffix are proportional, which is what lets
// "12.3K" fit the width four digits occupy.
int16_t glyphAdvance(const AltFont *af, char ch)
{
  if (ch < af->font->first || ch > af->font->last) return 0;
  return af->font->glyph[ch - af->font->first].xAdvance;
}

int16_t textWidth(const AltFont *af, const char *s)
{
  int16_t w = 0;
  for (const char *p = s; *p; p++) w += glyphAdvance(af, *p);
  return w;
}

void blitDigit(const AltFont *af, char ch, int16_t penX, int16_t baseline,
               uint16_t ink, uint16_t bg)
{
  if (!g_cell) return;

  const int16_t cw = glyphAdvance(af, ch);
  if (cw <= 0) return;
  const int16_t chh = af->inkBottom - af->inkTop;
  for (int32_t i = 0; i < (int32_t)cw * chh; i++) g_cell[i] = bg;

  if (ch >= af->font->first && ch <= af->font->last)
  {
    const GFXglyph *g = &af->font->glyph[ch - af->font->first];
    const uint8_t  *bm = af->font->bitmap + g->bitmapOffset;
    const int16_t gx = g->xOffset;
    const int16_t gy = g->yOffset - af->inkTop;   // glyph top within the cell

    uint32_t bit = 0;
    for (int16_t yy = 0; yy < g->height; yy++)
    {
      for (int16_t xx = 0; xx < g->width; xx++, bit++)
      {
        if (!(bm[bit >> 3] & (0x80 >> (bit & 7)))) continue;
        const int16_t px = gx + xx, py = gy + yy;
        if (px < 0 || px >= cw || py < 0 || py >= chh) continue;
        g_cell[(int32_t)py * cw + px] = ink;
      }
    }
  }
  g_gfx->draw16bitRGBBitmap(penX, baseline + af->inkTop, g_cell, cw, chh);
}

// Content only. Blinking is NOT done here — see backlightPhase().
//
// Repainting the background to blink looked wrong on hardware: fillScreen
// pushes 880 kbit down a serial bus, so at 40 MHz the new colour visibly
// sweeps across the panel over 23.7 ms instead of changing at once. A
// framebuffer would not help; the flush is the same 23.7 ms transfer.
void renderFrame(const Shared &st)
{
  // Replay draws through the altitude view: same colours, same digits, same
  // blink. That is the point of it — you are watching what the device showed,
  // not a diagram of it. Only the top band differs, and publish() carries that.
  const bool altView = (st.screen == UI_ALT || st.screen == UI_REPLAY);
  if (!altView)
  {
    if (st.screen != g_lastScreen) { renderUi(st); g_lastScreen = st.screen; }
    return;
  }
  if (g_lastScreen != st.screen)
  {
    // Coming back from the menu, or into replay: force a full repaint.
    g_lastScreen = st.screen;
    g_lastBg = 0xDEAD;
  }

  const uint16_t bg  = rgb565(st.color);
  const uint16_t ink = inkFor(bg);

  if (bg != g_lastBg)
  {
    fillAll(bg);
    g_lastBg    = bg;
    g_lastAlt     = INT32_MIN;   // everything on top must be redrawn
    g_lastVs      = INT32_MIN;
    g_lastBatt    = INT32_MIN;
    g_lastTop[0]  = 1; g_lastTop[1] = 0;
    g_lastUnit    = 0xFF;
    g_lastAltFont = nullptr;
    g_lastStr[0]  = '\0';        // nothing to erase, the fill did it
  }

  // Altitude in whole metres, as large as will fit. The label that used to sit
  // at the top is gone — the background colour already says which zone we are
  // in, so the text was redundant and the digits get the whole screen.
  // Units are converted here and nowhere else. Feet round to 10 — at 50 m/s the
  // foot digit changes 164 times a second, which is unreadable, and rounding is
  // what production altimeters do. Above 9999 ft the Viso form "12.3K" is used
  // instead of a fifth digit, so the digits stay large rather than shrinking.
  // One display step is 5 m, or 10 ft. Both are rounded for the same reason:
  // at 50 m/s a 1 m digit changes 50 times a second and a 1 ft digit 164 times,
  // and neither is readable. 5 m is finer than 10 ft, so metres stay the more
  // precise of the two, as they were.
  //
  // BENCH keeps 1 m, because the whole bench range is 0-2 m and a 5 m step
  // would show a constant zero.
  const float step = st.feet ? 10.0f : (BENCH_MODE ? 1.0f : 5.0f);
  const float val  = st.feet ? st.altM * METRES_TO_FEET : st.altM;
  g_lastAltN = roundSticky(val / step, g_lastAltN);
  const int32_t alt = g_lastAltN * (int32_t)step;

  char buf[12];
  if (st.feet && labs(alt) >= 10000)
    snprintf(buf, sizeof(buf), "%.1fK", alt / 1000.0f);   // Viso form, keeps digits big
  else
    snprintf(buf, sizeof(buf), "%ld", (long)alt);

  // Pick the face by measured width, not by counting characters: "9.8K" fits
  // the large one while "12.3K" and "4500" do not.
  const int16_t usable = g_w - 8;
  const AltFont *af = (textWidth(&kFontAltBig, buf) <= usable) ? &kFontAltBig
                                                              : &kFontAltMed;
  const int16_t total = textWidth(af, buf);

  if (strcmp(buf, g_lastStr) != 0 || af != g_lastAltFont)
  {
    const int16_t bandY = ALT_TOP_BAND;
    const int16_t bandH = g_h - ALT_TOP_BAND;
    const int16_t inkH  = af->inkBottom - af->inkTop;
    const int16_t x0 = (g_w - total) / 2;
    const int16_t baseline = bandY + (bandH - inkH) / 2 - af->inkTop;

    // A different face, length or width moves every glyph, so the band has to
    // be cleared. Otherwise only the characters that changed are touched —
    // usually just the last one, which is what keeps this cheap at 20 Hz.
    const bool moved = (af != g_lastAltFont) ||
                       (strlen(g_lastStr) != strlen(buf)) ||
                       (textWidth(af, g_lastStr) != total);
    if (moved) g_gfx->fillRect(0, bandY, g_w, bandH, bg);

    int16_t pen = x0;
    for (uint8_t i = 0; buf[i]; i++)
    {
      if (moved || g_lastStr[i] != buf[i])
        blitDigit(af, buf[i], pen, baseline, ink, bg);
      pen += glyphAdvance(af, buf[i]);
    }

    strncpy(g_lastStr, buf, sizeof(g_lastStr) - 1);
    g_lastStr[sizeof(g_lastStr) - 1] = '\0';
    g_lastAltFont = af;
    g_lastAlt     = (int32_t)lrintf(st.altM);
  }

  // Top strip: a message if one is set, otherwise vertical speed.
  const bool topChanged = strcmp(st.top, g_lastTop) != 0;
  if (topChanged)
  {
    g_gfx->fillRect(0, 0, g_w - 70, ALT_TOP_BAND, bg);   // leave the battery
    strncpy(g_lastTop, st.top, sizeof(g_lastTop) - 1);
    g_lastTop[sizeof(g_lastTop) - 1] = '\0';
    g_lastVs   = INT32_MIN;                               // force a redraw below
    g_lastUnit = 0xFF;                                    // the label was wiped
  }

  if (st.top[0])
  {
    if (topChanged)
    {
      g_gfx->setFont(NULL);
      g_gfx->setTextColor(ink, bg);
      g_gfx->setTextSize(3, 3, 0);
      g_gfx->setCursor(8, 2);
      g_gfx->print(st.top);
    }
  }
  else
  {
    // Whole units in both systems, and quantised to exactly what is shown so
    // the strip repaints only when the text changes. A tenth of a m/s was
    // never readable at 50 m/s and only made the digits churn; ft/s is 0.3 m/s
    // of resolution, which is already finer than the reading is used for.
    const int32_t vs = st.feet ? (int32_t)lrintf(st.vsMps * METRES_TO_FEET)
                               : (int32_t)lrintf(st.vsMps);
    if (vs != g_lastVs)
    {
      char vbuf[20];
      if (st.feet) snprintf(vbuf, sizeof(vbuf), "%+4ld ft/s", (long)vs);
      else         snprintf(vbuf, sizeof(vbuf), "%+4ld m/s", (long)vs);
      g_gfx->setFont(NULL);
      g_gfx->setTextColor(ink, bg);
      g_gfx->setTextSize(2, 2, 0);
      g_gfx->setCursor(8, 5);
      g_gfx->print(vbuf);
      g_lastVs = vs;
    }
  }

  // Units, centred between the speed and the battery. Permanent, not implied:
  // reading feet as metres is a factor-3.28 error, so which one is on screen
  // must never be a matter of memory.
  const uint8_t unit = st.feet ? 1 : 0;
  if (unit != g_lastUnit)
  {
    const char *label = st.feet ? "FT" : "M";
    const int16_t w = (int16_t)strlen(label) * 18;        // size 3 glyph advance
    g_gfx->fillRect(196 - 24, 0, 56, ALT_TOP_BAND, bg);
    g_gfx->setFont(NULL);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(3, 3, 0);
    g_gfx->setCursor(196 - w / 2, 2);
    g_gfx->print(label);
    g_lastUnit = unit;
  }

  // Battery, top-right opposite the speed. Quantised to 10 mV so ADC jitter
  // does not repaint it constantly.
  // Quantise to the precision actually shown, so it repaints only when the
  // displayed text changes.
#if BATTERY_SCREEN_DECIMALS >= 3
  const int32_t bv = (int32_t)lrintf(st.battV * 1000.0f);
#elif BATTERY_SCREEN_DECIMALS == 2
  const int32_t bv = (int32_t)lrintf(st.battV * 100.0f);
#else
  const int32_t bv = (int32_t)lrintf(st.battV * 10.0f);
#endif
  if (bv != g_lastBatt)
  {
    char bbuf[12];
#if BATTERY_SCREEN_DECIMALS >= 3
    snprintf(bbuf, sizeof(bbuf), "%.3f", bv / 1000.0f);   // no "V": same width
#elif BATTERY_SCREEN_DECIMALS == 2
    snprintf(bbuf, sizeof(bbuf), "%.2fV", bv / 100.0f);
#else
    snprintf(bbuf, sizeof(bbuf), "%.1fV", bv / 10.0f);
#endif
    g_gfx->setFont(NULL);
    g_gfx->setTextColor(ink, bg);
    g_gfx->setTextSize(2, 2, 0);
    g_gfx->setCursor(g_w - 8 - (int16_t)strlen(bbuf) * 12, 5);
    g_gfx->print(bbuf);
    g_lastBatt = bv;
  }
}

// Blink the backlight rather than the pixels. One GPIO write switches the
// entire panel at once, which is what "blinking" should look like, and costs
// nothing on the bus. The frame memory keeps holding the zone colour and the
// number underneath, so the content is intact the instant the light returns.
bool backlightPhase(const Shared &st, uint32_t nowMs)
{
  // Never blink a menu. It has to stay readable while you are using it, and the
  // altitude pattern underneath is irrelevant to what is on screen. This is not
  // only a bench-scale annoyance: the menu would strobe when opened below 800 m
  // in freefall, or anywhere in the 100-300 m landing ladder under canopy.
  if (st.screen != UI_ALT && st.screen != UI_REPLAY) return true;

  // The panel stays lit even when the pattern colour is black. Black is the
  // "no alarm" state, not "no display" — at and below ground level you still
  // want to read the altitude, including negative values, on a dark screen.
  // Only an actual blink pattern turns the light off.
  if (st.periodMs == 0) return true;
  return (nowMs % st.periodMs) < (st.periodMs / 2u);
}

void displayTask(void *)
{
  uint32_t lastDraw = 0;

  for (;;)
  {
    Shared st;
    portENTER_CRITICAL(&g_mux);
    st = g_shared;
    portEXIT_CRITICAL(&g_mux);

    if (g_suspended) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    const uint32_t now = millis();

    // Backlight every tick, so blink edges land within DISPLAY_TICK_MS.
    const bool bl = backlightPhase(st, now);
    if (bl != g_blOn)
    {
      digitalWrite(PIN_LCD_BL, bl ? HIGH : LOW);
      g_blOn = bl;
    }

    // Pixels far less often, and only when something actually changed.
    if (static_cast<uint32_t>(now - lastDraw) >= DISPLAY_PERIOD_MS)
    {
      renderFrame(st);
      lastDraw = now;
    }

    vTaskDelay(pdMS_TO_TICKS(DISPLAY_TICK_MS));
  }
}

}  // namespace

void setBattery(float volts)
{
  portENTER_CRITICAL(&g_mux);
  g_shared.battV = volts;
  portEXIT_CRITICAL(&g_mux);
}

void setUnitsFeet(bool feet)
{
  portENTER_CRITICAL(&g_mux);
  const bool changed = (g_shared.feet != feet);
  g_shared.feet = feet;
  portEXIT_CRITICAL(&g_mux);
  if (changed)
  {
    // The number changes size and length; force a clean repaint.
    g_lastAltFont = nullptr;
    g_lastStr[0]  = '\0';
    g_lastVs      = INT32_MIN;
    g_lastBg      = 0xDEAD;
    g_lastAltN    = 0;
  }
}

bool unitsFeet()
{
  portENTER_CRITICAL(&g_mux);
  const bool f = g_shared.feet;
  portEXIT_CRITICAL(&g_mux);
  return f;
}

void setTopText(const char *text)
{
  portENTER_CRITICAL(&g_mux);
  snprintf(g_shared.top, sizeof(g_shared.top), "%s", text ? text : "");
  portEXIT_CRITICAL(&g_mux);
}

void publish(const LedPattern &p, float altitudeM, float vspeedMps)
{
  portENTER_CRITICAL(&g_mux);
  g_shared.color    = p.color;
  g_shared.periodMs = p.periodMs;
  g_shared.altM     = altitudeM;
  g_shared.vsMps    = vspeedMps;
  portEXIT_CRITICAL(&g_mux);
}

void setScreen(uint8_t s)
{
  portENTER_CRITICAL(&g_mux);
  g_shared.screen = s;
  portEXIT_CRITICAL(&g_mux);
}

uint8_t screen()
{
  portENTER_CRITICAL(&g_mux);
  const uint8_t s = g_shared.screen;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void setLogbookPage(const char rows[3][32], uint8_t page, uint8_t pages,
                    uint16_t total)
{
  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < 3; i++) snprintf(g_lbRow[i], sizeof(g_lbRow[i]), "%s", rows[i]);
  g_lbPage = page; g_lbPages = pages; g_lbTotal = total;
  g_shared.screen = UI_LOGBOOK;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;
}

void setJumpDetail(const char *title, const char lines[5][32])
{
  portENTER_CRITICAL(&g_mux);
  snprintf(g_dtTitle, sizeof(g_dtTitle), "%s", title ? title : "");
  for (int i = 0; i < 5; i++) snprintf(g_dtLine[i], sizeof(g_dtLine[i]), "%s", lines[i]);
  g_shared.screen = UI_JUMP_DETAIL;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;
}

void setLedTest(const char *name, uint8_t bright, uint8_t active)
{
  portENTER_CRITICAL(&g_mux);
  snprintf(g_lbRow[0], sizeof(g_lbRow[0]), "%s", name ? name : "");
  g_clkF[0] = bright;
  g_clkActive = active;
  g_shared.screen = UI_LED_TEST;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;
}

void setJumpEdit(const int16_t *d4, uint8_t active)
{
  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < 4; i++) g_clkF[i] = d4[i];
  g_clkActive = active;
  g_shared.screen = UI_SET_JUMPNO;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;
}

void setClockEdit(const int16_t *f5, uint8_t active)
{
  portENTER_CRITICAL(&g_mux);
  memcpy(g_clkF, f5, sizeof(g_clkF));
  g_clkActive = active;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;          // values changed, so repaint even on the same screen
}

void setBanner(const char *l1, const char *l2)
{
  portENTER_CRITICAL(&g_mux);
  snprintf(g_shared.l1, sizeof(g_shared.l1), "%s", l1 ? l1 : "");
  snprintf(g_shared.l2, sizeof(g_shared.l2), "%s", l2 ? l2 : "");
  g_shared.screen = UI_BANNER;
  portEXIT_CRITICAL(&g_mux);
  g_lastScreen = 0xFF;          // force a repaint even if already on a banner
}

uint8_t hitTest(int16_t x, int16_t y)
{
  switch (screen())
  {
    case UI_MENU:
      if (inside(kBtnLeft, x, y))  return ACT_ZERO;
      if (inside(kBtnRight, x, y)) return ACT_POWER;
      // A stray tap does NOTHING. Mis-hitting a button and being thrown back to
      // the altitude screen is worse than the tap being ignored, and BOOT
      // already closes any screen, so there is always a way out.
      return ACT_NONE;
    case UI_MENU3:
      if (inside(kBtnLeft, x, y))  return ACT_USB;
      if (inside(kBtnRight, x, y)) return ACT_UNITS;
      return ACT_NONE;
    case UI_MENU4:
      if (inside(kBtnLeft, x, y))  return ACT_CLOCK;
      if (inside(kBtnRight, x, y)) return ACT_JUMPNO;
      return ACT_NONE;
    case UI_MENU5:
#if LED_TEST_ENABLED
      if (inside(kBtnLeft, x, y))  return ACT_LOGBOOK;
      if (inside(kBtnRight, x, y)) return ACT_LEDTEST;
      return ACT_NONE;
#endif
      if (inside(kBtnWide, x, y)) return ACT_LOGBOOK;
      return ACT_NONE;
    case UI_LOGBOOK:
      // Every tap here is picking a jump or turning a page, so a stray one must
      // not throw the list away — BOOT is the way out. The detail view keeps
      // tap-to-go-back, where there is nothing else a tap could mean.
      for (uint8_t i = 0; i < 3; i++)
        if (inside(kLbRow[i], x, y)) return (uint8_t)(ACT_LOG_ROW0 + i);
      if (inside(kLbPrev, x, y)) return ACT_LOG_PREV;
      if (inside(kLbNext, x, y)) return ACT_LOG_NEXT;
      return ACT_NONE;
    case UI_JUMP_DETAIL:
      if (inside(kBtnReplay, x, y)) return ACT_REPLAY;
      return ACT_LOGBOOK;                 // any other tap goes back to the list
    case UI_REPLAY:
      // Speed lives in the top band, which is free during replay — the
      // vertical-speed readout it normally holds is already on screen as the
      // altitude itself. Anywhere else exits, because during a replay there is
      // nothing else a tap could mean, and BOOT does the same.
      if (y < ALT_TOP_BAND + 14)
      {
        if (x < DISPLAY_W / 3)          return ACT_RSPD1;
        if (x < (DISPLAY_W * 2) / 3)    return ACT_RSPD2;
        return ACT_RSPD3;
      }
      return ACT_REPLAY_EXIT;
    case UI_LED_TEST:
    case UI_SET_JUMPNO:
    case UI_SET_CLOCK:
      // Nothing here cancels: not a stray tap, and not a swipe either. Both
      // used to, and both threw away a half-finished edit. BOOT is the only
      // way out, which is the one gesture that cannot happen by accident.
      if (inside(kClkDown, x, y)) return ACT_CLK_DOWN;
      if (inside(kClkUp, x, y))   return ACT_CLK_UP;
      if (inside(kClkNext, x, y)) return ACT_CLK_NEXT;
      return ACT_NONE;
    case UI_CONFIRM_ZERO:
    case UI_CONFIRM_UNMOUNT:
    case UI_CONFIRM_USB:
    case UI_CONFIRM_POWER:
      if (inside(kBtnRight, x, y)) return ACT_CONFIRM;
      if (inside(kBtnLeft, x, y))  return ACT_CANCEL;
      // Deliberately not "anything else cancels". Cancel is the safe outcome,
      // but a tap that lands nowhere should still mean nothing — otherwise a
      // fumbled confirm silently becomes a dismissal.
      return ACT_NONE;
    default:
      return ACT_NONE;
  }
}

void sleep()
{
  if (!g_ok) return;
  g_suspended = true;
  delay(80);                                   // let any in-flight draw finish
  digitalWrite(PIN_LCD_BL, LOW);
  g_blOn = false;
  const uint8_t off[] = {BEGIN_WRITE, WRITE_COMMAND_8, 0x28,   // display off
                         WRITE_COMMAND_8, 0x10,                // sleep in
                         END_WRITE};
  g_bus->batchOperation(off, sizeof(off));
}

void wake()
{
  if (!g_ok) return;
  const uint8_t on[] = {BEGIN_WRITE,
                        WRITE_COMMAND_8, 0x11,   // sleep out
                        END_WRITE};
  g_bus->batchOperation(on, sizeof(on));
  delay(120);                                    // panel needs this after 0x11
  const uint8_t dispon[] = {BEGIN_WRITE, WRITE_COMMAND_8, 0x29, END_WRITE};
  g_bus->batchOperation(dispon, sizeof(dispon));

  // Everything on screen is stale; force a full repaint.
  g_lastBg      = 0xDEAD;
  g_lastScreen  = 0xFF;
  g_lastAlt     = INT32_MIN;
  g_lastVs      = INT32_MIN;
  g_lastBatt    = INT32_MIN;
  g_lastAltFont = nullptr;
  g_lastStr[0]  = '\0';
  g_lastTop[0]  = 1; g_lastTop[1] = 0;
  g_lastUnit    = 0xFF;

  // Re-assert the backlight. Without this the panel comes back but stays unlit:
  // the task only writes the pin on a change, and its idea of the current state
  // would still say "on" from before the sleep.
  digitalWrite(PIN_LCD_BL, HIGH);
  g_blOn        = true;
  g_suspended   = false;
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
