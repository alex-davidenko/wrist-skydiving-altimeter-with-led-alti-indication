#include "display.h"

#include "config.h"
#include "zones.h"

#if !DISPLAY_ENABLED

namespace display {
bool begin() { return false; }
void message(const char *, const char *) {}
void startTask() {}
void publish(const LedPattern &, float, float) {}
void setBattery(float) {}
void setTopText(const char *) {}
void setUnitsFeet(bool) {}
bool unitsFeet() { return false; }
void setScreen(uint8_t) {}
uint8_t screen() { return UI_ALT; }
void setBanner(const char *, const char *) {}
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
  g_lastBatt = INT32_MIN;
  g_lastUnit = 0xFF;
  g_lastAltFont = nullptr;
  g_lastStr[0] = '\0';
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
      drawPager(0, 3);
      break;

    case UI_MENU2:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnLeft,  RGB565_BLUE,  "UNMOUNT", "CARD");
      drawButton(kBtnRight, RGB565_GREEN, "DEMO",    "JUMP");
      drawPager(1, 3);
      break;

    case UI_MENU3:
      g_gfx->setCursor(20, 12);
      g_gfx->print("MENU");
      drawButton(kBtnLeft,  RGB565_BLUE,   "USB",   "DRIVE");
      drawButton(kBtnRight, RGB565_ORANGE, "UNITS",
                 st.feet ? "ft -> m" : "m -> ft");
      drawPager(2, 3);
      break;

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
  if (st.screen != UI_ALT)
  {
    if (st.screen != g_lastScreen) { renderUi(st); g_lastScreen = st.screen; }
    return;
  }
  if (g_lastScreen != UI_ALT)
  {
    // Coming back from the menu: force a full repaint of the altitude view.
    g_lastScreen = UI_ALT;
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
  // One display step is 1 m, or 10 ft — feet are rounded to 10 because at
  // 50 m/s the foot digit changes 164 times a second and is unreadable.
  const float step = st.feet ? 10.0f : 1.0f;
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
    // Vertical speed, quantised to 0.1 m/s so it is not redrawn every frame.
    // mph pairs with feet, m/s with metres — the conventional pairing.
    const int32_t vs = st.feet ? (int32_t)lrintf(st.vsMps * MPS_TO_MPH)
                               : (int32_t)lrintf(st.vsMps * 10.0f);
    if (vs != g_lastVs)
    {
      char vbuf[20];
      if (st.feet) snprintf(vbuf, sizeof(vbuf), "%+4ld mph", (long)vs);
      else         snprintf(vbuf, sizeof(vbuf), "%+6.1f m/s", vs / 10.0f);
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
  const int32_t bv = (int32_t)lrintf(st.battV * 100.0f);
  if (bv != g_lastBatt)
  {
    char bbuf[12];
    snprintf(bbuf, sizeof(bbuf), "%.2fV", bv / 100.0f);
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
  if (st.screen != UI_ALT) return true;

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
      return ACT_CANCEL;                       // anywhere else backs out
    case UI_MENU2:
      if (inside(kBtnLeft, x, y))  return ACT_UNMOUNT;
      if (inside(kBtnRight, x, y)) return ACT_DEMO;
      return ACT_CANCEL;
    case UI_MENU3:
      if (inside(kBtnLeft, x, y))  return ACT_USB;
      if (inside(kBtnRight, x, y)) return ACT_UNITS;
      return ACT_CANCEL;
    case UI_CONFIRM_ZERO:
    case UI_CONFIRM_UNMOUNT:
    case UI_CONFIRM_USB:
    case UI_CONFIRM_POWER:
      if (inside(kBtnRight, x, y)) return ACT_CONFIRM;
      return ACT_CANCEL;                       // default to the safe choice
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
