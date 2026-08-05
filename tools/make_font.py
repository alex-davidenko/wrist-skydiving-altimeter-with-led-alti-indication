#!/usr/bin/env python3
"""Rasterise a TTF into an Adafruit-GFX font header.

The altitude used to be the built-in 5x7 font magnified ~17x, which turns every
source pixel into a 17x17 block — legible, but visibly choppy. This bakes a real
typeface at the size it is actually drawn, so the edges are the font's own.

Digits are made TABULAR (one uniform advance, each glyph centred in it) so a
live-updating altitude does not jitter sideways as digits change.

Two constraints come from the GFX font format itself:
  * GFXglyph.yOffset is int8_t, so glyph height cannot exceed 127 px.
  * GFXfont.bitmapOffset is uint16_t, so one font's bitmap must stay under 64 KB.

Usage:
  .venv/bin/python tools/make_font.py > src/font_alt.h
"""

import sys
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "/System/Library/Fonts/Avenir Next.ttc"
FONT_INDEX = 0            # Avenir Next Bold
# MUST be a contiguous ASCII run. Arduino_GFX indexes glyphs as (c - first),
# so any codepoint skipped between first and last shifts every glyph after it.
# Leaving out '.' and '/' here once made every digit render as digit+2, and
# '8'/'9' index past the end of the array entirely.
CHARS = "-./0123456789:;<=>?@ABCDEFGHIJK"
# Glyphs actually drawn. Anything in CHARS but not here is emitted as a blank
# placeholder purely to keep the array contiguous — '/' in particular rises
# above cap height and drops below the baseline, so rendering it at digit size
# would breach the 127 px yOffset limit for no benefit. The run has to reach 'K'
# because high feet are shown Viso-style as "12.3K", which keeps the digits big
# instead of shrinking them to fit five of them.
USED = "-.0123456789K"
# 'K' is a suffix, not a digit: rendered at this fraction of the digit size so
# "12.3K" fits the width a four-digit number occupies. Altimeters show it that
# way too — a full-height K reads as a fifth digit.
SUFFIX_SCALE = 0.55
PAD = 40


def load(size):
    return ImageFont.truetype(FONT_PATH, size, index=FONT_INDEX)


def digit_cap_height(font):
    """Ink height of '8', which is what we actually want to control."""
    img = Image.new("L", (400, 400), 0)
    ImageDraw.Draw(img).text((PAD, PAD), "8", font=font, fill=255)
    bbox = img.getbbox()
    return (bbox[3] - bbox[1]) if bbox else 0


def size_for_cap(target):
    """Binary-search the point size whose digit ink height hits `target`."""
    lo, hi = 4, 400
    best = lo
    while lo <= hi:
        mid = (lo + hi) // 2
        if digit_cap_height(load(mid)) <= target:
            best, lo = mid, mid + 1
        else:
            hi = mid - 1
    return best


def render(font, ch):
    """Return (bitmap_rows, w, h, xoff, yoff) with offsets relative to the pen."""
    img = Image.new("L", (600, 600), 0)
    ImageDraw.Draw(img).text((PAD, PAD), ch, font=font, fill=255)
    bbox = img.getbbox()
    ascent, _ = font.getmetrics()
    if bbox is None:                       # e.g. a space
        return [], 0, 0, 0, 0
    x0, y0, x1, y1 = bbox
    w, h = x1 - x0, y1 - y0
    px = img.crop(bbox).point(lambda v: 255 if v >= 128 else 0)
    rows = [[1 if px.getpixel((x, y)) else 0 for x in range(w)] for y in range(h)]
    return rows, w, h, x0 - PAD, y0 - (PAD + ascent)


def advance_for_cap(cap):
    """Tabular advance that a given digit height produces."""
    font = load(size_for_cap(cap))
    widest = max(render(font, c)[1] for c in "0123456789")
    return widest + max(4, widest // 8)


def cap_that_fits(max_digits, width, height_limit):
    """Largest digit height whose tabular advance fits `max_digits` across."""
    cap = height_limit
    while cap > 8 and advance_for_cap(cap) * max_digits > width:
        cap -= 2
    return cap


def build(name, cap_target):
    font = load(size_for_cap(cap_target))
    glyphs, bitmap = [], []

    small = load(max(6, int(size_for_cap(cap_target) * SUFFIX_SCALE)))
    rendered = {ch: render(font if ch != 'K' else small, ch) for ch in USED}

    # Tabular: every glyph gets the widest digit's advance, and is centred in it.
    widest = max(rendered[c][1] for c in "0123456789")
    advance = widest + max(4, widest // 8)

    for ch in CHARS:
        if ch not in USED:                     # contiguity filler, draws nothing
            glyphs.append((len(bitmap), 0, 0, advance, 0, 0))
            continue
        rows, w, h, _, yoff = rendered[ch]
        # Digits keep the uniform tabular advance so a live number does not
        # jitter sideways. '.' and 'K' get their natural width instead — five
        # tabular cells would not fit, and a full-width period looks wrong.
        adv = advance if ch in "0123456789" else w + 4
        offset = len(bitmap)
        bits = [b for row in rows for b in row]
        for i in range(0, len(bits), 8):
            byte = 0
            for j, b in enumerate(bits[i:i + 8]):
                byte |= b << (7 - j)
            bitmap.append(byte)
        glyphs.append((offset, w, h, adv, (adv - w) // 2, yoff))

    if len(bitmap) > 65535:
        sys.exit(f"{name}: bitmap {len(bitmap)} bytes exceeds uint16 offsets")
    if max(g[2] for g in glyphs) > 127:
        sys.exit(f"{name}: glyph height exceeds the int8 yOffset limit")

    out = []
    out.append(f"// {name}: Avenir Next Bold, digit height {cap_target}px, "
               f"advance {advance}px, {len(bitmap)} bytes")
    out.append(f"const uint8_t {name}_bitmap[] PROGMEM = {{")
    for i in range(0, len(bitmap), 16):
        out.append("  " + " ".join(f"0x{b:02X}," for b in bitmap[i:i + 16]))
    out.append("};")
    out.append(f"const GFXglyph {name}_glyphs[] PROGMEM = {{")
    for ch, g in zip(CHARS, glyphs):
        out.append(f"  {{{g[0]:5d}, {g[1]:3d}, {g[2]:3d}, {g[3]:3d}, "
                   f"{g[4]:4d}, {g[5]:4d}}},   // '{ch}'")
    out.append("};")
    out.append(f"const GFXfont {name} PROGMEM = {{")
    out.append(f"  (uint8_t *){name}_bitmap, (GFXglyph *){name}_glyphs,")
    out.append(f"  0x{ord(CHARS[0]):02X}, 0x{ord(CHARS[-1]):02X}, "
               f"{max(g[2] for g in glyphs) + 4}}};")

    # Exact ink extents of the digits, relative to the baseline. Arduino_GFX's
    # getTextBounds derives its own baseline as yAdvance*2/3, which is flagged
    # "arbitrary" in the library and does not match real glyph extents — using
    # it put the digits off-centre and clipped. These are measured.
    digit_glyphs = [g for ch, g in zip(CHARS, glyphs) if ch in "0123456789"]
    ink_top = min(g[5] for g in digit_glyphs)
    ink_bot = max(g[5] + g[2] for g in digit_glyphs)
    out.append(f"static const AltFont k{name} = {{&{name}, {advance}, "
               f"{ink_top}, {ink_bot}}};   // ink {ink_bot - ink_top}px tall")
    return "\n".join(out), advance, max(g[2] for g in glyphs)


def main():
    span = ord(CHARS[-1]) - ord(CHARS[0]) + 1
    if span != len(CHARS):
        sys.exit(f"CHARS must be contiguous: spans {span} codepoints "
                 f"but has {len(CHARS)} glyphs")
    print("#pragma once")
    print("// GENERATED by tools/make_font.py — do not edit by hand.")
    print("//")
    print("// Real typeface baked at the size it is drawn, replacing the built-in")
    print("// 5x7 font magnified ~17x (which made every source pixel a 17x17 block).")
    print("// Digits are tabular so the altitude does not jitter as digits change.")
    print()
    print('#include <Arduino_GFX_Library.h>')
    print()
    print("// Measured metrics so layout never depends on getTextBounds.")
    print("struct AltFont")
    print("{")
    print("  const GFXfont *font;")
    print("  int16_t advance;   // uniform per-glyph advance (tabular)")
    print("  int16_t inkTop;    // topmost ink relative to the baseline (negative)")
    print("  int16_t inkBottom; // bottom-most ink, positive for round overshoot")
    print("};")
    print()
    # The band above the vertical-speed strip is 146px; cap glyphs at 118 to
    # stay clear of the int8 yOffset limit with margin.
    WIDTH, BAND = 320 - 8, 118
    # Five digits are only needed above 10,000 ft. The sizing falls out nicely:
    # the number gets BIGGER as you get lower, which is when it matters most.
    # No five-digit face: metres never exceed four digits, and high feet use
    # the "12.3K" form rather than shrinking to fit five.
    for name, digits in (("FontAltBig", 3), ("FontAltMed", 4)):
        cap = cap_that_fits(digits, WIDTH, BAND)
        body, adv, h = build(name, cap)
        print(body)
        print()
        print(f"// {name}: height {h}px, advance {adv}px -> "
              f"{WIDTH // adv} digits fit (needed {digits})", file=sys.stderr)


if __name__ == "__main__":
    main()
