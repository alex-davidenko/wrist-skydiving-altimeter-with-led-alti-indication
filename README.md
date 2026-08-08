# Wrist altimeter — v0.9.0

ESP32-S3 + MS5611 (GY-63) → filtered altitude → colour-coded 172×320 panel.

Version lives in `FW_VERSION` in `config.h` and is stamped into the boot log and
every jump log header. **1.0.0 is reserved for the first firmware that has flown
and been checked against a reference altimeter** — until then it stays 0.x,
however finished it looks on a desk.

Three flight phases, chosen automatically from vertical speed:

| phase | when | LED shows |
|---|---|---|
| **CLIMB** | rising > 2 m/s | dark, one green flash per 100 m gained |
| **FREEFALL** | descending > 20 m/s | the colour zones (green/yellow/red/blinking red) |
| **CANOPY** | anything slower | the landing ladder — dark above 300 m |

> **Not a safety device.** Single uncertified sensor, no redundancy, no
> watchdog on the altitude path. Jump with your normal analogue and audible
> altimeters. This is for ground testing and as a secondary visual aid.

---

## 1. Wiring

Target board: **Waveshare ESP32-S3-Touch-LCD-1.47**.

| GY-63 | ESP32-S3 | Notes |
|-------|----------|-------|
| VCC   | **3V3**  | header, right side |
| GND   | GND      | |
| SCL   | **GPIO41** | the board's I2C bus, shared with the touch panel |
| SDA   | **GPIO42** | |
| PS    | **3V3**    | static strap: HIGH = I2C |
| CSB   | **GPIO11** | driven LOW by firmware = address 0x77 |
| SDO   | *leave unconnected* | SPI-only pin, unused in I2C mode |

PS goes to a rail rather than a GPIO: it selects I2C vs SPI and never changes at
runtime, so a GPIO bought nothing but a window during MCU boot where the pin was
an undriven input. CSB is still driven from GPIO11 (LOW = 0x77); wire it to GND
and set `PIN_SENSOR_CSB` to `-1` if you want the sensor down to four wires — but
only if you actually wire it, since floating CSB leaves the address undefined.

**Pull-ups:** the touch panel already pulls this bus up and the GY-63 adds its
own 4.7 kΩ pair. In parallel that is ~2.4 kΩ, sinking ~1.4 mA at 3.3 V — inside
the I2C spec's 3 mA, so it works, but do not add more. If the bus misbehaves,
lifting the GY-63's pull-ups is the first thing to try.

**No user RGB LED on this board** — GPIO38 is LCD_CLK here, unlike the non-touch
`ESP32-S3-LCD-1.47`, which has a different pinout entirely. The panel is the
primary output. The build also defaults to an external WS2812 on **GPIO7**;
driving that pin is harmless if nothing is attached, and `LED_DRIVER_NONE`
silences it.

**Serial is native USB CDC** — there is one USB-C socket wired straight to the
S3, no UART bridge. `ARDUINO_USB_CDC_ON_BOOT=1` is required and already set.

**I2C addresses on the shared bus:** touch controller (AXS5106L) sits at `0x63`,
MS5611 at `0x77`. No conflict — both appear in the boot scan, and seeing `0x63`
confirms the bus is alive even before the sensor is wired.

**Restoring the factory demo.** Waveshare's download bundle ships a complete
merged image (bootloader + partition table + app — verified `0xE9` at 0x0,
`AA50` at 0x8000, same partition layout as shipped). No flash backup needed:

```bash
~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32s3 \
  --port /dev/cu.usbmodem2101 write_flash 0x0 \
  ESP32-S3-Touch-LCD-1.47-Demo/Firmware/01_factory.bin
```

Do not bother dumping the flash: `esptool read_flash` runs at ~9 KB/s on this
board regardless of `--baud`, so a full 16 MB dump takes ~31 minutes.

**Display, for when we get there:** the panel is a **JD9853**, not an ST7789.
Waveshare's Arduino example drives it with `Arduino_GFX`'s ST7789 class plus a
custom JD9853 register init (the `0xDF 0x98 0x53` unlock sequence). Pins:
SCK 38, MOSI 39, DC 45, CS 21, backlight 46; 172x320 with a 34-pixel column
offset. `GFX_Library_for_Arduino` ships in the demo bundle.

Occupied, do not use: GPIO38/39/21/45/40/46 (LCD), GPIO47/48 (touch RST/INT),
GPIO13-18 (SD card), GPIO43/44 (UART0), GPIO33-37 (octal PSRAM).
Free on the headers: GPIO1-6, GPIO7, GPIO8, GPIO10.

---

## 2. Build and flash

PlatformIO is installed in a local venv (nothing was added to your system):

Two firmware environments, same source, one flag apart:

| environment | thresholds | use |
|---|---|---|
| `bench` | divided by 1000 | testing by hand on a desk (default) |
| **`flight`** | the real ones | **this is what you jump with** |

What the flag actually changes — everything else is identical in both:

| | bench | flight |
|---|---|---|
| freefall zone thresholds | metre-scale | 800 / 1200 / 1500 / 4500 m |
| filter tuning | sigma_a 0.05, sigma_m 0.25 | 3.0 / 0.60 |
| zeroing accept limits | tighter | looser |
| flight-phase machine + landing ladder | not compiled | active |
| weather-drift auto-zero | **not compiled** | active |
| idle light sleep | active | active |

Auto-zero is flight-only because its constants are absolute metres: a 1 m settle
band spans three bench zones, so a board held still would appear to walk down
through them. Idle sleep is in both — nothing about it is scale-sensitive, and
the bench is where current draw gets measured.

```bash
.venv/bin/pio run -e flight --target upload && .venv/bin/pio device monitor
```

In VS Code: PlatformIO sidebar → Project Tasks → **flight → General → Upload
and Monitor**.

They are separate environments rather than separate branches deliberately: a
production branch would diverge from this one and need merging forever, while a
build flag cannot drift. Both are built and tested on every change.

The boot log and the boot banner both name the mode, so which build is on the
board is never a guess.

**If the link step fails** with `undefined reference to _cleanup_r` or
`_Unwind_SetEnableExceptionFdeSorting`, the Xtensa toolchain package is
corrupted — its manifest claims 8.4.0 while the directory actually holds GCC
14.2.0, so Arduino 2.0.17's precompiled libs get linked by the wrong compiler.
Repair with:

```bash
.venv/bin/pio pkg install -g -t "espressif/toolchain-xtensa-esp32s3@8.4.0+2021r2-patch5" -f
```

Run the host-side logic tests (no hardware needed):

```bash
.venv/bin/pio test -e native
```

---

## 3. Bring-up order

1. **Power only.** Plug the board in with nothing else attached. The panel
   should light and show the boot message. That confirms the display path
   before you trust any colour it shows you later.
2. **Wire the GY-63, then watch the boot log.** You should see the I2C scan
   report a device at `0x77` (or `0x76`), then `MS5611 found at 0x…`.
   If the scan is empty: check PS, check 3V3/GND, check SDA/SCL are not swapped.
   If the sensor is not found, the screen goes magenta and the LED (if fitted)
   blinks fast magenta — the sensor-fault pattern, deliberately impossible to
   confuse with any altitude colour.
3. **Check the pressure is not exactly half.** See §6 — this board needs
   `SENSOR_MATH_MODE 1`, and the firmware now warns loudly at boot if the
   reading is outside 500–1085 hPa.
4. **Check the CSV stream.** Columns are:
   `t_ms, p_hpa, temp_c, raw_m, filt_m, vs_mps, sigma_m, zone`.
   Sitting still, `p_hpa` should be ~950–1030 near sea level, and `filt_m`
   should be far quieter than `raw_m`. That is the filter working.
5. **It zeroes itself at power-up**, while the boot animation plays — the same
   thing a Viso does. Type `z` or hold BOOT for 1.5 s to redo it by hand. Either
   way it refuses, and says why, if the board was moving during the average.

   That trusts you to switch it on **on the ground**. Power it up inside a
   climbing aircraft and it will call that altitude zero. No barometric
   altimeter that self-zeroes can tell the difference; `BOOT_ZERO_ENABLED`
   turns it off if you would rather zero by hand.

---

## 4. Serial commands

| key | action |
|-----|--------|
| `z` | zero here (averages 80 samples, ~2 s) |
| `r` | clear zero, back to raw ISA pressure altitude |
| `q <hPa>` | set QNH, e.g. `q 1017.4` |
| `b <0-255>` | LED brightness |
| `t` | LED colour sweep |
| `p` | play every pattern — climb flash, all zones, landing rates, fault |
| `u` | toggle feet / metres (also MENU page 3) |
| `c` | toggle serial CSV streaming |
| `l` | toggle SD logging |
| `s` | status |
| `?` | help |

The ground reference and QNH are stored in flash and survive a reboot — useful
if the board browns out in the aircraft. It still tells you to re-zero on boot,
because a reference from yesterday's weather is worse than none.

---

## 5. Bench test procedure

Bench mode uses the flight thresholds divided by 1000:

| height above zero | LED |
|-------------------|-----|
| ≥ 1.50 m | green |
| 1.20 – 1.50 m | yellow |
| 0.80 – 1.20 m | red |
| 0.15 – 0.80 m | blinking red |
| < 0.15 m | off |

Put the board flat on the desk, press `z`, then lift it. Hold at **0.05 /
0.50 / 0.95 / 1.35 / 2.00 m** — those sit in the middle of each band.

Two things worth knowing before you start:

- **Test in the middle of a band, not on a boundary.** Hysteresis means a
  height sitting exactly on a threshold legitimately depends on which
  direction you approached from. That is the feature working, not a bug.
- **A zone change takes ~0.5-0.75 s after a sharp move.** That is mostly the
  300 ms dwell timer plus the filter settling. It is intentional; see §6.

Air conditioning, opening a door, or an elevator in the building will all move
the reading by more than a metre at this scale. If the numbers wander, that is
usually real pressure, not the sensor.

---

## 5a. Flight phases and the landing ladder

*(FLIGHT builds only — bench keeps the plain colour ladder, unchanged.)*

The freefall colour zones only mean something in freefall. A good canopy at
900 m does not warrant a red light, so below 20 m/s of descent the display
switches to a landing ladder:

| altitude | under canopy |
|---|---|
| above 300 m | dark |
| 200 – 300 m | green, 3 blinks/s |
| 100 – 200 m | green, 6 blinks/s |
| 10 – 100 m | **bright steady green** (full output, ignores the brightness setting) |
| below 10 m | dark — assistance finished |

The 20 m/s split is deliberate: a **high-speed malfunction is still descending
fast**, so it stays in FREEFALL and keeps the red / blinking-red warnings.
That case is pinned down by `test_high_speed_malfunction_keeps_freefall_warnings`.

Enter and exit thresholds differ (20 / 15 m/s) so a descent hovering near the
split cannot flap the entire meaning of the LED.

**Caveat:** a hard spiral under a small canopy can genuinely exceed 20 m/s and
will flip the display back to freefall colours. Raise `MODE_FREEFALL_ENTER_MPS`
if you spiral hard.

**Also expect** the LED to stay dark for the first few seconds after exit —
you need to accelerate past 20 m/s before FREEFALL engages.

Use `p` on the console to eyeball every pattern on the bench without flying.

### Reading logs over USB-C

**MENU -> swipe twice -> USB DRIVE** reboots the device as a plain USB mass
storage device, so the card can be read without pulling it.

This is a boot mode rather than a runtime toggle, deliberately. The firmware and
the host cannot both own a FAT filesystem — two writers corrupt it — so entry
closes the log, sets a flag and restarts. The next boot consumes the flag and
comes up as a drive with the altimeter not running at all.

The flag lives in `RTC_NOINIT` memory, which survives a software reset but not a
power cycle, and is cleared *before* anything that could fail. That gives the
recovery property for free: **any press of RST returns to normal operation**,
and no crash in USB mode can trap the device there. Normal boot is always the
serial console; reading logs is the thing that takes deliberate effort.

In this mode the card is driven at the block level through the ESP-IDF sdmmc
API rather than SD_MMC. MSC is a block protocol — the host supplies the
filesystem — so mounting FAT on the device would be pointless and would risk
two owners again.

Eject on the host, then press RST.

### Demo jump

**MENU -> swipe left -> DEMO JUMP** replays a whole jump on the screen in real
time: 4200 m exit, pilot chute at 1100 m, settled by 800 m, five
5-second spirals at 15 m/s, then the landing ladder from 300 m down.

It is driven through the *real* ZoneTracker, landing ladder and
FlightModeTracker — its own instances, so live state is untouched — and the
pattern is built exactly the way the flying firmware builds it. So it
demonstrates the actual logic rather than an animation of it, and a bug in the
thresholds would show up here. Sampling and logging carry on underneath; any
touch or BOOT press ends it.

It runs in **real time** (`DEMO_SPEED 1.0`), so the whole profile takes about
3.5 minutes. That is the point — it shows how fast the altitude digits actually
change in freefall and whether that is readable, which a compressed replay
cannot tell you.

Two things it shows that are easy to forget:

- **The screen is dark for the first few seconds after exit.** You leave the
  aircraft at zero vertical speed, so the device is in CANOPY mode until you
  accelerate past 20 m/s. Verified: green appears about 4 s in.
- **15 m/s spirals stay under canopy.** They sit between the 15 m/s exit and
  20 m/s entry thresholds, so the display does not flip back to freefall
  colours — which is exactly what the hysteresis is there for.

---

## 6. How the signal path works

**Altitude.** ISA barometric formula. AGL is computed as a *difference* of two
pressure altitudes (current and the stored ground pressure), not by re-anchoring
the formula at ground level. That keeps AGL almost independent of QNH error —
`test_agl_is_qnh_insensitive` pins this down.

**Filter — 2-state Kalman (altitude + vertical speed), not a moving average.**
A boxcar of N samples lags a ramp by (N−1)/2 samples; at 40 Hz and 50 m/s
freefall a 25-tap average sits ~15 m behind the truth, which is a real error at
the 800 m threshold. A constant-velocity Kalman has **zero** steady-state lag on
a ramp because the velocity state absorbs it. Measured freefall tracking error:
0.43 m.

**Adaptive process noise.** A constant-velocity model has one nasty failure:
it *coasts*. When real motion stops, the velocity state is still wound up, so
the estimate overshoots and bleeds off slowly. Measured on simulated profiles:

| | plain CV Kalman | with adaptive gate |
|---|---|---|
| canopy opening (50→5 m/s), peak error | 3.50 m | **0.81 m** |
| canopy opening, settling time | 0.50 s | **0.10 s** |
| bench lift-and-hold, settling time | 3.95 s | **0.12 s** |
| freefall ramp tracking error | 0.43 m | 0.43 m (unchanged) |

The detector is deliberately *not* a single-sample outlier gate. It cannot be:
the innovation covariance `S = P00 + R` is dominated by sensor noise, so a coast
error of a few tenths of a metre sits inside 1σ and any instantaneous gate stays
silent. What actually gives it away is that the innovations stop looking
zero-mean and acquire a persistent sign — so the filter tracks a slow EMA of the
innovation and inflates the covariance when that EMA stands out from its own
noise floor.

**Hysteresis and dwell are asymmetric in flight.** Dropping a band means danger
increasing and commits almost immediately; climbing back is damped hard. A
safety indicator should fail toward urgency. Measured on a 50 m/s descent
through the 800 m threshold:

| | LED lights |
|---|---|
| symmetric (5 m hyst, 120 ms dwell) | 11.2 m late (225 ms) |
| **urgent-asymmetric (0 m, 50 ms)** | **~2-3 m late** |

The urgent dwell is 50 ms rather than 25 ms because 25 ms false-alarms:
injecting one 30 m bad read every 50 s while hovering above a threshold produced
**24 spurious blink-reds in 20 minutes at 25 ms, and zero at 50 ms**. A false
blink-red is a safe failure, but false alarms destroy trust in the indicator,
which is its own safety problem.

One consequence worth knowing: an altitude parked exactly on a threshold now
settles into the *lower* band and stays there. That bias is intentional.

Bench mode keeps symmetric damping — the asymmetry is a flight behaviour, and at
desk scale it would just make the reading sit one band low near a boundary.

**Hysteresis and dwell** are separate mechanisms because they stop different
things. Hysteresis (overshoot required to leave a zone) kills flicker from noise
on a slowly drifting altitude. Dwell (must stay out of the committed zone before
a change commits) kills flicker from single-sample I2C glitches large enough to
clear the hysteresis band. Cost at the 1200 m threshold in freefall: the red zone
triggers ~11 m late, which is 0.2 s at 50 m/s.

**The dwell clock measures time since leaving the committed zone** — not time
the candidate has held one particular value. That distinction is worth keeping.
On a sharp move the filter briefly overshoots past its target, so the candidate
can read GREEN → RED → YELLOW while it settles. Restarting the clock on every
candidate change made settling time and dwell *compound*: a sharp 2.2 m → 1.35 m
bench move took ~1.0-1.45 s to show yellow instead of the ~0.5 s the dwell alone
should cost. Spike rejection is unaffected, because a transient that returns to
the committed zone still cancels the clock.

Measured on that move after the fix, and why the dwell is 300 ms:

| dwell | sharp move response | min gap between changes, parked |
|---|---|---|
| 500 ms | 0.73 s | 87 s |
| **300 ms** | **0.53 s** | **87 s** |
| 250 ms | 0.48 s | 1.6 s ← chatter |

**Sensor maths — the factor-of-two trap.** The MS5611 and MS5607 dies use
different scaling on the two dominant calibration terms (`SENSt1` is C1·2¹⁵ vs
C1·2¹⁶, `OFFt1` is C2·2¹⁶ vs C2·2¹⁷). Both differ by exactly 2×, and pressure is
linear in both, so the wrong choice yields a pressure exactly half or double the
truth. Many modules sold as "GY-63 MS5611" carry an MS5607 or clone die.
`MS5611::begin()` hardcodes mathMode 0, so `sensorBegin()` calls `reset()`
directly instead — `SENSOR_MATH_MODE` in `config.h` selects it, and this board
needs **1**.

This is worth understanding because of *how* it fails: the trace stays smooth,
stable, low-noise and entirely self-consistent. The filter converges beautifully.
It is simply wrong by kilometres. `checkPressurePlausible()` now catches it at
boot and tells you what doubling or halving would give.

**Zeroing acceptance checks.** Two separate tests, because scatter and movement
are different problems:

- *sigma* (single-sample scatter) — catches a failing sensor. Set generously,
  because normal scatter is large and harmless; it averages out.
- *drift* (first-half mean vs second-half mean) — catches the board actually
  being moved mid-average. Noise cancels in a mean; a trend does not. This can
  be tight, since each half-mean has a standard error of only ~0.035 m.

Do **not** replace these with a peak-to-peak spread test. Peak-to-peak grows
with sample count and is dominated by tails: at the MS5611's real ~0.22 m
single-sample sigma, 80 samples span 0.9–1.3 m routinely. Any peak-to-peak limit
tight enough to catch a moving board also rejects every good calibration. (This
build shipped with exactly that bug; it rejected every zeroing attempt.)

**The panel renders on the other core.** A full-screen fill measured
**23,696 us** against a 25 ms sample period — close enough that drawing from
the sample loop cost exactly one dropped sample per zone change, and would have
cost ~28% of the loop once the screen started blinking at 6 Hz (12 fills/s).
So the renderer is a FreeRTOS task pinned to core 0 while the Arduino loop runs
on core 1. The sample loop only calls `display::publish()`, a spinlock-protected
struct copy, and never touches SPI.

The screen is driven from the same `LedPattern` the LED uses, so both outputs
agree by construction rather than by a second colour table kept in step by hand.

**Boot sequence.** Panel up, sensor up, then the eyes: fade in, two blinks, the
smile snaps closed, fade out. Then the banner fades up with *Setting the ground
zero*, its three dots cycling once every 350 ms while the boot zero runs, and
*Flight mode activated* fades in beneath it when the zero lands. Around seven
seconds from power-up to a live altitude.

The dots are driven from the zero's own sample loop — `zeroHere()` takes a tick
callback — rather than from a timer running alongside it. A timer would keep
animating through a stall, or stop while the device was still working; this
cannot. What is on screen is what it is doing.

Text that can shrink has to be space-padded to a constant width. The banner
draws with an opaque background, so each pass repaints its own glyph boxes, but
only the boxes it is given: drop from three dots to none and the old tail would
otherwise stay lit. The animation is also what pays
for the sensor's warm-up settle — `BOOT_ZERO_SETTLE_MS` is shorter than
`FILTER_SETTLE_MS` precisely because the graphics have already run.

Powering off runs the mirror image: the eyes fade up, blink twice slowly, then
close for good and fade to black before the panel sleeps and the S3 does. That
one runs with the renderer already live on the other core, so it takes the panel
through the same suspend handshake `display::sleep()` uses rather than drawing
into a race. Idle light sleep does *not* play it — it happens every 30 s.

The altitude is drawn in a real typeface (Avenir Next Bold) baked at the size it
is actually displayed. It was previously the built-in 5x7 font magnified ~17x,
which turns every source pixel into a 17x17 block — legible but visibly choppy.
`tools/make_font.py` rasterises the glyphs into GFX font data:

```bash
.venv/bin/python tools/make_font.py > src/font_alt.h
```

Two sizes are baked and chosen at runtime by digit count — 116 px for up to
three digits, 86 px for four — because a GFX font cannot be scaled without
going blocky again, and `GFXglyph.yOffset` is `int8_t`, capping any glyph at
127 px tall.

The altitude does not go through Arduino_GFX's text drawing at all. That path
is flagged unreliable by the library itself ("may introduce many ugly output,
it should limited using on mono font only") and produced two distinct faults:
it derives a baseline as `yAdvance * 2 / 3`, marked `TODO ... arbitrary` in the
source, so `getTextBounds` mis-positioned the digits and clipped them; and its
opaque-background fill sits on that same fictional baseline, missing the top
third of a tall glyph and stranding fragments of the previous number.

Instead each digit is composited into a PSRAM scratch cell and pushed with one
`draw16bitRGBBitmap`. Drawing to the panel directly means erasing the old glyph
and then drawing the new one, leaving the digit blank for milliseconds in
between — at 20 Hz that reads as constant flicker. Compositing off-screen
removes the intermediate state entirely, and positioning comes from the
generator's measured ink extents rather than the library's guess.

Only digits that actually changed are redrawn, so a typical update touches one
cell: 2.6 ms, about 5% of the render core. The band is cleared only when the
digit count or typeface changes and every cell moves. The generator sizes each font to the widest string it must fit
rather than to a guessed height. Digits are tabular (one uniform advance, each
glyph centred in it) so a live-updating altitude does not jitter sideways as
digits change. Cost is 18 KB of flash.

There is no zone label: the background colour already says which zone you are
in, so the text was redundant.

**Idle sleep, and the settling transient.** Off USB the device light-sleeps
after 60 s of stillness below 25 m, waking every 30 s to check; a BOOT press
wakes it and shows the panel for 20 s.

Getting the 20 s to hold took finding a non-obvious fault. The panel stayed lit
for 60 s after every wake instead, because the idle gate treats > 0.5 m/s as
motion and restarts the quiet timer — and the sensor genuinely produces that
after a reset. Measured: the die warms 26.4 → 27.1 °C over the first seconds,
raw altitude drifts a full metre downward, and the filter reports up to
**1.02 m/s of descent**. Real signal, correctly detected. So `FILTER_SETTLE_MS`
suspends the velocity gate for 6 s after any filter reset; the altitude gate
keeps working throughout.

Two things this cost, worth recording. Four hypotheses drawn from reading the
source were wrong. And a simulation *excluded the true cause* — it fed a
stationary altitude, so it contained no thermal drift to reproduce, and its
"ruled out, do not re-check" verdict was written into `config.h` directly over
the answer. One field of on-device trace named it in a single run. **Simulate
the physics you have modelled; trace the physics you have not.** `IDLE_TRACE`
in `config.h` is that trace.

The same transient had a second, quieter effect: it broke auto-zero completely.
`AUTOZERO_SETTLED_MPS` is also 0.5 m/s, so every 30 s wake restarted the settle
timer, which needs 3+ minutes to mature — it could never get there, so the
correction never ran and a standing offset lived forever. Meanwhile a zero taken
seconds after power-up stored a pressure the sensor was about to leave, putting
the offset there in the first place. One transient, two symptoms, and the
visible result was a device that sat at −1 m and stayed. `zeroHere()` now waits
for the sensor before averaging, and the velocity is suppressed in both gates.
`test_periodic_wake_transient_does_not_starve_autozero` pins the starvation down.

**Sampling** is 40 Hz. The library's `read()` is blocking and does a pressure
conversion followed by a temperature conversion — ~20 ms total at
`OSR_ULTRA_HIGH` — so 40 Hz is near the practical ceiling. CSV output is
decoupled at 10 Hz so logging never gates the control loop. The loop counts its
own overruns; check `s` if you suspect it is not keeping up.

---

## 7. Layout

```
include/config.h              all tuning knobs — pins, thresholds, filter
src/main.cpp                  sampling loop, calibration, serial console
src/led.{h,cpp}               RGB output; NeoPixel or discrete RGB
lib/altimeter_core/
  baro_math.{h,cpp}           pressure <-> altitude
  altitude_filter.{h,cpp}     Kalman + adaptive process noise
  zones.{h,cpp}               hysteresis + dwell state machine
test/test_core/               18 host-side tests
```

`lib/altimeter_core` is deliberately Arduino-free so the logic that is
miserable to debug on hardware runs on the host in under a second.

---

## 7a. Jump logging and replay

Every sample goes to `/LOGnnnn.CSV` on the card, a new file per power-up.
Toggle with `l`; `s` reports the filename, row count and any drops.

The log stores **both the raw sensor values and the derived ones**. The derived
columns are recomputable, so they look redundant — but they are what makes an
offline replay trustworthy. Recompute from the raw, compare against what the
device actually produced in the air: if they match, the tooling is sound and
tuning experiments mean something; if they diverge you have found a bug or a
firmware mismatch *before* drawing conclusions. That check is impossible if only
raw is stored. For the same reason the header records the build and every
tuning constant in play, including QNH and the ground reference, so the file is
self-contained.

SD writes stall unpredictably — a card doing wear levelling can block 100 ms,
which would reintroduce the loop overruns the second core just removed. So
`push()` only appends to a lock-free SPSC ring in PSRAM (32k records, ~13
minutes of slack) and a writer task does all the blocking I/O. If the ring ever
does fill, the newest sample is dropped and counted rather than stalling the
sample loop: a gap in the log is recoverable, a stalled altimeter is not.

```bash
c++ -std=gnu++17 -O2 -I lib/altimeter_core tools/replay.cpp \
    lib/altimeter_core/*.cpp -o /tmp/replay
/tmp/replay /Volumes/<card>/LOG0001.CSV
```

`tools/replay.cpp` links `lib/altimeter_core` directly — the same translation
units the firmware builds — so replay cannot silently drift from the device.
It prints a verification block first (baro maths, filter and zone agreement),
then the jump summary: apogee, exit, freefall time and distance, opening
altitude, canopy time, peak descent rate, and time spent in each colour zone.
Validated against a synthetic 42,155-row jump: filter reproduced to 0.0015 m
with zero zone mismatches.

---

## 7b. Power, measured

With a multimeter in series with the cell, and the two hard-wired PWR LED
resistors desoldered:

| state | current | on 1000 mAh |
|---|---|---|
| active, backlight on | 110 mA | ~9 h |
| idle light sleep, floor | 2.7 mA | — |
| idle light sleep, average | ~3-5 mA | ~10 days |
| deep sleep (auto-off) | **765 uA** | ~54 days |

A realistic jump day — 16 h idle plus six jumps at ~20 min of active display —
comes to about **285 mAh**, comfortable on a 1000 mAh cell.

Two results worth keeping, because both close off work that looked worthwhile:

**Power-gating the sensor through a GPIO is not worth doing.** Unsoldering the
MS5611 entirely during deep sleep moved the draw from 1.2 mA to 1.1 mA. The
sensor is 0.1 mA. A switched supply would recover that 0.1 mA and add a warm-up
transient on every wake — the same transient documented in §6, which has already
cost two bugs.

**The 1.1 mA floor is peripherals, not the MCU.** An ESP32-S3 in deep sleep draws
10-20 uA, so essentially all of it is parts that stay powered while the CPU is
off. The SD card was measured and ruled out at **50 uA**. What remains splits in
two, and the distinction is the difference between a firmware fix and a respin:

*Reachable from software, and measured.* `PIN_TOUCH_RST` (GPIO47) and
`PIN_LCD_RST` (GPIO40) both float during deep sleep, leaving those controllers
running. Neither pin is on the headers, so this was tested in firmware:
`DEEPSLEEP_HOLD_TOUCH_RST` and `DEEPSLEEP_HOLD_LCD_RST` in `config.h` drive a
reset low before `esp_deep_sleep_start()` and latch it with the digital-pad hold
(both pins are outside the RTC bank, GPIO0-21, so `rtc_gpio_hold` does not
apply). Results:

| holds | deep sleep |
|---|---|
| none | 1200 uA, jittering 1150-1250 |
| touch + LCD | 950 uA |
| **touch only** | **765 uA** — shipped |

Touch is worth **435 uA**, and the jitter that disappeared with it was the
AXS5106L's scan cycle — it is self-clocked and keeps scanning with nothing
listening. Waveshare's driver has no power management for that part at all, so
the reset line is the only lever short of the datasheet.

**The LCD hold measured 185 uA worse than doing nothing**, and that is the
instructive one. `display::sleep()` already sends `0x28` and `0x10`, so the panel
was properly asleep before its reset was ever touched; the hold added nothing and
drew current through the pull-up on that line for the privilege. Holding a reset
low is not free, and it is only worth it when the part has no sleep path of its
own.

The hold survives the wake, so `setup()` releases both before anything touches a
peripheral — without that the panel and touch stay in reset for the whole
session. That release is unconditional, since the previous boot may have been a
build with holds on.

*Not reachable.* Regulator quiescent and the charge IC. If those turn out to be
the residue, that is the floor for this PCB, and getting under it means a board
with a high-side load switch over everything outside the RTC domain plus an LDO
with single-digit-uA quiescent.

Neither pin is broken out on the headers, so this is done in firmware rather
than with a jumper: `DEEPSLEEP_HOLD_TOUCH_RST` and `DEEPSLEEP_HOLD_LCD_RST` in
`config.h` drive each reset low before `esp_deep_sleep_start()` and latch it
with the digital-pad hold. They are independent so the draw can be attributed —
flash with both 0 for the baseline, then one at a time.

The hold survives the wake, so `setup()` releases both before anything touches a
peripheral; without that the panel and touch controller stay in reset for the
whole session. That release is unconditional, since the previous boot may have
been a build with the holds on.

One caveat on interpreting the result: driving a reset low costs current back
through any pull-up on the line — a 10k to 3V3 is 0.33 mA. So a *small*
improvement does not exonerate the controller, it means the reset pin is the
wrong lever and the part's own sleep command is the right one.

## 8. Where this is up to

Everything below is on hardware and working: sensor, filter, zones, flight
phases, panel, touch menu, SD logging with verified offline replay, USB drive
mode, deep sleep, idle light sleep, weather-drift correction, and a full
real-time demo jump.

**The next milestone is not software. It is one logged jump.** Every tuning
number in this project came from simulation against assumed noise. A single
jump with the logger running, and your handheld's readings noted at exit,
opening and a couple of points under canopy, replaces all of it with
measurement — and answers the one question nothing here can: how big the
airflow error actually is (see §9).

### Measured on hardware, for the record

| | |
|---|---|
| raw sensor noise | 0.14-0.22 m RMS |
| filtered noise, bench tuning | 0.037 m RMS, ~7x reduction |
| filtered noise, flight tuning | 0.019 m RMS, ~7.5x reduction |
| pressure quantisation | 1 Pa = 0.084 m |
| zero repeatability | +/-0.021 m |
| full-screen panel fill | 23.7 ms at 40 MHz SPI |
| one digit blit | 2.6 ms |
| current, panel active | **110 mA**, backlight on (meter in series) |
| current, idle light sleep | **2.7 mA** floor; ~3-5 mA average incl. 30 s wakes |
| current, deep sleep | **765 uA** with touch held in reset (was 1.2 mA) |
| deep sleep, sensor unsoldered | 1.1 mA — the MS5611 is only 0.1 mA of it |
| sensor temperature, on-board | 38 C — the reason it was moved |
| sensor temperature, on wires | **27 C** — thermal plume resolved |
| post-reset warm-up drift | 1 m over ~1 s, reads as 1.02 m/s (see §6) |

### Immediate next steps

1. ~~Measure idle sleep current~~ and ~~desolder the PWR LEDs~~ — **both done**,
   see the table above and §7b. Idle sleep is real: 110 mA -> 2.7 mA.
2. ~~Attribute the deep-sleep floor~~ — **done**, see §7b. 1.2 mA -> 765 uA by
   holding the touch controller in reset. The remainder is regulator and charge
   IC, which firmware cannot reach.
3. ~~Move the GY-63 off-board on wires~~ — **done**, sensor now reads 27 C
   instead of 38 C. The static port still has to go there.
4. **Jump it.** Flight build, zeroed at the DZ, logging on.

### Still planned

- **Menu with stats and jump history**, now that logging exists to feed it.
- **Charge management.** The board has VBAT and onboard charging; the firmware
  reads the battery but does nothing about charging state.

## 9. Not done yet

- **LED is optional and unused on this board.** The panel is the primary
  output. A sunlight-readable external LED would need proper constant-current
  drivers, not GPIO — but the screen may make it unnecessary.
- **`ZONE_ABOVE` (above 4500 m) shows dim blue.** Guessed, not specified —
  change in `led::colorFor`.
- **The LED-off band is disabled in flight mode**, so it blinks red all the way
  down. If you want it dark after landing, set the first flight boundary in
  `config.h` to ~15 m.
- **Sensor self-heating.** The MS5611 reads 38 C sitting on this board with the
  panel running, versus 28 C on the C3. Steady-state accuracy is fine, but the
  MS5611's compensation assumes the die and the pressure element are at the
  same temperature, and during a fast descent from a 38 C board to sub-zero
  ambient they are not. Mount the GY-63 off-board on wires, away from the S3
  and the backlight. That is also where the static port needs to go.
- **Airflow error is completely uncharacterised**, and it is almost certainly
  the dominant error in freefall — far larger than the ~2-3 m of trigger
  latency. At 50 m/s dynamic pressure is ~14 hPa; full stagnation would be
  ~120 m of error, and even 10-30% coupling on a wrist is tens of metres,
  varying as the arm moves or the body spins. Needs a proper static port
  (sealed chamber, vent perpendicular to flow, foam over the opening) and a
  real jump logged against a handheld altimeter before any of this is
  trustworthy in an emergency.
- **No BMP581 support.** When it arrives, `AltitudeFilter` and `ZoneTracker`
  take metres and know nothing about the sensor, so it should only mean a new
  reader feeding `aglFromPressure`.
- **Vibration untested.** MS5611 pressure ports are sensitive to airflow over
  the opening; on a wrist in freefall this may matter more than sensor noise.
  Worth a foam vent cover and a real jump comparison against the handheld.
