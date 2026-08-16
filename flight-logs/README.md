# Flight logs

Real jumps, exactly as the device wrote them. Nothing here is synthetic and
nothing has been cleaned up.

| file | exit | opened | freefall | avg speed | notes |
|---|---|---|---|---|---|
| `JUMP0183.CSV` | 3744 m | 808 m | 60 s | 48.9 m/s | first with per-jump logging |
| `JUMP0184.CSV` | 3639 m | 1042 m | — | — | the jump where the LED stayed dark |
| `JUMP0185.CSV` | 3778 m | 850 m | 58 s | 50.4 m/s | two mid-freefall dropouts |
| `JUMP0186.CSV` | 3785 m | 996 m | 60 s | 46.6 m/s | noisiest of the six, 17.6 m RMS |

Columns: `t_ms, p_hpa, temp_c, alt_raw_m, alt_filt_m, vs_mps, zone, phase,
ground_p`. The header carries the build and every tuning constant in play, so a
file is self-describing.

## Read them with the replay tool

```bash
c++ -std=gnu++17 -O2 -DBENCH_MODE=0 -I lib/altimeter_core -I include \
    tools/replay.cpp lib/altimeter_core/*.cpp -o /tmp/replay
/tmp/replay flight-logs/JUMP0185.CSV
```

`tools/replay.cpp` links `lib/altimeter_core` directly — the same translation
units the firmware builds — so a replay cannot silently drift from the device.

## Two things these logs will show you

**`vs_mps` is wrong, on purpose.** It records the Kalman velocity state, which
these jumps proved unusable: it peaks past 2000 m/s and reads as *climbing* for
38% of freefall. The column is kept because it is what the device computed at
the time. The firmware no longer uses it — vertical speed now comes from a
window of altitude, which is what `tools/replay.cpp` reconstructs.

**`alt_filt_m` is barely filtered in freefall.** Compare it to `alt_raw_m`: they
agree to three decimals. The adaptive process noise had inflated to its cap and
the Kalman had degenerated into a pass-through. Same bug, seen from the other
side.

Both were invisible on a desk and obvious in the first log. That is the whole
argument for keeping these files.

## And the noise

Freefall RMS runs 9–16 m against 0.22 m sitting still, and the enclosure is why:
air reaches the sensor through the printed body. Twice in `JUMP0185.CSV` the
measured altitude simply stops descending for about a second — once climbing
87 m before resuming — while body position changed the airflow. The samples
through those stretches are clean and evenly spaced at 25 ms. The pressure
really did that.

## Why there is no "max speed"

Tested against all four of these files. Peak descent is not measurable with this
sensor at any noise level yet seen — and that includes the best of them.

Taking a 3 s windowed descent rate over the freefall, against the true average:

| jump | raw noise | true | median | p90 | p95 | max |
|---|---|---|---|---|---|---|
| 183 | 4.9 m | 50.3 | **53.3** | 58.6 | 59.9 | 65.7 |
| 184 | 7.0 m | 53.5 | **53.5** | 63.8 | 69.1 | 82.7 |
| 185 | 10.6 m | 54.5 | **55.7** | 73.6 | 78.7 | 91.6 |
| 186 | 17.6 m | 50.0 | **51.2** | 75.4 | 81.1 | 128.5 |

The median holds to within 1–3 m/s of the truth across a 3.5x range of noise.
Everything above it moves with the noise instead: p90 climbs 58.6 → 75.4 as the
noise climbs 4.9 → 17.6. It is not reporting how fast the jump was, it is
reporting how leaky the enclosure was.

The max of a noisy estimator is biased upward by construction, and a longer
window reduces the bias without removing it — at 4.9 m an 8 s window still reads
58.8 against a true 50.3.

So the summaries report the average, which is robust, and no maximum, which
would be a number that changes when you tape over a hole.
