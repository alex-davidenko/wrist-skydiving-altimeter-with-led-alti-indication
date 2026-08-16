# Flight logs

Real jumps, exactly as the device wrote them. Nothing here is synthetic and
nothing has been cleaned up.

| file | exit | opened | freefall | avg speed | notes |
|---|---|---|---|---|---|
| `JUMP0183.CSV` | 3744 m | 808 m | 60 s | 48.9 m/s | first with per-jump logging |
| `JUMP0184.CSV` | 3639 m | 1042 m | — | — | the jump where the LED stayed dark |
| `JUMP0185.CSV` | 3778 m | 850 m | 58 s | 50.4 m/s | two mid-freefall dropouts |

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
