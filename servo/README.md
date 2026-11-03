# STM8S003F3U6 Dual Motor Controller

A non-blocking, interrupt-free (polling-based) RC receiver controller for
two DC motors on an STM8S003F3U6, written for the Sduino/SDCC Arduino-style
core. It reads two standard RC PWM (servo) signals — throttle and steering —
and mixes them into differential-drive commands for a left/right motor pair,
with current/voltage fault monitoring and a status LED.

See also:
- [`FLOW.md`](FLOW.md) — detailed flowchart of the main loop and signal decode logic.
- [`DEBUG_STEER_LED.md`](DEBUG_STEER_LED.md) — how the steering debug LED feature works.

## 1. Pin Configuration

### RC signal inputs (from receiver)

| Pin | Function | Direction |
|-----|----------|-----------|
| `PA1` | `SERVO_IN_1` — Throttle channel | Input |
| `PA2` | `SERVO_IN_2` — Steering channel | Input |

### Motor driver outputs

| Pin | Function | Direction |
|-----|----------|-----------|
| `PC3` | `INH_PIN` — Driver enable/inhibit | Output |
| `PC6` | `M1_A` — Motor 1 (Left) direction pin | Output |
| `PD4` | `M1_B` — Motor 1 (Left) PWM/speed pin | Output |
| `PC7` | `M2_A` — Motor 2 (Right) direction pin | Output |
| `PA3` | `M2_B` — Motor 2 (Right) PWM/speed pin | Output |
| `PC5` | `LED_PIN` — Status LED | Output |

### Analog sense inputs (fault monitoring)

| Pin | Function |
|-----|----------|
| `PC4` | `IS1_PIN` — Motor 1 current sense (channel A) |
| `PD3` | `IS2_PIN` — Motor 1 current sense (channel B) |
| `PD2` | `IS3_PIN` — Motor 2 current sense (channel A) |
| `PD6` | `IS4_PIN` — Motor 2 current sense (channel B), enabled via `ENABLE_IS4_CHECK` |
| `PD5` | `VSENSE_PIN` — Supply voltage sense (divider), enabled via `ENABLE_VSENSE_CHECK` |

### Pin summary diagram

```
            STM8S003F3U6
          ┌──────────────┐
Throttle →│ PA1          │
Steering →│ PA2          │
          │       PC3    │→ INH (driver enable)
          │       PC6    │→ M1_A  (Left direction)
          │       PD4    │→ M1_B  (Left PWM)
          │       PC7    │→ M2_A  (Right direction)
          │       PA3    │→ M2_B  (Right PWM)
          │       PC5    │→ LED (status)
          │       PC4    │← IS1 (current sense)
          │       PD3    │← IS2 (current sense)
          │       PD2    │← IS3 (current sense)
          │       PD6    │← IS4 (current sense)
          │       PD5    │← VSENSE (voltage divider)
          └──────────────┘
```

## 2. How It Works

Each pass of `loop()`:

1. **Capture inputs (polling, non-blocking)** — `updateServoInput()` is
   called for `PA1` and `PA2` on every loop iteration. It detects rising and
   falling edges by comparing the current `digitalRead()` level to the
   previous one (no interrupts/`attachInterrupt`, since that approach was
   found unreliable on this SDCC/Sduino core). A rising edge marks the start
   of the pulse; a falling edge computes the pulse width and, if it's within
   `VALID_MIN_US`..`VALID_MAX_US` (800–2200 µs), stores it as the latest
   valid pulse.

2. **Fetch + validate pulses** — `getServoPulse()` returns the last captured
   pulse width for each channel, but only if it isn't older than
   `TIMEOUT_US` (30 ms) — i.e. the receiver is still sending signal.

3. **Decode to signed value** — `decodeServoSigned()` converts a 1000–2000 µs
   pulse into a signed value from **-255 to +255**:
   - Pulse `> 1550 µs` (neutral 1500 + 50 µs deadband) → positive value, scaled from 1550–2000 µs to 0–255.
   - Pulse `< 1450 µs` → negative value, scaled from 1450–1000 µs to 0 to -255.
   - Pulse within 1450–1550 µs → `0` (dead zone around center stick).

4. **Glitch debounce** — A single bad/invalid frame (e.g. an edge briefly
   missed because `analogRead()` delayed `loop()`) doesn't immediately zero
   the channel. Up to `VALID_GLITCH_CONFIRM` (3) consecutive bad frames are
   tolerated by reusing the last known-good value, preventing a visible
   brake-then-run jolt. Only after 3+ consecutive bad frames is the channel
   actually treated as lost.

5. **Mixing / driving logic** — depends on `MIX_MODE` (see Modes below).

6. **Fault checking** — `checkFault()` is run periodically (every 10 ms, not
   every loop, to avoid `analogRead()` blocking delays corrupting RC edge
   capture). It checks motor current-sense pins (and optionally supply
   voltage) against `IS_FAULT_COUNTS` / `VCC_MIN_V..VCC_MAX_V`. A fault must
   be confirmed for `FAULT_CONFIRM` (3) consecutive checks before
   `faultActive` is set, to avoid false trips from noise.

7. **Drive motors** — `driveMotor()` sets direction pin HIGH/LOW and speed
   via `analogWrite()` PWM for each motor. `INH_PIN` enables/disables the
   motor driver IC as a whole (`driversOn`).

8. **Update status LED** — blinks at 1 Hz normally, or 10 Hz (`BLINK_FAULT_MS`
   = 100 ms) when a fault is active — unless a debug mode overrides it (see
   below).

## 3. Modes

All modes are configured via `#define` compile-time flags at the top of
`servo.ino`. Set the value to `1` to enable, `0` to disable, then re-flash.

### Mixing mode — `MIX_MODE`

| `MIX_MODE` | Behavior |
|---|---|
| `1` (default) | **Differential drive / mixing mode.** `left = throttle + steer`, `right = throttle - steer` (each clamped to ±255). This is standard tank-style steering, like an RC car mixing throttle and steering into two independently-driven wheels/tracks. Invalid throttle stops both motors; invalid steering just zeros the steer contribution. `driversOn` follows throttle validity only. |
| `0` | **Independent mode.** Throttle channel drives Motor 1 directly, steering channel drives Motor 2 directly — no mixing. Each motor stops if its own channel is invalid. `driversOn` requires both channels valid. |

### Steering options

| Define | Effect |
|---|---|
| `MIX_USE_STEERING` (1/0) | When `0`, steering input is ignored entirely (`steer = 0`) — useful for throttle-only testing. |
| `MIX_STEER_INVERT` (1/0) | When `1`, inverts the steering sign (for correcting reversed wiring/channel direction without rewiring). |

### Fault handling

| Define | Effect |
|---|---|
| `ENABLE_IS4_CHECK` (1/0) | Include the 4th current-sense pin (`PD6`) in `checkFault()`. |
| `ENABLE_VSENSE_CHECK` (1/0) | Include supply-voltage monitoring (`PD5`) in `checkFault()`, tripping fault if outside `VCC_MIN_V`–`VCC_MAX_V`. |
| `FAULT_SHUTDOWN` (1/0) | When `1`, a confirmed fault forces both motors to stop (`dir`/`speed` = 0) and disables the driver (`driversOn = false`). When `0`, faults only affect the LED blink rate — motors keep running. |

Regardless of `FAULT_SHUTDOWN`, the status LED always blinks faster
(100 ms vs 1000 ms) while `faultActive` is true, so you get visual feedback
of a fault condition either way (unless a debug LED mode below is active).

### Debug modes (temporary / diagnostic, mutually exclusive)

These repurpose the status LED (`LED_PIN`) to help diagnose wiring/signal
issues and should normally be left at `0` for production use.

| Define | Effect |
|---|---|
| `DEBUG_STEER_LED` (1/0) | **Steering visualization mode.** Bypasses normal fault-blink behavior to show live steering position: LED **solid ON** when steer `> +40` (stick right), **solid OFF** when steer `< -40` (stick left), **blinking** when centered (-40..+40) or the channel is invalid/timed out. Lets you confirm `PA2` capture/decoding is working just by watching the LED while moving the steering stick. See [`DEBUG_STEER_LED.md`](DEBUG_STEER_LED.md) for full flow/state diagrams. |
| `DEBUG_LOOP_TOGGLE` (1/0) | **Loop-rate measurement mode.** Toggles `LED_PIN` once per `loop()` iteration, producing a square wave on the LED pin you can measure with an oscilloscope or frequency-capable multimeter to determine the actual loop rate. |

> Note: only one of `DEBUG_STEER_LED` / `DEBUG_LOOP_TOGGLE` should be
> enabled at a time, since both control `LED_PIN` directly and will
> conflict with each other and with normal fault-blink behavior.

## Key Tunable Constants

| Define | Default | Meaning |
|---|---|---|
| `PULSE_MIN_US` / `PULSE_MAX_US` | 1000 / 2000 µs | Expected full-travel RC pulse range. |
| `NEUTRAL_US` | 1500 µs | Center/neutral stick pulse width. |
| `DEADBAND_US` | 50 µs | Half-width of the dead zone around neutral. |
| `VALID_MIN_US` / `VALID_MAX_US` | 800 / 2200 µs | Acceptable pulse range before a reading is discarded as noise. |
| `TIMEOUT_US` | 30000 µs (30 ms) | Max age of a pulse before the channel is considered lost (no signal). |
| `VALID_GLITCH_CONFIRM` | 3 | Consecutive bad frames tolerated before a channel is actually marked invalid. |
| `IS_FAULT_COUNTS` | 600 | ADC threshold (0–1023) for current-sense fault trip. |
| `VCC_MIN_V` / `VCC_MAX_V` | 9.0 / 18.0 V | Acceptable supply voltage range (if `ENABLE_VSENSE_CHECK`). |
| `FAULT_CONFIRM` | 3 | Consecutive bad fault checks required before `faultActive` is set. |
| `BLINK_NORMAL_MS` / `BLINK_FAULT_MS` | 1000 / 100 ms | LED blink interval, normal vs. fault state. |

