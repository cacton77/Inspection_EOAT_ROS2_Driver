# Firmware Migration: Feather RP2040 → QT Py RP2040

## Context

You are modifying the Arduino firmware in `firmware/macro-ps/` of the `Inspection_EOAT_ROS2_Driver` repository. The hardware platform is changing from an **Adafruit Feather RP2040** to an **Adafruit QT Py RP2040**, and the stepper driver is changing from a generic TMC2209 breakout to a **BIGTREETECH TMC2209** step-stick module.

The firmware currently has a working IMU subsystem (LSM6DSOX over I2C, publishing to micro-ROS via USB CDC) and stub modules for LEDs, stepper, joystick, buttons, and XVS sync. **This iteration implements the LED ring and stepper control modules.** Joystick, buttons, and XVS remain as stubs with updated pin defines.

The repo README contains the full design spec for all subsystems. Use it as the authoritative reference for behavior, state machine logic, NeoPixel chain layout, joystick-to-illumination mapping, homing sequence, and micro-ROS interface definitions. The instructions below cover only what must *change* — everything not mentioned here should be carried over from the README spec as-is.

---

## 1. Pin Remapping (Old → New)

All pin defines live in `config.h`. Every GPIO number below refers to the RP2040 GPIO, not the Arduino alias.

### Board-level changes

| Item | Feather RP2040 | QT Py RP2040 |
|---|---|---|
| Board variant (FQBN) | `rp2040:rp2040:adafruit_feather` | `rp2040:rp2040:adafruit_qtpy` |
| Onboard NeoPixel data | GP16 | GP12 (`PIN_NEOPIXEL`) |
| Onboard NeoPixel power | none (-1) | GP11 (`NEOPIXEL_POWER`) |
| STEMMA QT I2C bus | `Wire` (GP2/GP3) | `Wire1` (GP22/GP23) |
| Serial1 UART | UART0 (GP0/GP1) | UART1 (GP20/GP5) |

### Per-peripheral pin reassignment

| Signal | Old GPIO | New GPIO | QT Py Silk | Notes |
|---|---|---|---|---|
| NeoPixel data | GP18 | GP24 | SDA / D4 | PIO-driven, unchanged semantics |
| TMC UART TX | GP4 | GP20 | TX / D6 | Now Serial1 (was Serial2). 1 kΩ series resistor to PDN_UART. |
| TMC UART RX | GP5 | GP5 | RX / D7 | Same GPIO, but now Serial1 RX (was Serial2 RX). Direct to PDN_UART. |
| STEP | GP10 | GP6 | SCK / D8 | PWM3A for hardware pulse gen |
| DIR | GP9 | GP4 | MI / D9 | Digital output |
| EN | GP11 | GP3 | MO / D10 | Active HIGH = disabled |
| Joystick X | GP26 (A0) | GP29 (A0) | A0 / D0 | Arduino alias `A0` still works; underlying GPIO changed |
| Joystick Y | GP27 (A1) | GP28 (A1) | A1 / D1 | Arduino alias `A1` still works |
| NeoKey 1 switch | GP28 (A2, was Mag+) | GP27 | A2 / D2 | Digital input, internal pullup, active LOW |
| NeoKey 2 switch | GP29 (A3, was Mag−) | GP26 | A3 / D3 | Digital input, internal pullup, active LOW |
| NeoKey 3 switch | GP6 (was PS btn) | GP25 | SCL / D5 | Digital input, internal pullup, active LOW. This pin was freed by removing DIAG from dedicated GPIO. |
| DIAG | (was implicit, not GPIO) | N/A | — | No dedicated GPIO. Poll `SG_RESULT` register over UART instead. |
| XVS sync | GP12 | N/A | — | No pin available (GP12 is now onboard NeoPixel). Remains a stub. |
| IMU SDA | GP2 | GP22 | STEMMA SDA1 | Wire → Wire1. |
| IMU SCL | GP3 | GP23 | STEMMA SCL1 | Wire → Wire1. |
| Status LED | GP16 | GP12 | (onboard) | Power enable pin GP11 must be driven HIGH. |

---

## 2. File-by-File Changes

### 2.1 `config.h` — Update all pin defines and board comments

Apply every pin change from the table above. Specific edits:

```
// Board target → QT Py RP2040

// I2C for IMU — QT Py STEMMA QT uses Wire1 (GP22/GP23)
#define I2C_SDA_PIN              22
#define I2C_SCL_PIN              23

// Status LED — QT Py onboard NeoPixel
#define STATUS_LED_PIN           12    // GP12
#define STATUS_LED_POWER_PIN     11    // GP11 — must be driven HIGH to power NeoPixel

// External NeoPixel chain
#define NEOPIXEL_DATA_PIN        24    // GP24 (SDA/D4)

// TMC2209 — uses Serial1 on QT Py (UART1: TX=GP20, RX=GP5)
#define TMC_SERIAL               Serial1
#define STEPPER_STEP_PIN         6     // GP6 (SCK/D8), PWM3A
#define STEPPER_DIR_PIN          4     // GP4 (MI/D9)
#define STEPPER_ENABLE_PIN       3     // GP3 (MO/D10)

// Joystick (stub — pin defines only)
#define JOYSTICK_X_PIN           A0    // GP29 on QT Py
#define JOYSTICK_Y_PIN           A1    // GP28 on QT Py

// NeoKey switches (stub — pin defines only)
// Replaces the old BUTTON_PS_PIN / BUTTON_MAG_PLUS_PIN / BUTTON_MAG_MINUS_PIN
#define NEOKEY_1_PIN             27    // GP27 (A2/D2)
#define NEOKEY_2_PIN             26    // GP26 (A3/D3)
#define NEOKEY_3_PIN             25    // GP25 (SCL/D5)

// XVS — no pin available on QT Py (GP12 is onboard NeoPixel). Stub.
// Remove or comment out XVS_PIN. Keep XVS_TIMEOUT_US and RING_SETTLE_MS
// for future use.
```

Remove `BUTTON_PS_PIN`, `BUTTON_MAG_PLUS_PIN`, `BUTTON_MAG_MINUS_PIN` defines and replace with the `NEOKEY_*_PIN` defines above.

### 2.2 `sketch.yaml` — Update board FQBN

Change the profile name and FQBN:

```yaml
profiles:
  qtpy:
    fqbn: rp2040:rp2040:adafruit_qtpy
    platforms:
      - platform: rp2040:rp2040 (4.4.4)
        platform_index_url: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    libraries:
      - TMCStepper (0.7.3)
      - Adafruit NeoPixel (1.12.3)
      - Adafruit LSM6DS (4.7.4)
      - Adafruit BusIO (1.16.3)
      - Adafruit Unified Sensor (1.1.14)
```

### 2.3 `imu.cpp` — Switch from `Wire` to `Wire1`

The QT Py RP2040's STEMMA QT connector is wired to `Wire1` (GP22/GP23), not `Wire` (GP2/GP3) as on the Feather. Update `imu_init()`:

```cpp
// Change these lines:
Wire.setSDA(I2C_SDA_PIN);
Wire.setSCL(I2C_SCL_PIN);
Wire.begin();
if (!imu.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire)) {

// To:
Wire1.setSDA(I2C_SDA_PIN);
Wire1.setSCL(I2C_SCL_PIN);
Wire1.begin();
if (!imu.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire1)) {
```

The rest of `imu.cpp` is unchanged — the sensor reads go through the `Adafruit_LSM6DSOX` object which already holds its bus reference.

### 2.4 `status_led.cpp` — Add power enable for QT Py onboard NeoPixel

No code changes needed — the existing `status_led_init()` already handles `STATUS_LED_POWER_PIN >= 0` by setting it as OUTPUT and driving it HIGH. With the config.h change to `STATUS_LED_POWER_PIN = 11`, this logic activates automatically.

### 2.5 `led_stub.h` → `led.h` + `led.cpp` — **Implement LED ring driver**

Replace the stub with a real module. This module owns the `Adafruit_NeoPixel` strip object for the *external* chain (the onboard status NeoPixel remains under `status_led.cpp` on a separate `Adafruit_NeoPixel` instance).

**`led.h`:**
```cpp
#pragma once

bool led_init();
void led_tick();
```

**`led.cpp` responsibilities:**

1. **Initialization:**
   - Instantiate `Adafruit_NeoPixel strip(NUM_PIXELS_TOTAL, NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800)`.
   - Call `strip.begin()`, `strip.setBrightness(255)`, `strip.clear()`, `strip.show()`.
   - Build the polar coordinate lookup table (`led_table[]`) as specified in README §4.2. The table maps each of the `NUM_RING_PIXELS` (72) LEDs to `{angle_rad, radius_norm, chain_index}` based on the three physical rings (inner 16 @ r=0.5652, mid 24 @ r=0.7826, outer 32 @ r=1.0).

2. **`led_tick()` — called from `loop1()` every iteration:**
   - Lock `state_mutex`, read `state.mode`, copy `state.ring_colors[]` and `state.neokey_colors[]` into local buffers, unlock.
   - **Mode-based rendering** (see README §4.1 priority rules):
     - `SYS_PS_CAPTURING`: Apply `ring_colors[]` to strip directly (action server owns the ring). NeoKey PS lit solid, others reflect lens state.
     - `SYS_HOMING`: NeoKey blink pattern per `state.homing_phase` (see README §4.4). Ring blank or dim white.
     - `SYS_IDLE` / `SYS_LENS_MOVING` / `SYS_UNCALIBRATED`: Apply `ring_colors[]` from the last `/led_ring/command`. NeoKey Mag+/Mag− show opposing brightness ramps based on `lens_position_norm` (see README §4.5). When joystick is active (future, not this iteration), joystick overrides ring colors via 2D Gaussian — leave a clearly marked code path for this but don't implement the ADC read.
   - Call `strip.show()` only when the pixel buffer has changed (use a `pixels_dirty` flag).

3. **Joystick override placeholder:**
   - Declare `static bool joystick_active = false;` — always false for now.
   - In the IDLE/LENS_MOVING rendering path, include the conditional `if (joystick_active) { /* 2D Gaussian — TODO */ }` so the integration point is obvious.

4. **Do NOT put the LED strip on the same `Adafruit_NeoPixel` instance as the status LED.** They are different PIO channels on different GPIO pins (GP24 vs GP12). Two independent `Adafruit_NeoPixel` objects is correct.

### 2.6 `stepper_stub.h` → `stepper.h` + `stepper.cpp` — **Implement stepper driver**

Replace the stub with a real module.

**`stepper.h`:**
```cpp
#pragma once

bool stepper_init();
void stepper_tick();

// Called from micro-ROS callbacks (core 0) via shared state — the stepper
// module reads state.lens_velocity_cmd and state.mode on each tick.
```

**`stepper.cpp` responsibilities:**

1. **Initialization:**
   - `TMC_SERIAL.begin(TMC_UART_BAUD)` — where `TMC_SERIAL` is the `Serial1` define from config.h.
   - Instantiate `TMC2209Stepper driver(&TMC_SERIAL, TMC_RSENSE, TMC_DRIVER_ADDRESS)`.
   - Call `driver.begin()`.
   - Configure: `driver.toff(5)`, `driver.rms_current(TMC_RMS_CURRENT_MA)`, `driver.microsteps(TMC_MICROSTEPS)`, `driver.SGTHRS(STALL_THRESHOLD)`, `driver.en_spreadCycle(false)` (StealthChop for quiet operation).
   - Set `STEPPER_DIR_PIN`, `STEPPER_STEP_PIN` as OUTPUT. Set `STEPPER_ENABLE_PIN` as OUTPUT and drive LOW (enabled).

2. **`stepper_tick()` — called from `loop1()`:**
   - Lock `state_mutex`, read `state.lens_velocity_cmd`, `state.mode`, `state.lens_steps`, limits, and `state.homing_phase`. Unlock.
   - **Step pulse generation:** Use a time-based approach (not hardware timer alarm for this iteration — keep it simple). Track `last_step_us`. If `lens_velocity_cmd != 0` and enough time has elapsed for the next step (`step_interval_us = 1e6 / |velocity|`), toggle the STEP pin (HIGH then LOW with ~2 µs pulse) and update `lens_steps` (increment or decrement based on DIR).
   - **Direction:** Set `STEPPER_DIR_PIN` based on sign of `lens_velocity_cmd` before stepping.
   - **Soft limits:** Before each step, check that `lens_steps` will remain within `[lens_steps_min, lens_steps_max]`. If a limit would be exceeded, set `lens_velocity_cmd = 0`, set `state.mode = SYS_IDLE`, and hold position.
   - **Position tracking:** After each step, update `lens_position_norm`:
     ```cpp
     state.lens_position_norm = (float)(state.lens_steps - state.lens_steps_min)
                              / (float)(state.lens_steps_max - state.lens_steps_min);
     ```
   - **Velocity watchdog:** If `state.mode == SYS_LENS_MOVING` and `time_us_64() - state.last_vel_cmd_us > VEL_WATCHDOG_MS * 1000`, set `lens_velocity_cmd = 0` and transition to `SYS_IDLE`.
   - **StallGuard polling:** During `SYS_HOMING`, periodically read `driver.SG_RESULT()` (every ~10 ms, not every tick — UART reads are slow). Compare against `STALL_THRESHOLD`. If stalled, execute the homing phase transition logic per README §4.4.

3. **Homing FSM (driven from `stepper_tick()` when `state.mode == SYS_HOMING`):**
   - Phase 0 `HOMING_SEEKING_MAX`: Drive forward at `HOMING_VELOCITY`. Poll `SG_RESULT`. On stall, record `lens_steps_max`, back off `HOMING_BACKOFF_STEPS`, transition to phase 1.
   - Phase 1 `HOMING_SEEKING_MIN`: Drive reverse. On stall, record `lens_steps_min`, back off, transition to phase 2.
   - Phase 2 `HOMING_ZEROING`: Drive to midpoint `(max + min) / 2`. On arrival, set `lens_position_norm = 0.5`, transition to `SYS_IDLE`.
   - Update `state.homing_phase` at each transition (the LED module reads this for NeoKey blink patterns).

4. **Note on Serial1 echo stripping:** The TMCStepper library handles the half-duplex echo internally when using a hardware serial port. No manual echo stripping is needed.

### 2.7 `buttons_stub.h` — Update to NeoKey pin names

Keep as a stub, but update the comment to reference the new NeoKey pin defines:

```cpp
#pragma once
// NeoKey switch debounce + edge dispatch. Stub — real implementation pending.
// Will own: 10 ms debounce on NEOKEY_1_PIN / NEOKEY_2_PIN / NEOKEY_3_PIN,
// edge events for ps_trigger publish and lens_position_cmd nudges.
inline void buttons_init() {}
inline void buttons_tick() {}
```

### 2.8 `xvs_stub.h` — Update comment

Note that XVS has no pin on the QT Py:

```cpp
#pragma once
// XVS camera-sync interrupt. Stub — no GPIO available on QT Py RP2040
// (GP12 is the onboard NeoPixel). Will require board change or external
// interrupt routing to implement.
inline void xvs_init() {}
```

### 2.9 `macro-ps.ino` — Update includes

```cpp
// Replace:
#include "led_stub.h"
#include "stepper_stub.h"

// With:
#include "led.h"
#include "stepper.h"
```

Update `setup1()` to handle init failures:

```cpp
void setup1() {
  while (!mutex_is_initialized(&state_mutex)) {
    tight_loop_contents();
  }

  status_led_init();

  if (!imu_init()) {
    while (true) {
      status_led_signal_imu_fault();
      delay(50);
    }
  }

  if (!led_init()) {
    // LED init failure — non-fatal, continue without LEDs
  }

  if (!stepper_init()) {
    // Stepper init failure — could flash a fault pattern,
    // but for now continue (homing will simply fail)
  }

  joystick_init();
  buttons_init();
}
```

Make `led_init()` and `stepper_init()` return `bool` for consistency with `imu_init()`.

---

## 3. Things That Stay the Same

- **micro-ROS transport**: USB CDC via `Serial`. No change to `microros.cpp` transport callbacks.
- **State machine** (§3 of README): All mode transitions, priority rules, and `SystemState` struct unchanged.
- **IMU math** (`imu.cpp`): Coordinate transforms, lever-arm correction, IIR smoothing all unchanged. Only the I2C bus object changes (`Wire` → `Wire1`).
- **`state.h` / `state.cpp`**: No structural changes. The `SystemState` struct, mutex, and queue are unchanged.
- **`compat.cpp`**: Unchanged (newlib shim still needed).
- **`domain_id.h`**: Unchanged (auto-generated).

---

## 4. Build & Verification Notes

- The arduino-pico board variant name is `adafruit_qtpy` (lowercase, no hyphens).
- `Serial1` on the QT Py maps to UART1 with TX=GP20, RX=GP5 by default in the arduino-pico core. Do not call `Serial1.setTX()` / `Serial1.setRX()` unless the defaults are wrong — verify against the board variant pins_arduino.h if in doubt.
- The onboard NeoPixel power pin (GP11) must be driven HIGH before the onboard NeoPixel will respond. The status_led.cpp code already handles this via `STATUS_LED_POWER_PIN`.
- Two independent `Adafruit_NeoPixel` instances (status LED on GP12, external chain on GP24) each get their own PIO state machine. The RP2040 has 8 PIO state machines across 2 PIO blocks — two NeoPixel instances is well within budget.
- The TMCStepper library's `TMC2209Stepper` constructor takes a `HardwareSerial*`. Pass `&Serial1`.
