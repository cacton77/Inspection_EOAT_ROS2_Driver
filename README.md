# RP2040 Firmware & micro-ROS Integration Plan
## Camera Head Controller — Photometric Stereo System

---

## 1. Hardware Summary & Pin Assignment

### Peripherals

Board: **Adafruit Feather RP2040**, FQBN `rp2040:rp2040:adafruit_feather`.

`firmware/macro-ps/config.h` is the source of truth for every pin below; this
table is a wiring aid. If the two disagree, config.h is right.

| Peripheral | Interface | Pins | Notes |
|---|---|---|---|
| NeoPixel chain (NeoKeys + 3× rings) | PIO (Adafruit_NeoPixel) | GP7 (D5) | Moved off GP24, which the encoder now uses. GP16 is the onboard NeoPixel — cannot use. Driven at 3.3 V against a ~3.5 V WS2812 threshold: in spec only with a level shifter. **Currently cut down to 1 pixel for bring-up — see config.h chain composition** |
| TMC2209 stepper driver | `Serial1` half-duplex + DIR/STEP/EN | GP0/GP1 + GP26/GP27/GP6 | See TMC2209 wiring below |
| LSM6DSOX IMU | I2C1 via STEMMA QT (`Wire`) | GP2 (SDA), GP3 (SCL) | JST-SH cable direct to the STEMMA QT port, no additional wiring |
| Joystick X | ADC2 | GP28 (A2) | 12-bit, center ~2048. Not yet wired |
| Joystick Y | ADC3 | GP29 (A3) | 12-bit, center ~2048. Not yet wired |
| NeoKey 1 (Mag +) | GPIO | GP9 (D9) | Pull-up, active low. Not yet wired |
| NeoKey 2 (Mag −) | GPIO | GP10 (D10) | Pull-up, active low. Not yet wired |
| NeoKey 3 (PS trigger) | GPIO | GP11 (D11) | Pull-up, active low. Not yet wired |
| Quadrature encoder (ME1K) | PIO | GP24 (A), GP25 (B), GP8 (Z) | Differential; needs an AM26LV32 receiver at 3.3 V — outputs measured 1.9 V at VCC=3.3 V and 3.8 V at VCC=5 V, i.e. below V_IH and above absolute max respectively. Not yet wired |
| XVS camera sync input | GPIO interrupt | GP12 (D12) | 1.8V→3.3V level shifter required; Pi HQ Camera sync connector. Pin reserved, ISR not implemented |
| micro-ROS transport | USB-CDC (`Serial`) | USB connector | Not a UART — this is why GP0/GP1 are free for the TMC2209 |

**Stepper pin detail** (DIR/STEP on the ADC-capable pins is deliberate — they
were the pads free on the harness, and nothing here needs their ADC):

| Signal | GPIO | Feather silk |
|---|---|---|
| TMC2209 PDN_UART TX | GP0 | TX |
| TMC2209 PDN_UART RX | GP1 | RX |
| DIR | GP26 | A0 |
| STEP | GP27 | A1 |
| EN (active low) | GP6 | D4 |

**Unassigned and available:** GP7 (D5), GP8 (D6), GP13 (D13, shared with the
onboard red LED), GP18 (SCK), GP19 (MO), GP20 (MI), GP25 (D25). This is the
headroom a quadrature encoder (A/B/I) needs — the QT Py this replaced had none.

**XVS Level Shifting:**
The Pi HQ Camera (IMX477) XVS sync output is a 1.8V signal. The RP2040 GPIO input high threshold is ~2.3V and will not reliably detect it without level shifting. Use a BSS138-based 1.8V→3.3V shifter (two BSS138 MOSFETs + pull-up resistors) or a dedicated chip such as the TXS0101. The Pi HQ Camera sync connector is a 2-pin JST-SH carrying XVS and GND.

**LSM6DSOX I2C address:** `0x6A` (SA0 low) or `0x6B` (SA0 high).

**Onboard peripherals to avoid:**
- **GP16** — onboard NeoPixel (status indicator). Must not be used for the external chain.
- **GP13** — onboard red LED (D13). Usable, but it drives the LED too.

**Pins not broken out on the Feather RP2040 header:** GP14, GP15, GP17, GP21,
GP22, GP23. These must not be assigned. (An earlier revision of this table also
listed GP7, GP8 and GP21–GP25 as unavailable — that was wrong. `variants/
adafruit_feather/pins_arduino.h` in arduino-pico defines `__PIN_D4 6`,
`__PIN_D5 7`, `__PIN_D6 8` and `PIN_WIRE1_SDA/SCL 24/25`, all of which are
header pins. GP6 is **D4**, not D6.)

**`Serial2` is unusable on this board.** The variant pins it to `31u`, i.e. not
routed. Code that opens it will compile and then sit mute on nothing. The
TMC2209 UART is `Serial1` (GP0/GP1).

### NeoPixel Chain Layout

```
PIO out (GP18) →
  [NeoKey_PS]  [NeoKey_Mag+]  [NeoKey_Mag-]   (pixels  0 ..  2)
  [Inner ring  52mm: pixels  3 .. 18]    (16 LEDs)
  [Middle ring 72mm: pixels 19 .. 42]    (24 LEDs)
  [Outer ring  92mm: pixels 43 .. 74]    (32 LEDs)
```

Total: 75 pixels on a single PIO chain. One PIO state machine and one DMA channel drive the entire strand.

### TMC2209 Wiring

```
RP2040 UART1 TX (GP4) → TMC2209 PDN_UART (single-wire half-duplex via 1kΩ)
RP2040 UART1 RX (GP5) ← TMC2209 PDN_UART
GP9  → DIR
GP10 → STEP  (timer-driven pulse generation)
GP11 → EN    (active low)
```

StallGuard output is read via UART (SG_RESULT register) rather than the DIAG pin, giving a continuous stall load value rather than a binary threshold trip.

---

## 2. Software Architecture

### Build Environment

- **IDE:** Arduino IDE 2.x (or PlatformIO with arduino-pico framework)
- **Board package:** Earle Philhower's `arduino-pico` (v3.x) — select **Adafruit Feather RP2040** from the board list
- **micro-ROS:** `micro_ros_arduino` library (install via Arduino Library Manager or as a ZIP from the micro-ROS Arduino GitHub releases)
- **Transport:** UART (`Serial2` on GP4/GP5) at 1Mbaud → Pi 5 `/dev/ttyAMA0`
- **NeoPixels:** Adafruit NeoPixel library (handles PIO/DMA internally)
- **IMU:** Adafruit LSM6DS library (`Adafruit_LSM6DSOX`)
- **Stepper:** TMCStepper library

**Transport init (in `setup()`):**
```cpp
Serial2.begin(1000000);  // GP4=TX, GP5=RX via arduino-pico pin mapping
set_microros_serial_transports(Serial2);
```

**Note on `Serial2`:** The arduino-pico core maps `Serial1` to GP0/GP1 (UART0) and `Serial2` to GP4/GP5 (UART1) by default on the Feather RP2040. `Serial1` is reserved for the micro-ROS UART0 transport to the Pi 5. Confirm the mapping matches your board package version before building.

### Task Structure (Arduino Multicore)

The arduino-pico core provides native dual-core support without requiring FreeRTOS. Core 0 runs `setup()`/`loop()` and core 1 runs `setup1()`/`loop1()`.

```
┌─────────────────────────────────────────────────────┐
│ Core 0 — setup() / loop()                            │
│   - micro-ROS executor spin                          │
│   - all ROS2 entity callbacks                        │
│   - IMU FIFO drain → /camera_head/imu publish        │
│   - XVS edge ISR (sets flag + timestamp)             │
│   - watchdog feed                                    │
├─────────────────────────────────────────────────────┤
│ Core 1 — setup1() / loop1()                          │
│   - ADC sampling (joystick)                          │
│   - GPIO debounce (buttons)                          │
│   - stepper step pulse generation                    │
│   - NeoPixel write (Adafruit NeoPixel library)       │
│   - LSM6DSOX I2C read + transform + FIFO push        │
└─────────────────────────────────────────────────────┘
```

Keeping I2C and peripheral work on core 1 ensures the micro-ROS executor on core 0 is never stalled by I2C bus transactions.

**Inter-core communication** uses the RP2040's hardware FIFO for the IMU sample handoff, and a pico-SDK `mutex_t` protecting the shared `SystemState` struct:

```cpp
#include "pico/mutex.h"
#include "pico/util/queue.h"

// Shared state — access from either core must hold state_mutex
typedef struct {
    // LED
    uint32_t ring_colors[NUM_RING_PIXELS];
    uint32_t neokey_colors[3];

    // Lens
    float    lens_position_norm;
    float    lens_velocity_cmd;       // steps/sec, 0 = hold
    int32_t  lens_steps;
    int32_t  lens_steps_min;
    int32_t  lens_steps_max;

    // System mode
    SystemMode  mode;                 // see §3
    HomingPhase homing_phase;        // see §4

    // Timestamps (microseconds, from time_us_64())
    uint64_t last_vel_cmd_us;
    uint64_t last_ring_cmd_us;
} SystemState;

mutex_t    state_mutex;
SystemState state;

// IMU samples passed core1→core0 via a single-element queue (non-blocking)
typedef struct {
    float    omega_C[3];        // angular velocity in camera frame [rad/s]
    float    a_cam_origin[3];   // linear acceleration at camera origin [m/s²]
    uint64_t timestamp_us;
} ImuSample;

queue_t imu_queue;   // depth 1, element size sizeof(ImuSample)
                     // queue_init(&imu_queue, sizeof(ImuSample), 1);
                     // core1: queue_try_add (drops if full)
                     // core0: queue_try_remove (non-blocking)
```

`mutex_t` and `queue_t` are from the Pico SDK, which is always available underneath the arduino-pico core. The hardware FIFO (`rp2040.fifo`) is an alternative for simple scalar values but the `queue_t` API is cleaner for the `ImuSample` struct.

---

## 3. System Mode State Machine

All input arbitration flows through a single mode variable. Higher modes lock out lower-priority inputs.

```
                    ┌─────────────────┐
              ┌────►│   UNCALIBRATED  │◄────┐
              │     └────────┬────────┘     │
        reset/fault          │ /lens/home   │ fault
              │              ▼ goal recv    │
              │     ┌─────────────────┐     │
              │     │    HOMING       │─────┘
              │     └────────┬────────┘
              │              │ homing complete
              │              ▼
              │     ┌─────────────────┐
              │◄────│     IDLE        │◄────────────────┐
              │     └────────┬───┬────┘                 │
              │              │   │                      │
              │   ps_trigger │   │ /lens/command or     │
              │   button or  │   │ mag +/- buttons      │
              │   /ps_capture│   ▼                      │
              │   action     │  ┌──────────────────┐    │
              │              │  │  LENS_MOVING     │────┘
              │              │  └──────────────────┘  complete/
              │              │                         cancel
              │              ▼
              │     ┌─────────────────┐
              │     │  PS_CAPTURING   │────────────────►(back to IDLE)
              └─────┴─────────────────┘
```

**Mode priority rules:**
- `PS_CAPTURING` locks out joystick, mag buttons, and `/lens/command`
- `HOMING` locks out all inputs except `/lens/home` cancellation
- `LENS_MOVING` allows joystick LED control but locks mag buttons to prevent re-entry
- `UNCALIBRATED` allows LED joystick control and manual `/led_ring/command` but disables all lens interfaces
- IMU reading and publishing runs in **all modes** — it is never gated by system state

---

## 4. Subsystem Specifications

### 4.1 NeoPixel Chain

**Pixel index map:**
```c
// NeoKeys are first in chain
#define IDX_NEOKEY_PS        0
#define IDX_NEOKEY_MAG_PLUS  1
#define IDX_NEOKEY_MAG_MINUS 2

// Physical rings follow
#define NUM_RING_INNER    16         // 52mm diameter
#define NUM_RING_MID      24         // 72mm diameter
#define NUM_RING_OUTER    32         // 92mm diameter
#define NUM_RING_PIXELS   72         // total illumination pixels

// Chain offsets for rings
#define OFFSET_INNER       3
#define OFFSET_MID        19
#define OFFSET_OUTER      43

#define NUM_PIXELS_TOTAL  75

// Physical radii normalized to outer ring [unitless, 0..1]
// 52/92 = 0.5652,  72/92 = 0.7826,  92/92 = 1.0000
#define RADIUS_INNER  0.5652f
#define RADIUS_MID    0.7826f
#define RADIUS_OUTER  1.0000f
```

**Write path:** The Adafruit NeoPixel library handles PIO and DMA internally. Declare the strip globally and call `strip.show()` from `loop1()` whenever the pixel buffer is dirty:

```cpp
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip(NUM_PIXELS_TOTAL, 18, NEO_GRB + NEO_KHZ800);
// GP18 = pin 18 in arduino-pico's default pin numbering

// In setup1():
strip.begin();
strip.setBrightness(255);

// In loop1(), after computing pixel values:
if (pixels_dirty) {
    strip.show();
    pixels_dirty = false;
}
```

Rather than maintaining a raw `uint32_t pixel_buf[]`, use `strip.setPixelColor(index, r, g, b)` to stage values and `strip.show()` to commit. The library's internal buffer is the single source of truth for LED state.

**Priority:**
```cpp
// In loop1(), before strip.show():
mutex_enter_blocking(&state_mutex);
SystemMode current_mode = state.mode;
mutex_exit(&state_mutex);

if (current_mode == PS_CAPTURING) {
    // action server owns ring pixels
    // NeoKeys: PS key lit solid, others follow lens state
} else if (current_mode == HOMING) {
    // NeoKeys: blink pattern per homing_phase (see §4.4)
    // Ring: blank or dim white
} else {
    // Ring: joystick override if active, else last /led_ring/command
    // NeoKeys: lens position visualization (see §4.5)
}
```

### 4.2 Joystick → Illumination Direction

#### Polar Coordinate Table

Each LED is assigned a physical angle and a radius (normalized to the outer ring) at init time. Using real physical radii rather than equal fractions ensures that joystick magnitude maps to a physically meaningful illumination radius.

```c
typedef struct {
    float    angle_rad;     // [0, 2π), evenly spaced within each ring
    float    radius_norm;   // physical radius / 92mm
    uint16_t chain_index;   // index into pixel_buf[]
} LedPolar;

static LedPolar led_table[NUM_RING_PIXELS];

void build_led_table(void) {
    // Physical assumption: pixel 0 of each ring is aligned at 0 radians (3 o'clock
    // in standard math convention, or whichever direction the rig defines as 0°).
    // All three rings share this alignment. If a ring is ever remounted at a different
    // physical rotation, add a per-ring angle_offset_rad term here rather than
    // touching the Gaussian mapping code.
    //
    // {LED count, physical radius normalized to outer ring (92mm)}
    //   inner: 52/92 = 0.5652
    //   mid:   72/92 = 0.7826
    //   outer: 92/92 = 1.0000
    const struct { int n; float r; } rings[3] = {
        { NUM_RING_INNER, RADIUS_INNER },   // 16 LEDs
        { NUM_RING_MID,   RADIUS_MID   },   // 24 LEDs
        { NUM_RING_OUTER, RADIUS_OUTER }    // 32 LEDs
    };
    int offsets[3] = { OFFSET_INNER, OFFSET_MID, OFFSET_OUTER };
    for (int ring = 0; ring < 3; ring++) {
        for (int i = 0; i < rings[ring].n; i++) {
            int idx = offsets[ring] + i;
            led_table[offsets[ring] - OFFSET_INNER + i] = (LedPolar){
                .angle_rad   = (2.0f * M_PI * i) / rings[ring].n,
                .radius_norm = rings[ring].r,
                .chain_index = idx
            };
        }
    }
}
```

`build_led_table()` is called once during initialization; the table is read-only thereafter.

#### ADC Sampling and 2D Gaussian Mapping

**ADC sampling:** 1kHz in `loop1()`. Joystick deflection maps directly to polar illumination coordinates: angle controls azimuth, magnitude controls which ring(s) are lit via a 2D Gaussian in (Δθ, Δr) space.

```cpp
int adc_x = analogRead(A0);   // GP26
int adc_y = analogRead(A1);   // GP27

float dx  = (adc_x - 2048.0f) / 2048.0f;  // [-1, 1]
float dy  = (adc_y - 2048.0f) / 2048.0f;
float mag = sqrtf(dx*dx + dy*dy);          // [0, 1], maps to radius_norm

if (mag > JOYSTICK_DEADZONE && current_mode != PS_CAPTURING
                             && current_mode != HOMING) {
    float angle = atan2f(dy, dx);           // [-π, π]
    set_ring_2d_gaussian(angle, mag);
    joystick_active = true;
} else {
    joystick_active = false;
    // Ring reverts to last /led_ring/command state
}
```

```cpp
void set_ring_2d_gaussian(float joystick_angle, float joystick_magnitude) {
    for (int i = 0; i < NUM_RING_PIXELS; i++) {
        // Angular distance, wrapped to [-π, π]
        float dtheta = joystick_angle - led_table[i].angle_rad;
        dtheta = atan2f(sinf(dtheta), cosf(dtheta));

        // Radial distance in normalized radius space
        float dr = joystick_magnitude - led_table[i].radius_norm;

        float weight = expf(-(dtheta * dtheta) / (2.0f * SIGMA_A * SIGMA_A)
                            -(dr    * dr)       / (2.0f * SIGMA_R * SIGMA_R));

        uint8_t v = (uint8_t)(weight * 255.0f);
        strip.setPixelColor(led_table[i].chain_index, v, v, v);
    }
    pixels_dirty = true;
}
```

`SIGMA_A` controls angular spread (suggested starting value: `M_PI / 6`, i.e. ±30° at half-power). `SIGMA_R` controls radial spread across rings (suggested starting value: `0.15`, tight enough to primarily illuminate one ring at a time given the normalized radii are spaced ~0.22 apart).

At full deflection (`mag = 1.0`) the highlight sits on the outer ring. At `mag ≈ 0.78` it centres on the middle ring. At `mag ≈ 0.57` it centres on the inner ring. Intermediate magnitudes smoothly blend across adjacent rings.

The joystick does **not** publish to `/led_ring/state` — its control is purely local. The state topic always reflects what is physically on the ring.

### 4.3 Stepper / TMC2209

**UART configuration:** Use the TMCStepper library (`TMC2209Stepper`). Initialize on `Serial2` (GP4/GP5) at 500kbps:

```cpp
#include <TMCStepper.h>

TMC2209Stepper driver(&Serial2, 0.11f, 0b00);  // R_sense, driver address

// In setup1():
Serial2.begin(500000);
driver.begin();
driver.toff(5);
driver.rms_current(600);        // mA, tune to motor
driver.microsteps(16);
driver.SGTHRS(STALL_THRESHOLD); // StallGuard threshold
driver.en_spreadCycle(false);   // StealthChop for quiet operation
```

Read StallGuard load value during homing:
```cpp
uint16_t sg = driver.SG_RESULT();  // 0 = stalled, higher = less load
```

**Step generation:** Hardware timer (alarm) in `peripheral_task` generates STEP pulses at the commanded rate. Step interval:
```c
uint32_t step_interval_us = (uint32_t)(1e6f / fabsf(state.lens_velocity_cmd));
```
DIR pin set before enabling the timer.

**Position tracking:**
```c
// In step ISR:
state.lens_steps += (dir == FORWARD) ? 1 : -1;
state.lens_position_norm = (float)(state.lens_steps - state.lens_steps_min)
                         / (float)(state.lens_steps_max - state.lens_steps_min);
```

**Velocity watchdog:** In `watchdog_task`, if `state.mode == LENS_MOVING` and time since `last_vel_cmd_us` exceeds 150ms, set `lens_velocity_cmd = 0` and transition mode to `IDLE`.

**Soft limits:** Before executing any step, check that `lens_steps` will remain within `[lens_steps_min, lens_steps_max]`. Stop and hold if a limit would be exceeded.

### 4.4 Homing Sequence (StallGuard)

Homing runs entirely in `micro_ros_task` as part of the `/lens/home` action server callback, with periodic yields to keep the executor alive. Sequence:

```
Phase 0 — SEEKING_MAX:
    Drive forward at homing_velocity until SG_RESULT drops below STALL_THRESHOLD
    Record lens_steps_max = current steps
    Back off N steps away from hard stop

Phase 1 — SEEKING_MIN:
    Drive reverse at homing_velocity until SG_RESULT drops below STALL_THRESHOLD
    Record lens_steps_min = current steps
    Back off N steps away from hard stop

Phase 2 — ZEROING:
    Drive forward to midpoint ( (max+min)/2 )
    Set lens_steps offset so midpoint = 0.5 normalized

Publish feedback after each phase transition.
On completion: set mode = IDLE, publish result with calibrated range.
```

**NeoKey LED behavior during homing:**
```c
// Blink period 500ms, driven by peripheral_task
switch (state.homing_phase) {
    case SEEKING_MAX:
        neokey[MAG_PLUS]  = blink_on ? COLOR_AMBER : COLOR_OFF;
        neokey[MAG_MINUS] = COLOR_OFF;
        break;
    case SEEKING_MIN:
        neokey[MAG_PLUS]  = COLOR_OFF;
        neokey[MAG_MINUS] = blink_on ? COLOR_AMBER : COLOR_OFF;
        break;
    case ZEROING:
        neokey[MAG_PLUS]  = COLOR_AMBER;
        neokey[MAG_MINUS] = COLOR_AMBER;
        break;
}
```

### 4.5 NeoKey LED — Normal Operation

After calibration, NeoKey LEDs reflect current lens position as opposing brightness ramps:

```cpp
// In loop1(), mode == IDLE or LENS_MOVING:
float p = state.lens_position_norm;  // [0.0, 1.0]
uint8_t v_plus  = (uint8_t)(p * 255.0f);
uint8_t v_minus = (uint8_t)((1.0f - p) * 255.0f);
strip.setPixelColor(IDX_NEOKEY_MAG_PLUS,  0, v_plus,  0);
strip.setPixelColor(IDX_NEOKEY_MAG_MINUS, 0, v_minus, 0);
pixels_dirty = true;
```

At p=1.0: MAG+ fully lit, MAG− off. At p=0.5: both at half brightness. At p=0.0: MAG+ off, MAG− fully lit.

### 4.6 Button Handling

All three buttons use 10ms debounce in `loop1()`. On confirmed press edge:

```cpp
if (btn_ps_pressed && current_mode == IDLE)
    ps_trigger_pending = true;  // picked up by loop()

if (btn_mag_plus_pressed && current_mode == IDLE)
    lens_position_cmd_pending = min(state.lens_position_norm + MAG_STEP, 1.0f);

if (btn_mag_minus_pressed && current_mode == IDLE)
    lens_position_cmd_pending = max(state.lens_position_norm - MAG_STEP, 0.0f);
```

`MAG_STEP` default: 0.05 (5% of calibrated range per press). Hold-to-repeat can be added later with a press-duration counter. In `micro_ros_task`, `ps_trigger_pending` causes a publish on `/camera_head/ps_trigger`.

### 4.7 LSM6DSOX IMU

#### Initialization

Use the Adafruit LSM6DS library (`Adafruit_LSM6DSOX`). The STEMMA QT connector uses the default `Wire` bus (GP2/GP3):

```cpp
#include <Adafruit_LSM6DSOX.h>

Adafruit_LSM6DSOX imu;

// In setup1():
Wire.begin();  // GP2/GP3 — arduino-pico default for Feather RP2040
if (!imu.begin_I2C()) {
    while (1) { /* halt — IMU not found */ }
}

// Accelerometer: ±4g, 416 Hz ODR
imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
imu.setAccelDataRate(LSM6DS_RATE_416_HZ);

// Gyroscope: ±500 dps, 416 Hz ODR
imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
imu.setGyroDataRate(LSM6DS_RATE_416_HZ);
```

Reduce accelerometer range to `LSM6DS_ACCEL_RANGE_2_G` if the camera head environment is vibration-quiet.

#### Reading & Rate Control

IMU reading is integrated into `loop1()` via a tick divider against a 1kHz loop rate:

```cpp
static uint32_t imu_divider = 0;
if (++imu_divider >= IMU_DIVIDER) {     // 1kHz / 5 = 200 Hz
    imu_divider = 0;

    sensors_event_t accel_evt, gyro_evt, temp_evt;
    imu.getEvent(&accel_evt, &gyro_evt, &temp_evt);

    float accel_S[3] = {
        accel_evt.acceleration.x,   // m/s²
        accel_evt.acceleration.y,
        accel_evt.acceleration.z
    };
    float omega_S[3] = {
        gyro_evt.gyro.x,            // rad/s
        gyro_evt.gyro.y,
        gyro_evt.gyro.z
    };

    ImuSample sample = compute_imu_camera_frame(accel_S, omega_S);
    queue_try_add(&imu_queue, &sample);  // drops silently if full
}
```

#### Coordinate Transform Mathematics

Let the rigid body transform from **sensor frame S** to **camera frame C** be defined by:
- `R_CS` ∈ SO(3): rotation mapping vectors from S into C
- `t_CS` ∈ ℝ³: position of the sensor origin expressed in camera frame C

Both are compile-time constants derived from CAD or physical calibration (see §10).

**Angular velocity:**
```
ω_C = R_CS · ω_S
```

**Angular acceleration (finite difference + IIR smoothing):**
```c
float3 alpha_raw = float3_scale(float3_sub(omega_C, omega_C_prev), inv_dt);
float3 alpha_C   = float3_lerp(alpha_raw, alpha_C_prev, 1.0f - ALPHA_BETA);
// ALPHA_BETA ≈ 0.1–0.3; tune to balance noise vs. lag
```

**Linear acceleration at camera origin (lever arm correction):**

The accelerometer measures specific force at the IMU location. The acceleration at the camera origin on the same rigid body is:

```
a_cam = R_CS · a_S  +  α_C × t_CS  +  ω_C × (ω_C × t_CS)
         ──────────    ───────────    ──────────────────────
         accel in      tangential     centripetal
         cam frame     correction     correction
```

```c
float3 omega_C      = mat3_mul_vec(R_CS, omega_S);
float3 accel_C      = mat3_mul_vec(R_CS, accel_S);
float3 tangential   = cross3(alpha_C, t_CS);
float3 centripetal  = cross3(omega_C, cross3(omega_C, t_CS));
float3 a_cam_origin = float3_add(accel_C, float3_add(tangential, centripetal));

omega_C_prev = omega_C;
alpha_C_prev = alpha_C;
```

`mat3_mul_vec`, `cross3`, and `float3_*` are lightweight inline helpers — no external math library required. Total cost is 1 matrix-vector multiply (9 mults, 6 adds) plus 2 cross products (12 mults, 6 adds), well within the RP2040 hardware FPU at 200 Hz.

The correction terms are zero when `t_CS = 0` (IMU co-located with camera origin), providing a useful sanity check during integration.

**Gravity:** `a_cam_origin` includes gravity (specific force). For vibration characterization or motion blur estimation this is acceptable as-is. If removal is needed, average `a_cam_origin` over `N_GRAVITY_SAMPLES` at startup while stationary and subtract.

#### Transform Constants

```c
// R_CS: row-major 3×3, from CAD or physical calibration
// If the IMU is mounted at a simple 90° rotation, R_CS will be a
// permutation matrix — easy to verify by inspection.
static const float R_CS[3][3] = {
    { r00, r01, r02 },
    { r10, r11, r12 },
    { r20, r21, r22 }
};

// t_CS: sensor origin to camera origin, expressed in camera frame [m]
static const float t_CS[3] = { tx, ty, tz };
```

---

## 5. micro-ROS Interface Definitions

### 5.1 Custom Messages

**`ps_interfaces/msg/LedRingCommand.msg`**
```
std_msgs/ColorRGBA[] colors    # length must equal NUM_RING_PIXELS
```

**`ps_interfaces/msg/LedRingState.msg`**
```
std_msgs/ColorRGBA[] colors
builtin_interfaces/Time stamp
```

**`ps_interfaces/msg/LensCommand.msg`**
```
uint8 mode               # 0=POSITION, 1=VELOCITY
float32 value            # normalized [0,1] position OR steps/sec velocity
```

**`ps_interfaces/msg/LensState.msg`**
```
float32 position_norm
float32 velocity         # steps/sec (positive = toward max)
uint8   status           # 0=UNCALIBRATED, 1=IDLE, 2=MOVING, 3=STALLED, 4=HOMING
builtin_interfaces/Time stamp
```

The IMU topic uses the standard `sensor_msgs/msg/Imu` — no custom message required.

### 5.2 Custom Action Definitions

**`ps_interfaces/action/CaptureSequence.action`**
```
# Goal
ps_interfaces/msg/LedRingCommand[] patterns   # ordered illumination patterns
# (no dwell_ms — frame timing is governed by XVS hardware sync)
---
# Result
bool success
uint32 frames_captured
---
# Feedback
uint32 current_pattern_index
builtin_interfaces/Time pattern_start_time    # timestamp of XVS rising edge
```

**`ps_interfaces/action/HomeLens.action`**
```
# Goal  (empty)
---
# Result
bool    success
float32 range_steps
string  message
---
# Feedback
uint8  phase                 # 0=SEEKING_MAX, 1=SEEKING_MIN, 2=ZEROING
string phase_description
```

### 5.3 ROS2 Entity Registration

```c
// Subscriptions
RCCHECK(rclc_subscription_init_best_effort(
    &sub_led_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LedRingCommand),
    "/led_ring/command"));

RCCHECK(rclc_subscription_init_best_effort(
    &sub_lens_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LensCommand),
    "/lens/command"));

// Publishers
RCCHECK(rclc_publisher_init_best_effort(
    &pub_led_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LedRingState),
    "/led_ring/state"));

RCCHECK(rclc_publisher_init_best_effort(
    &pub_lens_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LensState),
    "/lens/state"));

RCCHECK(rclc_publisher_init_best_effort(
    &pub_ps_trigger, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/camera_head/ps_trigger"));

RCCHECK(rclc_publisher_init_best_effort(
    &pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/camera_head/imu"));

// Action servers
RCCHECK(rclc_action_server_init_default(
    &as_ps_capture, &node, &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(ps_interfaces, CaptureSequence),
    "/ps_capture"));

RCCHECK(rclc_action_server_init_default(
    &as_home_lens, &node, &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(ps_interfaces, HomeLens),
    "/lens/home"));
```

### 5.4 Executor Configuration

```c
// 2 subscriptions + 2 action servers = 2 + (3×2) = 8 handles
// IMU publisher uses direct rcl_publish outside the executor — no handle needed
rclc_executor_t executor;
RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));

rclc_executor_add_subscription(&executor, &sub_led_cmd,
    &led_cmd_msg, &cb_led_cmd, ON_NEW_DATA);

rclc_executor_add_subscription(&executor, &sub_lens_cmd,
    &lens_cmd_msg, &cb_lens_cmd, ON_NEW_DATA);

rclc_executor_add_action_server(&executor, &as_ps_capture,
    1, &ps_goal_msg, sizeof(ps_goal_msg),
    &cb_ps_goal_request, &cb_ps_cancel_request, &cb_ps_goal_accepted);

rclc_executor_add_action_server(&executor, &as_home_lens,
    1, &home_goal_msg, sizeof(home_goal_msg),
    &cb_home_goal_request, &cb_home_cancel_request, &cb_home_goal_accepted);
```

### 5.5 Callback Implementations

**`cb_led_cmd`** — LED ring command subscription:
```c
void cb_led_cmd(const void* msg_in) {
    if (state.mode == PS_CAPTURING) return;
    const LedRingCommand* msg = (const LedRingCommand*)msg_in;
    mutex_enter_blocking(&state_mutex);
    for (int i = 0; i < NUM_RING_PIXELS; i++)
        state.ring_colors[i] = rgba_to_grb(msg->colors.data[i]);
    state.last_ring_cmd_us = time_us_64();
    mutex_exit(&state_mutex);
}
```

**`cb_lens_cmd`** — Lens command subscription:
```c
void cb_lens_cmd(const void* msg_in) {
    if (state.mode == PS_CAPTURING || state.mode == HOMING
     || state.mode == UNCALIBRATED) return;
    const LensCommand* msg = (const LensCommand*)msg_in;
    mutex_enter_blocking(&state_mutex);
    if (msg->mode == LENS_CMD_POSITION) {
        float target = fclampf(msg->value, 0.0f, 1.0f);
        state.lens_velocity_cmd = (target > state.lens_position_norm)
                                 ? LENS_DEFAULT_VELOCITY
                                 : -LENS_DEFAULT_VELOCITY;
        state.lens_position_target = target;
        state.mode = LENS_MOVING;
    } else {
        state.lens_velocity_cmd = msg->value;
        state.last_vel_cmd_us = time_us_64();
        if (fabsf(msg->value) > 0.0f) state.mode = LENS_MOVING;
    }
    mutex_exit(&state_mutex);
}
```

**XVS interrupt setup (in `setup()`):**
```cpp
#define XVS_PIN 12  // GP12, signal arrives via 1.8V→3.3V level shifter

volatile bool     xvs_edge = false;
volatile uint64_t xvs_timestamp_us = 0;

void xvs_isr() {
    xvs_timestamp_us = time_us_64();
    xvs_edge = true;
}

// In setup():
pinMode(XVS_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(XVS_PIN), xvs_isr, RISING);
```

**`cb_ps_goal_accepted`** — PS capture action execution:
```cpp
void cb_ps_goal_accepted(rclc_action_goal_handle_t* goal_handle, void* ctx) {
    CaptureSequence_Goal* goal = goal_handle->ros_goal_request;
    save_ring_state();
    mutex_enter_blocking(&state_mutex);
    state.mode = PS_CAPTURING;
    mutex_exit(&state_mutex);

    CaptureSequence_Feedback feedback;
    for (uint32_t i = 0; i < goal->patterns.size; i++) {
        if (rclc_action_goal_is_cancel_requested(goal_handle)) {
            restore_ring_state();
            mutex_enter_blocking(&state_mutex);
            state.mode = IDLE;
            mutex_exit(&state_mutex);
            CaptureSequence_Result result = {.success = false, .frames_captured = i};
            rclc_action_send_result(goal_handle, GOAL_STATE_CANCELED, &result);
            return;
        }

        // 1. Apply illumination pattern
        apply_ring_pattern(&goal->patterns.data[i]);

        // 2. Wait for LEDs to settle
        delay(RING_SETTLE_MS);

        // 3. Wait for XVS rising edge (next frame start), with timeout
        xvs_edge = false;
        uint64_t timeout_us = time_us_64() + XVS_TIMEOUT_US;
        while (!xvs_edge && time_us_64() < timeout_us) { delayMicroseconds(100); }

        if (!xvs_edge) {
            // Camera not running or XVS wiring fault — abort
            restore_ring_state();
            mutex_enter_blocking(&state_mutex);
            state.mode = IDLE;
            mutex_exit(&state_mutex);
            CaptureSequence_Result result = {.success = false, .frames_captured = i};
            rclc_action_send_result(goal_handle, GOAL_STATE_ABORTED, &result);
            return;
        }

        // 4. Publish feedback — Pi matches this timestamp to the
        //    libcamera frame timestamp to identify which frame carries
        //    this illumination pattern
        feedback.current_pattern_index = i;
        feedback.pattern_start_time = us_to_ros_time(xvs_timestamp_us);
        rclc_action_publish_feedback(goal_handle, &feedback);

        // 5. Wait for frame to complete (next XVS edge) before advancing
        //    pattern — ensures LEDs don't change mid-exposure
        xvs_edge = false;
        timeout_us = time_us_64() + XVS_TIMEOUT_US;
        while (!xvs_edge && time_us_64() < timeout_us) { delayMicroseconds(100); }
    }

    restore_ring_state();
    mutex_enter_blocking(&state_mutex);
    state.mode = IDLE;
    mutex_exit(&state_mutex);
    CaptureSequence_Result result = {.success = true,
                                     .frames_captured = goal->patterns.size};
    rclc_action_send_result(goal_handle, GOAL_STATE_SUCCEEDED, &result);
}
```

---

## 6. State Publishing

State topics and the IMU topic are published at fixed rates from `micro_ros_task`, independent of callbacks.

| Topic | Rate | Trigger |
|---|---|---|
| `/led_ring/state` | 10 Hz | Timer, always |
| `/lens/state` | 20 Hz | Timer, always |
| `/camera_head/imu` | 200 Hz | IMU queue drain, always |
| `/camera_head/ps_trigger` | On event | Button press edge |

```cpp
// In loop() on core 0, after executor spin:
uint64_t now = time_us_64();

if (now - last_led_pub_us > 100000) {       // 10 Hz
    publish_led_state();
    last_led_pub_us = now;
}
if (now - last_lens_pub_us > 50000) {       // 20 Hz
    publish_lens_state();
    last_lens_pub_us = now;
}

// IMU: drain on every loop iteration (non-blocking)
ImuSample imu_sample;
if (queue_try_remove(&imu_queue, &imu_sample)) {
    imu_msg.header.stamp           = us_to_ros_time(imu_sample.timestamp_us);
    imu_msg.header.frame_id        = micro_ros_string("camera_optical_frame");
    imu_msg.angular_velocity.x    = imu_sample.omega_C[0];
    imu_msg.angular_velocity.y    = imu_sample.omega_C[1];
    imu_msg.angular_velocity.z    = imu_sample.omega_C[2];
    imu_msg.linear_acceleration.x = imu_sample.a_cam_origin[0];
    imu_msg.linear_acceleration.y = imu_sample.a_cam_origin[1];
    imu_msg.linear_acceleration.z = imu_sample.a_cam_origin[2];
    imu_msg.orientation_covariance[0] = -1.0f;
    rcl_publish(&pub_imu, &imu_msg, NULL);
}

if (ps_trigger_pending) {
    rcl_publish(&pub_ps_trigger, &empty_msg, NULL);
    ps_trigger_pending = false;
}
```

Setting `orientation_covariance[0] = -1` is the ROS2 convention for "orientation not provided." Downstream nodes such as `robot_localization` respect this sentinel and will not attempt to use orientation data.

The IMU queue uses `queue_try_add` on the write side (core 1) so that `loop1()` never blocks. During agent disconnection, samples are simply dropped until reconnection — no data hazard.

---

## 7. micro-ROS Agent Connection & Reconnection

The RP2040 must handle agent disconnection gracefully (e.g. Pi 5 reboot, UART glitch). This runs in `loop()` on core 0:

```cpp
// In loop():
switch (agent_state) {
    case WAITING_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
            agent_state = AGENT_AVAILABLE;
        break;

    case AGENT_AVAILABLE:
        create_entities();
        agent_state = AGENT_CONNECTED;
        break;

    case AGENT_CONNECTED:
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
        } else {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            // publish states, drain IMU queue, handle ps_trigger
        }
        break;

    case AGENT_DISCONNECTED:
        destroy_entities();
        mutex_enter_blocking(&state_mutex);
        state.lens_velocity_cmd = 0;
        if (state.mode == PS_CAPTURING) state.mode = IDLE;
        mutex_exit(&state_mutex);
        agent_state = WAITING_AGENT;
        break;
}
delay(10);
```

---

## 8. Memory Budget (RP2040)

The RP2040 has 264KB SRAM. The arduino-pico core itself uses less overhead than a FreeRTOS build, which improves headroom further.

| Item | Size (approx) |
|---|---|
| micro-ROS static pool | 64KB (tune down if needed) |
| arduino-pico core + dual-core stacks | ~8KB |
| Adafruit NeoPixel internal buffer (75 pixels × 3 bytes) | ~256 bytes |
| Message buffers (all ROS entities) | ~4KB |
| `sensor_msgs/Imu` message buffer | ~512 bytes |
| Pico SDK `queue_t` IMU queue (depth 1) | ~128 bytes |
| ImuSample struct + transform state | ~256 bytes |
| Adafruit LSM6DSOX driver state | ~512 bytes |
| SystemState struct | <1KB |
| **Total** | **~84KB** |

Leaves ~180KB headroom. Profile with `xPortGetFreeHeapSize()` after bringing all subsystems up; tune the micro-ROS static pool size first if memory becomes tight.

---

## 9. Parameters to Tune at Integration Time

| Parameter | Suggested Default | Notes |
|---|---|---|
| `NUM_RING_INNER/MID/OUTER` | 16 / 24 / 32 | Fixed — matches physical rings |
| `RADIUS_INNER/MID/OUTER` | 0.5652 / 0.7826 / 1.0 | Fixed — 52/72/92mm ÷ 92mm |
| `RING_SETTLE_MS` | 5 | Time after pixel write before FSTROBE |
| `SIGMA_A` | `M_PI / 6` | Angular spread of joystick highlight (~±30° half-power) |
| `SIGMA_R` | 0.15 | Radial spread across rings (rings spaced ~0.22 apart) |
| `MAG_STEP` | 0.05 | Normalized position increment per button press |
| `HOMING_VELOCITY` | 200 steps/s | Slow enough for reliable StallGuard |
| `LENS_DEFAULT_VELOCITY` | 500 steps/s | For position-mode commands |
| `STALL_THRESHOLD` | ~50 (SG_RESULT) | Tune per motor load |
| `HOMING_BACKOFF_STEPS` | 20 | Steps to retreat after stall detection |
| `VEL_WATCHDOG_MS` | 150 | Velocity command timeout |
| `JOYSTICK_DEADZONE` | 0.05 | Fraction of full ADC range |
| `FSTROBE_PULSE_US` → removed | — | Replaced by XVS hardware sync |
| `XVS_TIMEOUT_US` | 200000 (200ms) | Max wait for XVS edge before aborting; set > 1 frame period at your chosen frame rate |
| `IMU_DIVIDER` | 5 | peripheral_task ticks per IMU read (1kHz / 5 = 200 Hz) |
| `ALPHA_BETA` | 0.2 | IIR smoothing coefficient for angular acceleration |
| `N_GRAVITY_SAMPLES` | 200 | Samples to average for static gravity estimation at startup |

---

## 10. IMU Calibration & Transform Notes

### Determining R_CS and t_CS

`R_CS` and `t_CS` encode the physical relationship between the LSM6DSOX chip frame and the camera optical frame and must be populated before building firmware.

**From CAD:** Export the transformation between the LSM6DSOX chip frame (defined in the datasheet — not necessarily aligned with the PCB silkscreen) and the camera optical frame from your assembly model.

**From physical verification:** Mount the camera head on a precision rotary stage. Rotate by a known angle about a known axis and verify that `ω_C` (after applying `R_CS`) has its energy on the expected axis with the correct sign. If the IMU is mounted at a simple 90° rotation, `R_CS` will be a permutation matrix, straightforward to confirm by inspection.

**Sanity check for t_CS:** Command a pure rotation and observe the centripetal correction term. With `t_CS` correctly set, `ω_C × (ω_C × t_CS)` should point radially inward from the IMU toward the camera origin.

### Gravity Removal (Optional)

If needed, estimate gravity at startup over `N_GRAVITY_SAMPLES` samples while the head is stationary and subtract from all subsequent `a_cam_origin` values. Do not attempt gravity removal during HOMING or if the head is known to be in motion at startup.

---

## 11. Host-Side Integration Notes

### Pi 5 UART Setup

The micro-ROS agent runs on the Pi 5 and communicates with the RP2040 over hardware UART on the 40-pin GPIO header. No level shifting is required — both devices are 3.3V logic.

**Wiring:**

| RP2040 | Pi 5 40-pin header |
|---|---|
| GP0 (UART0 TX) | Pin 10 (GPIO15 / RX) |
| GP1 (UART0 RX) | Pin 8  (GPIO14 / TX) |
| GND | Pin 6  (GND) |

**Pi 5 configuration** — edit `/boot/firmware/config.txt`:
```
enable_uart=1
```

Disable the Linux serial console on this port by removing `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Without this step the OS will print boot messages and accept shell input on the same UART the agent is trying to use.

The UART is exposed as `/dev/ttyAMA0`. Confirm with:
```bash
ls -la /dev/ttyAMA0
```

### Launching the micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 1000000
```

1Mbaud gives ample headroom for the combined topic load — the 200Hz IMU stream is the highest-bandwidth publisher and fits comfortably within the available throughput. For a production deployment, wrap the agent in a systemd service so it starts automatically and restarts on failure.

### QoS and Topic Notes

Set QoS durability to `TRANSIENT_LOCAL` for command topics so the RP2040 receives the last command immediately on reconnection:

```python
from rclpy.qos import QoSProfile, DurabilityPolicy
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
self.led_pub  = self.create_publisher(LedRingCommand, '/led_ring/command', qos)
self.lens_pub = self.create_publisher(LensCommand,    '/lens/command',     qos)
```

The `/camera_head/imu` topic publishes `sensor_msgs/Imu` in the `camera_optical_frame` and is compatible out of the box with `rviz2`, `robot_localization`, and any EKF/UKF node expecting standard ROS2 IMU data.

The `/ps_capture` action client on the host constructs a goal with:
- `patterns`: ordered PS illumination directions (e.g. N azimuth angles at fixed elevation)

The `dwell_ms` field from the original design is no longer needed — frame timing is now governed entirely by the camera's XVS cadence. The RP2040 advances to the next pattern only after the XVS edge confirming the current frame is complete.

**Host-side frame matching:** Run `libcamera` in continuous capture mode. Each frame carries a hardware timestamp. The PS node subscribes to the `/ps_capture` action feedback and, for each `(pattern_index, pattern_start_time)` pair received, finds the libcamera frame whose timestamp is closest to and after `pattern_start_time`. This is the frame illuminated by that pattern. The timestamp correlation window should be less than one frame period (e.g. < 33ms at 30fps).

A simple ROS2 node structure on the Pi:
```python
# Collect frames in a timestamped deque from libcamera
# Collect feedback messages from /ps_capture action
# On action result (success), zip patterns to frames by timestamp proximity
```# Inspection_EOAT_ROS2_Driver
# Inspection_EOAT_ROS2_Driver
