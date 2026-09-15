#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// Board target
// =============================================================================
// Adafruit Feather RP2040 — FQBN rp2040:rp2040:adafruit_feather.
// Stepper driver is a BIGTREETECH TMC2209 step-stick.
//
// Every pin number below is an RP2040 GPIO, not an Arduino alias, except
// JOYSTICK_X/Y_PIN which are deliberately the A2/A3 aliases.
//
// Broken out on the Feather header and unassigned after everything below:
// GP8 (D6), GP13 (D13, shared with the onboard red LED), GP18 (SCK), GP19 (MO),
// GP20 (MI). The QT Py had none — this is what makes a quadrature encoder
// (A/B/I) and the XVS input possible on this board.
//
// GP7 was held for the encoder's A channel, but the NeoPixel chain is
// physically wired there, so the chain moved off GP24 and the encoder moved
// onto GP24/GP25 (see the encoder block at the end of this file). Both
// assignments still satisfy the PIO decoder's one real constraint — A and B on
// consecutive GPIOs — and this ordering additionally leaves SPI (GP18/19/20)
// whole, which the GP7/GP8 plan did not.
//
// Not brought out at all: GP14, GP15, GP17, GP21, GP22, GP23. GP16 is the
// onboard NeoPixel. Do not assign any of them.

// =============================================================================
// I2C — LSM6DSOX over STEMMA QT
// =============================================================================
// The Feather routes its STEMMA QT connector to the default Wire (GP2/GP3),
// which is the same pair as the SDA/SCL header pins. This is the reverse of
// the QT Py, where it was Wire1 on GP22/GP23 — imu.cpp must use Wire here.
// Wire1 does exist on this variant, but on GP24/GP25, and GP24 carries the
// NeoPixel chain, so nothing may call Wire1.begin().
#define I2C_SDA_PIN              2
#define I2C_SCL_PIN              3

// =============================================================================
// IMU (LSM6DSOX)
// =============================================================================
#define IMU_RATE_HZ              200
#define IMU_QUEUE_DEPTH          16
#define IMU_FRAME_ID             "eoat_camera_link"
#define IMU_ALPHA_BETA           0.2f   // IIR smoothing for angular acceleration

// IMU calibration: rotation S->C and translation t_CS in camera frame [m].
// Default = identity / zero (raw passthrough). Replace with values from CAD or
// physical calibration once mechanical assembly is finalised.
#define R_CS_00                  1.0f
#define R_CS_01                  0.0f
#define R_CS_02                  0.0f
#define R_CS_10                  0.0f
#define R_CS_11                  1.0f
#define R_CS_12                  0.0f
#define R_CS_20                  0.0f
#define R_CS_21                  0.0f
#define R_CS_22                  1.0f
#define T_CS_X                   0.042f
#define T_CS_Y                   0.0f
#define T_CS_Z                   0.0035f

// =============================================================================
// Onboard status LED (board-dependent, used to visualise micro-ROS agent state)
// =============================================================================
// Feather RP2040: single NeoPixel data on GP16, permanently powered. There is
// no power-enable pin, so STATUS_LED_POWER_PIN is -1 and status_led_init()
// skips the enable write it had to make for the QT Py's GP11.
//
// Freeing GP12 is the point of the move: it is the XVS input again (below).
#define STATUS_LED_PIN            16
#define STATUS_LED_POWER_PIN      -1
#define STATUS_LED_BRIGHTNESS     40
#define STATUS_LED_TICK_HZ        10

#define STATUS_COLOR_WAITING      0xFFA500  // amber — searching for agent
#define STATUS_COLOR_AVAILABLE    0x0000FF  // blue  — agent reachable, creating entities
#define STATUS_COLOR_CONNECTED    0x00FF00  // green — entities live, publishing
#define STATUS_COLOR_DISCONNECTED 0xFF0000  // red   — agent lost, tearing down

// =============================================================================
// micro-ROS
// =============================================================================
#define MICROROS_NODE_NAME       "camera_head"
#define MICROROS_NAMESPACE       "camera_head"
#define MICROROS_PING_TIMEOUT_MS 100
#define MICROROS_PING_PERIOD_US  500000   // 500 ms between pings while connected
#define MICROROS_SERIAL_BAUD     115200

// install.sh writes domain_id.h into the sketch dir from .env's ROS_DOMAIN_ID.
// Standalone arduino-cli builds (no install.sh wrapper) fall back to 0.
#if __has_include("domain_id.h")
#include "domain_id.h"
#endif
#ifndef MICROROS_DOMAIN_ID
#define MICROROS_DOMAIN_ID       0
#endif

// =============================================================================
// ROS topic names
// =============================================================================
// All names are relative to MICROROS_NAMESPACE (currently "camera_head"), so
// final fully-qualified topics resolve to /camera_head/<name>. rclc rejects
// leading slashes here, which is why we don't carry them in the literals.
#define IMU_TOPIC                "imu"
#define LED_RING_COMMAND_TOPIC   "led_ring/command"
#define LED_RING_STATE_TOPIC     "led_ring/state"
#define LENS_COMMAND_TOPIC       "lens/command"
#define LENS_STATE_TOPIC         "lens/state"
#define PS_TRIGGER_TOPIC         "ps_trigger"
#define PS_CAPTURE_ACTION        "ps_capture"
#define LENS_HOME_ACTION         "lens/home"

// =============================================================================
// State publishing rates (used once subsystems are implemented)
// =============================================================================
#define LED_STATE_RATE_HZ        10
#define LENS_STATE_RATE_HZ       20

// =============================================================================
// LED ring + NeoKeys (external NeoPixel chain — separate from the status LED)
// =============================================================================
// GP7 (D5), not the QT Py's GP24: this is where the chain is actually wired.
// Nothing else may claim GP7 — Adafruit_NeoPixel drives WS2812 timing from a
// PIO state machine, so a second owner would not merely share the pin, it
// would fail to acquire a state machine at all and die at led_init().
#define NEOPIXEL_DATA_PIN        7     // GP7 (D5)
#define LED_TICK_HZ              50    // render/compare rate; show() only on change
// Uniform scale applied by Adafruit_NeoPixel at show() time, and a power budget
// rather than an aesthetic choice. At the full 75-pixel chain a WS2812 draws
// ~60 mA at full white, so an all-white frame is ~4.5 A at 5 V — which a host
// bug can ask for in one message. At 128 that worst case is ~2.2 A, and with the
// full ring now enabled below that ceiling is live rather than theoretical.
//
// Real photometric-stereo patterns light a fraction of the ring at a time, so
// this mostly bounds the pathological frame rather than the working one. Raise
// it only against a measured supply rating, and note that it scales everything:
// host-side colour values are relative to this, not absolute radiometric units.
#define LED_RING_MAX_BRIGHTNESS  128

// -----------------------------------------------------------------------------
// Chain composition
// -----------------------------------------------------------------------------
// The full 72-pixel ring. Verified at one pixel first (R/G/B/W in order, which
// also confirmed the NEO_GRB channel order and that the 3.3 V data line drives
// a WS2812 at all) before being opened up to the whole strand.
//
// NUM_NEOKEYS stays 0 — the keys are STILL not on the strand, and this is not
// the place to pre-declare them. The chain map puts NeoKeys FIRST, so a non-zero
// count here shifts every ring pixel down the strand by that many positions:
// set this to 3 while they are unwired and ring pixel 0 lands on physical pixel
// 3 while the last three ring pixels fall off the end of the strand entirely.
// Raise it in the same change that physically fits the keys, not before.
//
// Driving FEWER pixels than are physically present is always safe — a WS2812
// takes the first 24 bits and passes the rest along — so cutting these counts
// back down for a future bring-up costs nothing but a recompile.
#define NUM_NEOKEYS              0     // 3 once the NeoKeys are physically fitted
#define NUM_RING_INNER           16
#define NUM_RING_MID             24
#define NUM_RING_OUTER           32
#define NUM_RING_PIXELS          (NUM_RING_INNER + NUM_RING_MID + NUM_RING_OUTER)
#define NUM_PIXELS_TOTAL         (NUM_NEOKEYS + NUM_RING_PIXELS)

// Wire capacities for LedRingCommand/LedRingState. These track the BOUNDS in the
// .msg files and must never be derived from the counts above.
//
// microcdr is handed the sequence capacity and rejects a message whose declared
// length exceeds it -- the WHOLE message, not the overflowing field. So if this
// followed NUM_RING_PIXELS down to 1, a host publishing a normal 72-entry frame
// would have every LedRingCommand discarded during deserialisation and nothing
// would light at all, with no error anywhere to say why. cb_led_cmd() clamps to
// the physical count after a successful decode; that is where truncation
// belongs, not here.
#define LED_RING_WIRE_MAX        72    // == LedRingCommand.colors bound
#define LED_NEOKEY_WIRE_MAX      3     // == LedRingCommand.neokey_colors bound

#define IDX_NEOKEY_PS            0
#define IDX_NEOKEY_MAG_PLUS      1
#define IDX_NEOKEY_MAG_MINUS     2
// Derived, not written out: the ring sits immediately after the NeoKeys, and
// each ring after the previous one. These used to be the literals 3/19/43, which
// silently encoded NUM_NEOKEYS == 3 in three more places.
#define OFFSET_INNER             (NUM_NEOKEYS)
#define OFFSET_MID               (OFFSET_INNER + NUM_RING_INNER)
#define OFFSET_OUTER             (OFFSET_MID + NUM_RING_MID)
// Ring radii and the Gaussian's sigmas used to live here. They are host-side
// now: inspection_eoat/ring_spot.py owns the geometry in millimetres (52/72/92)
// and the spot model, and this board just renders the indexed frame it is sent.

// Locally-generated NeoKey colours (ROS-commanded colours arrive via
// state.neokey_colors[] and are used where nothing local overrides them).
#define NEOKEY_COLOR_OFF         0x000000
#define NEOKEY_COLOR_HOMING      0xFFA500  // amber — homing phase indicator
#define NEOKEY_COLOR_PS_ACTIVE   0xFFFFFF  // white — PS key solid while capturing
#define LED_BLINK_HALF_PERIOD_US 250000ULL // 500 ms full blink period

// =============================================================================
// Stepper / lens — TMC2209 over UART
// =============================================================================
// The TMC2209's PDN_UART lands on the Feather's TX (GP0) / RX (GP1) pads,
// which arduino-pico's adafruit_feather variant maps to Serial1 (UART0). On
// the QT Py the same signals sat on GP20/GP5 and were Serial2; the object name
// tracks the variant rather than the hardware UART number, which is why it
// changes with the board.
//
// Serial2 must NOT be used here: the variant pins it to 31u (not brought out),
// so it would compile and then be silently dead. stepper.cpp pins Serial1 to
// TMC_UART_TX/RX_PIN explicitly so a wrong build target fails loudly instead.
//
// UART0 is free for this because micro-ROS runs over USB-CDC (`Serial`), not
// over the header UART.
#define TMC_SERIAL               Serial1
#define TMC_UART_TX_PIN          0     // GP0 (TX)
#define TMC_UART_RX_PIN          1     // GP1 (RX)
#define TMC_UART_BAUD            500000
#define STEPPER_STEP_PIN         27    // GP27 (A1)
#define STEPPER_DIR_PIN          26    // GP26 (A0)
#define STEPPER_ENABLE_PIN       6     // GP6  (D4), active HIGH = disabled
#define STEPPER_PULSE_US         2     // STEP high time
// Floor on the ISR's re-arm interval. Below this the alarm scheduling overhead
// starts to dominate the interval itself and the rate stops being honest.
#define STEPPER_MIN_INTERVAL_US  100
// Acceleration. A stepper commanded straight from rest to HOMING_VELOCITY is
// far above its pull-in rate and simply skips — the motor is a NEMA 11 (0.67 A,
// 12 N.cm), so its margin is thin. Every jog used to start with a step change in
// rate, which loses sync on the way up and silently desynchronises the open-loop
// step counter from the mechanism.
//
// LENS_START_VELOCITY is the rate the ramp begins at (and stops below), chosen
// to sit inside the pull-in region; LENS_ACCEL_STEPS_S2 then slews to the
// commanded rate. 8000 steps/s^2 reaches 1600 steps/s in 200 ms.
#define LENS_START_VELOCITY      100
#define LENS_ACCEL_STEPS_S2      8000
#define TMC_RSENSE               0.11f
#define TMC_DRIVER_ADDRESS       0b00
// The motor is a NEMA 11 rated 0.67 A/phase, and a stepper's rated current is a
// *peak* per phase, while TMCStepper's rms_current() sets the RMS value. The
// ceiling is therefore 670 / sqrt(2) = 474 mA RMS. The old 600 was 22% over it,
// and since the driver holds current at rest that was a permanent overload with
// nothing moving.
//
// The library quantises this anyway: 470 lands on vsense=1, CS=14, i.e.
// 459 mA RMS / 649 mA peak — just inside the rating.
//
// Torque scales with current, so this is ~79% of what STALL_THRESHOLD below was
// characterised against. See the note there.
#define TMC_RMS_CURRENT_MA       470
// Standstill current as a fraction of run current, and how long after the last
// step the driver waits before dropping to it. Both are the library/silicon
// defaults, written out because the point of this block is that the behaviour
// at rest should be deliberate rather than inherited. TPOWERDOWN is in units of
// 2^18 / f_CLK ~= 22 ms at the internal 12 MHz clock, so 10 is ~0.22 s.
#define TMC_HOLD_MULTIPLIER      0.5f
#define TMC_TPOWERDOWN           10
#define TMC_MICROSTEPS           16
// Measured on the rig with the tuner, not guessed. StallGuard4 is strongly
// velocity-dependent and needs real shaft speed: at 16 microsteps 200 steps/s
// is only 3.75 RPM, where SG_RESULT free-running medians ~4 and carries no
// load information at all. Free-travel medians measured across a sweep:
//
//     400 steps/s -> 4     1200 -> 68     2000 ->  78
//     800 steps/s -> 48    1600 -> 64     2400 -> 107
//
// 1600 is the sweet spot: the ISR tracks it to 99.8% and the signal is well
// clear of zero. Note the free-running floor is direction-dependent (24 moving
// positive, 54 moving negative over 4000-step runs), so the threshold has to
// clear the *worst* direction.
#define HOMING_VELOCITY          1600
#define LENS_DEFAULT_VELOCITY    800
// At HOMING_VELOCITY the old 20 microsteps was 12 ms of travel — not enough to
// unload the mechanism after a stall before the next leg starts measuring.
#define HOMING_BACKOFF_STEPS     200
// Measured against a real stall at HOMING_VELOCITY, once the acceleration ramp
// stopped the motor losing sync on every start (before the ramp, SG carried no
// load information at all and loaded runs read *higher* than free ones).
//
//   free travel at 1600 steps/s : 44 - 92   (median 62-88)
//   driven into the hard stop   : 18 - 20
//
// 30 sits between the two with roughly equal margin. The dips into the 20s that
// appear in the first few samples of a run are ramp/direction-reversal
// transients, not stalls — HOMING_STALL_GUARD_MS is what masks them, so do not
// shorten it without re-checking this.
//
// Note this is calibrated *at* HOMING_VELOCITY: StallGuard4 here is strongly
// velocity-dependent (free-travel median 4 at 400 steps/s, 88 at 1600), so
// changing HOMING_VELOCITY invalidates this number.
//
// It was also measured at the old 600 mA RMS. TMC_RMS_CURRENT_MA is now 470,
// which moves both the free-travel band and the stall floor, so treat 30 as
// provisional until it is re-measured on the rig with the tuner. Homing has not
// been run since the current change.
#define STALL_THRESHOLD          30
// StallGuard4 only updates while TSTEP <= TCOOLTHRS, and the register powers up
// at 0 -- which disables it outright. Without this, SG_RESULT reads low noise
// regardless of load: a velocity sweep on the real mechanism gave a mean of
// 11-29 across 100-800 steps/s, entirely below STALL_THRESHOLD, so homing would
// have declared a stall the moment its holdoff expired.
//
// TSTEP is f_CLK / microstep_rate with f_CLK ~= 12 MHz internal, so covering
// 100 steps/s needs TCOOLTHRS >= 120000. Use the full 20-bit range to keep
// StallGuard valid across the whole jog and homing speed band.
#define TMC_TCOOLTHRS            0xFFFFF
// SG_RESULT reads low whenever the motor isn't turning, so a fresh homing leg
// would "stall" instantly. Ignore StallGuard for this long after each direction
// change, and don't poll the (slow, blocking) UART read every tick.
#define HOMING_STALL_GUARD_MS    150
#define STALL_POLL_MS            10
// Velocity deadman: hold the lens if a jog stops being refreshed. This is sized
// against the *transport*, not against how fast the host publishes.
//
// It was 150 ms, which at the tuner's 20 Hz command rate is three command
// periods -- so any three consecutive late commands halted the jog. Measured on
// the rig: the host publishes at a rock-steady 50.0 ms (max 50.1 ms, zero DDS
// skips), the MCU's core 0 never blocks more than ~6 ms (the 16-deep 200 Hz IMU
// queue would drop samples if it did, and it never does), and position seeks --
// which are exempt from this deadman -- run at a clean 769 of 800 steps/s. Yet
// commands still reach cb_lens_cmd with recurring 150-200 ms holes on the
// agent -> USB-CDC -> client leg. Each hole tripped this timer, and because a
// halt drops ramp_velocity to zero, every trip cost a full re-accelerate: the
// jog stuttered roughly twice a second and averaged 578 of 800 steps/s.
//
// 500 ms is ten command periods, which absorbs every hole measured with room to
// spare while still stopping the lens within half a second of a genuinely dead
// publisher. Coasting at LENS_DEFAULT_VELOCITY for the full 500 ms is 400 steps,
// ~3.6% of the calibrated range -- and the step ISR's soft limits bound it
// absolutely regardless of what this timer does.
#define VEL_WATCHDOG_MS          500
// Cut the driver's output stage entirely after this long at a standstill. EN is
// active low and stepper_init() used to assert it at boot and never release it,
// so the motor sat at TMC_HOLD_MULTIPLIER x run current for as long as the MCU
// was powered — heat and wear with nothing moving, and unaffected by stopping
// the ROS stack.
//
// The trade is that a disabled driver has no holding torque, so the open-loop
// step count is then only as good as the mechanism's friction and the motor's
// detent torque. Long enough that a burst of jogs never de-energises mid-move,
// short enough that walking away from the rig doesn't cook the motor.
#define LENS_IDLE_DISABLE_MS     5000
#define LENS_POSITION_TOLERANCE  0.005f

// =============================================================================
// Joystick (stub — pin defines only)
// =============================================================================
// DEFERRED, and probably for good: the joystick is expected to connect to the
// Pi rather than the Feather, since the thing it drives — the spatial LED
// controller — now lives host-side. These pins are reserved rather than
// committed, and GP28/GP29 should be considered available if something else
// needs an ADC channel. SIGMA_A/SIGMA_R went with the Gaussian; see
// inspection_eoat/ring_spot.py.
#define JOYSTICK_X_PIN           A2    // GP28 on the Feather
#define JOYSTICK_Y_PIN           A3    // GP29 on the Feather
#define JOYSTICK_DEADZONE        0.05f

// =============================================================================
// NeoKey switches (stub — pin defines only)
// =============================================================================
// Digital inputs, internal pullup, active LOW. These supersede the original
// BUTTON_PS_PIN / BUTTON_MAG_PLUS_PIN / BUTTON_MAG_MINUS_PIN names.
//
// Note the switch numbering is chain order, not function order: the NeoKey
// pixel indices above put PS first (IDX_NEOKEY_PS == 0), while the switch
// numbering makes NEOKEY_1 = Mag+, NEOKEY_2 = Mag-, NEOKEY_3 = PS.
// buttons.cpp owns that mapping when it lands.
// Moved off the QT Py's GP25/26/27: GP26 and GP27 are now the stepper. Also
// unwired at present, so these are a proposal to build the harness against,
// not a record of existing wiring.
#define NEOKEY_1_PIN             9     // GP9  (D9)
#define NEOKEY_2_PIN             10    // GP10 (D10)
#define NEOKEY_3_PIN             11    // GP11 (D11)
#define BUTTON_DEBOUNCE_MS       10
#define MAG_STEP                 0.05f

// =============================================================================
// XVS camera sync (stub — pin reserved, ISR still unimplemented)
// =============================================================================
// GP12 is available again: it was the onboard NeoPixel on the QT Py, which is
// why XVS_PIN was intentionally absent there. The pin is reserved here, but
// xvs_stub.h still has an empty xvs_init() — routing the signal is no longer
// the blocker, writing the interrupt handler is.
//
// The signal is 1.8 V from the HQ camera and needs a level shifter to 3.3 V.
#define XVS_PIN                  12    // GP12 (D12)
#define XVS_TIMEOUT_US           200000
#define RING_SETTLE_MS           5

// =============================================================================
// Quadrature encoder (stub — pin defines only, nothing reads these yet)
// =============================================================================
// StepperOnline ME1K on the NEMA 11: magnetic incremental, 1000 PPR / 4000 CPR,
// differential line-driver A/B/Z.
//
// A and B must be CONSECUTIVE GPIOs with A the lower of the pair: the PIO
// quadrature program reads both phases with a single `in pins, 2` based at
// ENC_A_PIN. That constraint, plus keeping SPI intact, is what picks GP24/GP25
// — freed when the NeoPixel chain moved to GP7.
//
// Nothing may call Wire1.begin(): Wire1 is on GP24/GP25 on this variant. That
// restriction predates the encoder (it used to protect the NeoPixel chain on
// GP24) and now protects both encoder phases instead.
//
// The encoder CANNOT be wired straight to these pins. Its outputs sit ~1.2–1.4 V
// below its own supply — measured 1.9 V at VCC=3.3 V and 3.8 V at VCC=5 V, so
// the datasheet's "Output High Voltage: 5V" is simply wrong. The first is below
// the RP2040's guaranteed V_IH (0.65 × IOVDD = 2.145 V); the second is above its
// absolute maximum (IOVDD + 0.3 = 3.6 V). An AM26LV32 differential receiver
// powered at 3.3 V goes between the two, which also uses the complement legs and
// buys common-mode rejection on 500 mm of cable run beside StealthChop edges.
#define ENC_A_PIN                24    // GP24 (D24) — EA via AM26LV32
#define ENC_B_PIN                25    // GP25 (D25) — EB via AM26LV32
#define ENC_Z_PIN                8     // GP8  (D6)  — EZ via AM26LV32
// 1.8° at TMC_MICROSTEPS=16 is 3200 microsteps/rev against 4000 counts/rev, so
// counts = steps * 5 / 4 exactly — integer, no accumulating float error.
//
// Compare CUMULATIVELY, never per-microstep. A magnetic encoder interpolates
// its counts from a sine/cosine pair, and that interpolation error is tens of
// counts — but it is bounded and periodic within a revolution rather than
// accumulating, so a whole-move comparison is sound where a per-step one is
// pure noise. Z fires once per motor revolution and so is ambiguous on its own;
// it only becomes a datum once a limit switch establishes which revolution.
#define ENC_CPR                  4000
#define ENC_COUNTS_PER_STEP_NUM  5
#define ENC_COUNTS_PER_STEP_DEN  4

#endif  // CONFIG_H
