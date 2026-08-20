#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// Board target
// =============================================================================
// Adafruit QT Py RP2040 — FQBN rp2040:rp2040:adafruit_qtpy.
// Stepper driver is a BIGTREETECH TMC2209 step-stick.
//
// Every pin number below is an RP2040 GPIO, not an Arduino alias, except
// JOYSTICK_X/Y_PIN which are deliberately the A0/A1 aliases.

// =============================================================================
// I2C — LSM6DSOX over STEMMA QT
// =============================================================================
// The QT Py routes its STEMMA QT connector to Wire1 (GP22/GP23), unlike the
// Feather where it was the default Wire. imu.cpp must use Wire1 on this board.
// Wire's default pads (GP24/GP25) carry the NeoPixel chain and NeoKey 3 here,
// so nothing may call Wire.begin().
#define I2C_SDA_PIN              22
#define I2C_SCL_PIN              23

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
// QT Py RP2040: single NeoPixel data on GP12, with a power-enable on GP11 that
// must be driven HIGH before the pixel responds. status_led_init() does that
// for any STATUS_LED_POWER_PIN >= 0, so setting it here is the whole change.
#define STATUS_LED_PIN            12
#define STATUS_LED_POWER_PIN      11
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
#define NEOPIXEL_DATA_PIN        24    // GP24 (SDA / D4)
#define LED_TICK_HZ              50    // render/compare rate; show() only on change
#define NUM_NEOKEYS              3
#define NUM_RING_INNER           16
#define NUM_RING_MID             24
#define NUM_RING_OUTER           32
#define NUM_RING_PIXELS          (NUM_RING_INNER + NUM_RING_MID + NUM_RING_OUTER)
#define NUM_PIXELS_TOTAL         (NUM_NEOKEYS + NUM_RING_PIXELS)
#define IDX_NEOKEY_PS            0
#define IDX_NEOKEY_MAG_PLUS      1
#define IDX_NEOKEY_MAG_MINUS     2
#define OFFSET_INNER             3
#define OFFSET_MID               19
#define OFFSET_OUTER             43
#define RADIUS_INNER             0.5652f
#define RADIUS_MID               0.7826f
#define RADIUS_OUTER             1.0f

// Locally-generated NeoKey colours (ROS-commanded colours arrive via
// state.neokey_colors[] and are used where nothing local overrides them).
#define NEOKEY_COLOR_OFF         0x000000
#define NEOKEY_COLOR_HOMING      0xFFA500  // amber — homing phase indicator
#define NEOKEY_COLOR_PS_ACTIVE   0xFFFFFF  // white — PS key solid while capturing
#define LED_BLINK_HALF_PERIOD_US 250000ULL // 500 ms full blink period

// =============================================================================
// Stepper / lens — TMC2209 over UART
// =============================================================================
// The TMC2209's PDN_UART lands on the QT Py's TX (GP20) / RX (GP5) pads. In
// arduino-pico's adafruit_qtpy variant those pads are Serial2, not Serial1
// (Serial1 defaults to GP28/GP29, which are the joystick's A1/A0). Keeping the
// physical pins from the wiring table therefore means using Serial2 — the
// object name differs from the hardware UART number and that is expected.
#define TMC_SERIAL               Serial2
#define TMC_UART_BAUD            500000
#define STEPPER_STEP_PIN         6     // GP6 (SCK / D8)
#define STEPPER_DIR_PIN          4     // GP4 (MI  / D9)
#define STEPPER_ENABLE_PIN       3     // GP3 (MO  / D10), active HIGH = disabled
#define STEPPER_PULSE_US         2     // STEP high time
#define TMC_RSENSE               0.11f
#define TMC_DRIVER_ADDRESS       0b00
#define TMC_RMS_CURRENT_MA       600
#define TMC_MICROSTEPS           16
#define HOMING_VELOCITY          200
#define LENS_DEFAULT_VELOCITY    500
#define HOMING_BACKOFF_STEPS     20
#define STALL_THRESHOLD          50
// SG_RESULT reads low whenever the motor isn't turning, so a fresh homing leg
// would "stall" instantly. Ignore StallGuard for this long after each direction
// change, and don't poll the (slow, blocking) UART read every tick.
#define HOMING_STALL_GUARD_MS    150
#define STALL_POLL_MS            10
#define VEL_WATCHDOG_MS          150
#define LENS_POSITION_TOLERANCE  0.005f

// =============================================================================
// Joystick (stub — pin defines only)
// =============================================================================
#define JOYSTICK_X_PIN           A0    // GP29 on the QT Py
#define JOYSTICK_Y_PIN           A1    // GP28 on the QT Py
#define JOYSTICK_DEADZONE        0.05f
#define SIGMA_A                  0.5236f   // ~M_PI/6
#define SIGMA_R                  0.15f

// =============================================================================
// NeoKey switches (stub — pin defines only)
// =============================================================================
// Digital inputs, internal pullup, active LOW. These replace the Feather's
// BUTTON_PS_PIN / BUTTON_MAG_PLUS_PIN / BUTTON_MAG_MINUS_PIN.
//
// Note the switch numbering is chain order, not function order: the NeoKey
// pixel indices above put PS first (IDX_NEOKEY_PS == 0), while the switch
// wiring carried over from the Feather makes NEOKEY_1 = Mag+, NEOKEY_2 = Mag-,
// NEOKEY_3 = PS. buttons.cpp owns that mapping when it lands.
#define NEOKEY_1_PIN             27    // GP27 (A2 / D2)
#define NEOKEY_2_PIN             26    // GP26 (A3 / D3)
#define NEOKEY_3_PIN             25    // GP25 (SCL / D5)
#define BUTTON_DEBOUNCE_MS       10
#define MAG_STEP                 0.05f

// =============================================================================
// XVS camera sync (stub — no GPIO available)
// =============================================================================
// The QT Py has no free pin for XVS: GP12 (the Feather's XVS pin) is the
// onboard NeoPixel here. XVS_PIN is intentionally absent; the timing constants
// are kept for whenever the signal can be routed.
#define XVS_TIMEOUT_US           200000
#define RING_SETTLE_MS           5

#endif  // CONFIG_H
