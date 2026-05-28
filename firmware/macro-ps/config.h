#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// Board target
// =============================================================================
// Currently building for the Adafruit Feather RP2040.

// =============================================================================
// I2C — LSM6DSOX over STEMMA QT
// =============================================================================
// On arduino-pico's Feather RP2040 variant, the STEMMA QT connector is wired to
// the default Wire instance on GP2 (SDA) / GP3 (SCL). Pins listed for
// reference only — imu.cpp must use Wire (not Wire1) on this board.
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
// Feather RP2040: single NeoPixel data on GP16, no power-enable pin.
#define STATUS_LED_PIN            16
#define STATUS_LED_POWER_PIN      -1        // -1 = no power-enable pin
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
// LED ring (placeholder — pending real implementation)
// =============================================================================
#define NEOPIXEL_DATA_PIN        18    // Feather RP2040 GP18
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

// =============================================================================
// Stepper / lens (placeholder)
// =============================================================================
#define TMC_UART_BAUD            500000
#define STEPPER_DIR_PIN          9
#define STEPPER_STEP_PIN         10
#define STEPPER_ENABLE_PIN       11
#define TMC_RSENSE               0.11f
#define TMC_DRIVER_ADDRESS       0b00
#define TMC_RMS_CURRENT_MA       600
#define TMC_MICROSTEPS           16
#define HOMING_VELOCITY          200
#define LENS_DEFAULT_VELOCITY    500
#define HOMING_BACKOFF_STEPS     20
#define STALL_THRESHOLD          50
#define VEL_WATCHDOG_MS          150
#define LENS_POSITION_TOLERANCE  0.005f

// =============================================================================
// Joystick / buttons (placeholder)
// =============================================================================
#define JOYSTICK_X_PIN           A0
#define JOYSTICK_Y_PIN           A1
#define JOYSTICK_DEADZONE        0.05f
#define BUTTON_PS_PIN            6
#define BUTTON_MAG_PLUS_PIN      28
#define BUTTON_MAG_MINUS_PIN     29
#define BUTTON_DEBOUNCE_MS       10
#define MAG_STEP                 0.05f
#define SIGMA_A                  0.5236f   // ~M_PI/6
#define SIGMA_R                  0.15f

// =============================================================================
// XVS camera sync (placeholder)
// =============================================================================
#define XVS_PIN                  12
#define XVS_TIMEOUT_US           200000
#define RING_SETTLE_MS           5

#endif  // CONFIG_H
