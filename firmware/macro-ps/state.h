#pragma once

#include <stdint.h>
#include "pico/mutex.h"
#include "pico/util/queue.h"
#include "config.h"

enum SystemMode {
  SYS_UNCALIBRATED = 0,
  SYS_HOMING,
  SYS_IDLE,
  SYS_LENS_MOVING,
  SYS_PS_CAPTURING,
};

// How to interpret lens_velocity_cmd / lens_target_steps. Mirrors
// ps_interfaces/msg/LensCommand's MODE_ constants. VELOCITY is 0 so a
// zero-initialised state means "hold", not "seek to the retracted limit".
enum LensCmdMode {
  LENS_CMD_VELOCITY = 0,
  LENS_CMD_POSITION = 1,
};

enum HomingPhase {
  HOMING_NONE = 0,
  HOMING_SEEKING_MAX,
  HOMING_SEEKING_MIN,
  HOMING_ZEROING,
};

// Manual range calibration, used while sensorless homing is unavailable.
// Set by the /lens/command callback on core 0 and serviced by stepper_tick()
// on core 1: the step counter belongs to the step ISR, and a core-0 write
// would race the ISR's own read-modify-write of it.
enum LensCalRequest : uint8_t {
  LENS_CAL_NONE = 0,
  LENS_CAL_SET_MIN,   // rezero: this position becomes step 0 and the minimum
  LENS_CAL_SET_MAX,   // this position becomes the maximum
};

struct SystemState {
  SystemMode  mode;
  HomingPhase homing_phase;

  // Lens
  float    lens_position_norm;
  float    lens_position_target;
  float    lens_velocity_cmd;
  int32_t  lens_steps;
  int32_t  lens_steps_min;
  int32_t  lens_steps_max;

  // Which command mode is live. The velocity deadman (VEL_WATCHDOG_MS) applies
  // only to LENS_CMD_VELOCITY: a position seek is issued once and must be
  // allowed to run to completion, so watchdogging it would kill the move
  // 150 ms in. README §5.4's callback sets last_vel_cmd_us only in velocity
  // mode, which is why this field has to exist.
  LensCmdMode lens_cmd_mode;

  // Position-mode goal in the step domain. Deriving it once on receipt and
  // comparing step counts makes the stop exact; re-deriving it from
  // lens_position_norm every tick would chatter around the target.
  int32_t  lens_target_steps;

  // TMC2209 answered over UART at boot. Without it there is no StallGuard, so
  // homing cannot run. Owned by stepper_init(), read by /lens/state.
  bool     driver_ok;

  // TMC2209 output stage is powered. Dropped after LENS_IDLE_DISABLE_MS at a
  // standstill so an idle lens holds no current — normal, not a fault. There is
  // no holding torque while this is false, so lens_steps is trusted only on the
  // mechanism's friction and the motor's detent torque. Owned by
  // stepper_enable(), read by /lens/state.
  bool     driver_enabled;

  // Last StallGuard reading. Refreshed by stepper_tick() while the motor turns
  // and held at its last value at rest, because SG_RESULT only means anything
  // under motion. Lower = higher load. Published so the threshold can be tuned
  // against real travel instead of guessed.
  uint16_t sg_result;

  // StallGuard fired outside homing: the lens hit something the stored
  // calibration did not predict, so the calibration is void. Latched until a
  // homing goal is accepted.
  bool     stall_latched;

  // Pending manual calibration request; see LensCalRequest.
  uint8_t  lens_cal_request;

  // LED state mirrored from /led_ring/command (canonical source from ROS).
  // Adafruit_NeoPixel's internal buffer is the realisation.
  uint32_t ring_colors[NUM_RING_PIXELS];
  uint32_t neokey_colors[NUM_NEOKEYS];

  // Last-command timestamps for watchdogs
  uint64_t last_vel_cmd_us;
  uint64_t last_ring_cmd_us;
};

struct ImuSample {
  float    omega_C[3];        // angular velocity in camera frame [rad/s]
  float    a_cam_origin[3];   // linear acceleration at camera origin [m/s^2]
  uint64_t timestamp_us;
};

extern mutex_t      state_mutex;
extern SystemState  state;
extern queue_t      imu_queue;

void state_init();
