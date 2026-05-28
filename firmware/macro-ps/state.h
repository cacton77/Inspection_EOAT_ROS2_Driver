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

enum HomingPhase {
  HOMING_NONE = 0,
  HOMING_SEEKING_MAX,
  HOMING_SEEKING_MIN,
  HOMING_ZEROING,
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
