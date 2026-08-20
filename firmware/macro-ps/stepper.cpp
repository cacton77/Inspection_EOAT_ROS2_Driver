#include "stepper.h"

#include <Arduino.h>
#include <TMCStepper.h>
#include <math.h>

#include "config.h"
#include "state.h"

// TMC_SERIAL is Serial2 on the QT Py — see the wiring note in config.h.
static TMC2209Stepper driver(&TMC_SERIAL, TMC_RSENSE, TMC_DRIVER_ADDRESS);

// True once the driver has answered over UART. Stepping works either way;
// StallGuard (and therefore homing) does not.
static bool driver_ok = false;

// Step pacing. last_step_us == 0 means "idle", so the next commanded move
// starts on its first tick instead of waiting out a stale interval.
static uint64_t last_step_us = 0;
static int8_t   dir_latched  = 0;   // 0 = DIR pin not driven yet

// Homing internals — core 1 only, never shared.
static uint64_t phase_started_us = 0;
static uint64_t last_sg_poll_us  = 0;
static bool     backing_off      = false;
static int32_t  backoff_target   = 0;

// -----------------------------------------------------------------------------
// Low-level pin work
// -----------------------------------------------------------------------------
static inline void set_dir(int8_t d) {
  if (d == dir_latched) return;
  digitalWrite(STEPPER_DIR_PIN, d > 0 ? HIGH : LOW);
  dir_latched = d;
  delayMicroseconds(STEPPER_PULSE_US);   // DIR setup time before the next STEP
}

static inline void pulse_step() {
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  delayMicroseconds(STEPPER_PULSE_US);
  digitalWrite(STEPPER_STEP_PIN, LOW);
}

// Limits only mean anything once homing has actually recorded a range.
static inline bool limits_valid(int32_t lo, int32_t hi) { return hi > lo; }

static void write_position(int32_t steps) {
  mutex_enter_blocking(&state_mutex);
  state.lens_steps = steps;
  if (state.lens_steps_max > state.lens_steps_min) {
    state.lens_position_norm =
        (float)(steps - state.lens_steps_min)
      / (float)(state.lens_steps_max - state.lens_steps_min);
  }
  mutex_exit(&state_mutex);
}

// Halt and hand the lens back to IDLE. The mode demotion is deliberately
// conditional: we snapshot state outside the mutex, so core 0 may have moved
// the system on (into a capture, say) since we decided to stop. Killing the
// velocity is always right; overwriting someone else's newer mode is not.
static void stop_and_idle() {
  mutex_enter_blocking(&state_mutex);
  state.lens_velocity_cmd = 0.0f;
  if (state.mode == SYS_LENS_MOVING) state.mode = SYS_IDLE;
  mutex_exit(&state_mutex);
  last_step_us = 0;
}

// -----------------------------------------------------------------------------
// Homing FSM (README §4.4)
// -----------------------------------------------------------------------------
static void homing_reset() {
  backing_off      = false;
  phase_started_us = 0;
  last_sg_poll_us  = 0;
}

static void enter_phase(HomingPhase p) {
  mutex_enter_blocking(&state_mutex);
  state.homing_phase = p;
  mutex_exit(&state_mutex);
  phase_started_us = time_us_64();
  backing_off      = false;
}

// SG_RESULT reads low whenever the motor isn't turning, so a leg that has only
// just started would report an instant stall. Hold off for HOMING_STALL_GUARD_MS
// after each phase change, and rate-limit the read itself — it's a blocking
// UART round trip, far too slow to run every tick.
static bool stall_detected(uint64_t now_us) {
  if (now_us - phase_started_us < (uint64_t)HOMING_STALL_GUARD_MS * 1000ULL) return false;
  if (now_us - last_sg_poll_us  < (uint64_t)STALL_POLL_MS * 1000ULL)         return false;
  last_sg_poll_us = now_us;
  return driver.SG_RESULT() < STALL_THRESHOLD;
}

// Returns the velocity to run this tick, advancing the FSM as it goes.
static float homing_velocity(uint64_t now_us, HomingPhase phase, int32_t steps) {
  switch (phase) {
    case HOMING_NONE:
      // Freshly entered SYS_HOMING — kick off the first leg.
      enter_phase(HOMING_SEEKING_MAX);
      return 0.0f;

    case HOMING_SEEKING_MAX:
      if (backing_off) {
        if (steps <= backoff_target) {
          enter_phase(HOMING_SEEKING_MIN);
          return 0.0f;
        }
        return -(float)HOMING_VELOCITY;
      }
      if (stall_detected(now_us)) {
        mutex_enter_blocking(&state_mutex);
        state.lens_steps_max = steps;
        mutex_exit(&state_mutex);
        backing_off    = true;
        backoff_target = steps - HOMING_BACKOFF_STEPS;
        return 0.0f;
      }
      return (float)HOMING_VELOCITY;

    case HOMING_SEEKING_MIN:
      if (backing_off) {
        if (steps >= backoff_target) {
          enter_phase(HOMING_ZEROING);
          return 0.0f;
        }
        return (float)HOMING_VELOCITY;
      }
      if (stall_detected(now_us)) {
        mutex_enter_blocking(&state_mutex);
        state.lens_steps_min = steps;
        mutex_exit(&state_mutex);
        backing_off    = true;
        backoff_target = steps + HOMING_BACKOFF_STEPS;
        return 0.0f;
      }
      return -(float)HOMING_VELOCITY;

    case HOMING_ZEROING: {
      mutex_enter_blocking(&state_mutex);
      const int32_t lo = state.lens_steps_min;
      const int32_t hi = state.lens_steps_max;
      mutex_exit(&state_mutex);

      const int32_t mid = lo + (hi - lo) / 2;
      if (steps != mid) {
        return (steps < mid) ? (float)HOMING_VELOCITY : -(float)HOMING_VELOCITY;
      }

      // Arrived at the midpoint: the range is calibrated and we're centred.
      mutex_enter_blocking(&state_mutex);
      state.lens_velocity_cmd  = 0.0f;
      state.lens_position_norm = 0.5f;
      state.homing_phase       = HOMING_NONE;
      state.mode               = SYS_IDLE;
      mutex_exit(&state_mutex);
      homing_reset();
      return 0.0f;
    }
  }
  return 0.0f;
}

// Abort homing when there's no way to detect the hard stops.
static void homing_abort() {
  mutex_enter_blocking(&state_mutex);
  state.lens_velocity_cmd = 0.0f;
  state.homing_phase      = HOMING_NONE;
  state.mode              = SYS_UNCALIBRATED;
  mutex_exit(&state_mutex);
  homing_reset();
  last_step_us = 0;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool stepper_init() {
  pinMode(STEPPER_DIR_PIN,  OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  digitalWrite(STEPPER_DIR_PIN,  LOW);
  digitalWrite(STEPPER_STEP_PIN, LOW);

  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);   // active HIGH = disabled

  TMC_SERIAL.begin(TMC_UART_BAUD);
  driver.begin();
  driver.toff(5);
  driver.rms_current(TMC_RMS_CURRENT_MA);
  driver.microsteps(TMC_MICROSTEPS);
  driver.SGTHRS(STALL_THRESHOLD);
  driver.en_spreadCycle(false);            // StealthChop — quiet operation

  // test_connection() returns 0 only when DRV_STATUS reads back something
  // other than all-ones / all-zeros, i.e. the half-duplex link really works.
  driver_ok = (driver.test_connection() == 0);
  return driver_ok;
}

void stepper_tick() {
  const uint64_t now_us = time_us_64();

  // Snapshot shared state in one short critical section; everything after this
  // runs without the mutex so core 0's callbacks aren't blocked on our timing.
  mutex_enter_blocking(&state_mutex);
  const SystemMode  mode        = state.mode;
  const HomingPhase phase       = state.homing_phase;
  const uint64_t    last_vel_us = state.last_vel_cmd_us;
  const int32_t     steps_min   = state.lens_steps_min;
  const int32_t     steps_max   = state.lens_steps_max;
  int32_t           steps       = state.lens_steps;
  float             velocity    = state.lens_velocity_cmd;
  mutex_exit(&state_mutex);

  if (mode == SYS_HOMING) {
    if (!driver_ok) {
      // Without a UART link there's no StallGuard, and without StallGuard
      // homing would just drive the lens into its hard stop. Fail out.
      homing_abort();
      return;
    }
    velocity = homing_velocity(now_us, phase, steps);
  } else {
    homing_reset();

    // Velocity watchdog: a move that stops being refreshed is a dead link or a
    // dead publisher, so drop to a hold rather than running on stale command.
    if (mode == SYS_LENS_MOVING &&
        now_us - last_vel_us > (uint64_t)VEL_WATCHDOG_MS * 1000ULL) {
      stop_and_idle();
      return;
    }
  }

  // Below 1 step/s the interval maths stops being meaningful — treat as stopped.
  if (fabsf(velocity) < 1.0f) {
    last_step_us = 0;
    return;
  }

  const uint64_t interval_us = (uint64_t)(1000000.0f / fabsf(velocity));
  if (last_step_us != 0 && (now_us - last_step_us) < interval_us) return;

  const int8_t dir = (velocity > 0.0f) ? 1 : -1;

  // Soft limits. Homing is exempt — it's the thing that discovers the range.
  if (mode != SYS_HOMING && limits_valid(steps_min, steps_max)) {
    const int32_t next = steps + dir;
    if (next < steps_min || next > steps_max) {
      stop_and_idle();
      return;
    }
  }

  set_dir(dir);
  pulse_step();
  last_step_us = now_us;

  steps += dir;
  write_position(steps);
}
