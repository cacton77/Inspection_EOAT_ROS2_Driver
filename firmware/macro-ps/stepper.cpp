#include "stepper.h"

#include <Arduino.h>
#include <TMCStepper.h>
#include <limits.h>
#include <math.h>

#include "hardware/timer.h"
#include "pico/time.h"

#include "config.h"
#include "state.h"

// TMC_SERIAL is Serial1 on the Feather — see the wiring note in config.h.
static TMC2209Stepper driver(&TMC_SERIAL, TMC_RSENSE, TMC_DRIVER_ADDRESS);

// True once the driver has answered over UART. Stepping works either way;
// StallGuard (and therefore homing) does not.
static bool driver_ok = false;

// Output-stage state and the idle timer that drives it. EN is active low, so
// the coils are energised for as long as the pin is held down; see
// LENS_IDLE_DISABLE_MS. Boots disabled — nothing has commanded motion yet, and
// the UART configuration in stepper_init() works regardless of EN.
static bool     drv_enabled    = false;
static uint64_t last_motion_us = 0;

// -----------------------------------------------------------------------------
// Step generation — hardware alarm ISR (README §4.3)
// -----------------------------------------------------------------------------
// Pulses used to be paced from stepper_tick() inside loop1(), sharing the loop
// with imu_tick()'s blocking I2C read, the NeoPixel refresh and the StallGuard
// UART poll. Measured on the rig: 400 steps/s commanded gave 253 steps/s actual,
// with per-interval rates scattered between 118 and 285. StallGuard only means
// anything at a constant velocity, so that alone made homing untunable.
//
// Timing now lives in a hardware alarm ISR, which preempts everything else on
// core 1. stepper_tick() is reduced to policy: it decides direction, rate and
// the range the ISR may move within, then arms it.
//
// The ISR and the policy layer both run on core 1 — the ISR preempts the policy
// but they never run concurrently — so plain volatiles are enough here. No
// mutex is taken in the ISR: mutex_enter_blocking() from an interrupt that has
// preempted the holder would deadlock the core.
static int step_alarm = -1;

static volatile uint32_t step_interval_us = 0;            // 0 = idle
static volatile int8_t   step_dir_v       = 1;
static volatile int32_t  step_count       = 0;            // ISR owns this
static volatile int32_t  step_stop_min    = INT32_MIN;
static volatile int32_t  step_stop_max    = INT32_MAX;
static volatile bool     step_armed       = false;
static volatile bool     step_limit_hit   = false;
static volatile uint64_t step_next_us     = 0;

static int8_t  dir_latched          = 0;   // 0 = DIR pin not driven yet

// Ramp state. ramp_velocity is what we actually run; the policy layer's
// velocity is only ever a target to slew toward.
static float    ramp_velocity = 0.0f;
static uint64_t last_ramp_us  = 0;
static int32_t published_steps      = INT32_MIN;

// hardware_alarm_set_target() reports true when the target had already passed,
// in which case nothing is scheduled and the train would stall. Retry a little
// further out until it takes.
static inline void step_schedule(uint64_t at_us) {
  step_next_us = at_us;
  while (hardware_alarm_set_target(step_alarm, from_us_since_boot(step_next_us))) {
    step_next_us = time_us_64() + 3;
  }
}

static void step_isr(uint alarm_num) {
  (void)alarm_num;
  const uint32_t interval = step_interval_us;
  if (interval == 0) {
    step_armed = false;
    return;
  }

  const int32_t next = step_count + step_dir_v;
  if (next < step_stop_min || next > step_stop_max) {
    // Enforced here rather than in the policy layer so that a starved loop1()
    // can never walk the lens past a soft limit or a position target.
    step_interval_us = 0;
    step_armed       = false;
    step_limit_hit   = true;
    return;
  }

  digitalWrite(STEPPER_STEP_PIN, HIGH);
  delayMicroseconds(STEPPER_PULSE_US);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  step_count = next;

  // Advance from the previous scheduled time, not from now, so the ISR's own
  // entry latency does not accumulate into a slow drift.
  uint64_t next_at = step_next_us + interval;
  const uint64_t now = time_us_64();
  if (next_at <= now) next_at = now + interval;   // fell behind; resynchronise
  step_schedule(next_at);
}

static void stepper_disarm() {
  // Any halt — watchdog, soft limit, direction change — also drops the ramp, so
  // motion never resumes at speed from a standstill.
  ramp_velocity    = 0.0f;
  step_interval_us = 0;
  if (step_alarm >= 0) hardware_alarm_cancel(step_alarm);
  step_armed = false;
}

static void stepper_arm(uint32_t interval_us) {
  step_interval_us = interval_us;
  if (step_armed || step_alarm < 0) return;   // already running: rate updated
  step_armed = true;
  step_schedule(time_us_64() + interval_us);
}

// EN is active low. Deliberately does not block: a fresh enable is always
// followed by stepper_arm() at LENS_START_VELOCITY, whose first STEP edge is a
// full 10 ms out, which is far more settling than the output stage needs. A
// delay here would stall loop1() — and with it imu_tick() — for nothing.
static void stepper_enable(bool on) {
  if (on == drv_enabled) return;
  digitalWrite(STEPPER_ENABLE_PIN, on ? LOW : HIGH);
  drv_enabled = on;
  // Guarded by the early return above, so the mutex is only touched on a real
  // transition rather than on every armed tick.
  mutex_enter_blocking(&state_mutex);
  state.driver_enabled = on;
  mutex_exit(&state_mutex);
}

// Homing internals — core 1 only, never shared.
static uint64_t phase_started_us = 0;
static uint64_t last_sg_poll_us  = 0;
static bool     backing_off      = false;
static int32_t  backoff_target   = 0;

// -----------------------------------------------------------------------------
// Low-level pin work
// -----------------------------------------------------------------------------
// DIR has to be stable before the next STEP edge, and flipping it under a live
// ISR could land inside a pulse. Stop the train, change direction, then let the
// caller re-arm.
static void stepper_set_dir(int8_t d) {
  if (d == dir_latched) return;
  stepper_disarm();
  digitalWrite(STEPPER_DIR_PIN, d > 0 ? HIGH : LOW);
  dir_latched = d;
  step_dir_v  = d;
  delayMicroseconds(STEPPER_PULSE_US);   // DIR setup time
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
  if (state.mode == SYS_LENS_MOVING) {
    // SYS_LENS_MOVING does not record whether we were calibrated when the move
    // started, so recover it from the range itself. Without this, an open-loop
    // jog on an unhomed lens would stop into SYS_IDLE and report the lens as
    // calibrated and ready.
    state.mode = (state.lens_steps_max > state.lens_steps_min)
                   ? SYS_IDLE : SYS_UNCALIBRATED;
  }
  mutex_exit(&state_mutex);
  stepper_disarm();
}

// -----------------------------------------------------------------------------
// Homing FSM (README §4.4)
// -----------------------------------------------------------------------------
static void homing_reset() {
  backing_off      = false;
  phase_started_us = 0;
  // Deliberately does NOT reset last_sg_poll_us: that is sample_stallguard()'s
  // rate limiter now, and homing_reset() runs on every non-homing tick, so
  // clearing it here would force a blocking UART read per step. The
  // HOMING_STALL_GUARD_MS holdoff already covers the start of a homing leg.
}

static void enter_phase(HomingPhase p) {
  mutex_enter_blocking(&state_mutex);
  state.homing_phase = p;
  mutex_exit(&state_mutex);
  phase_started_us = time_us_64();
  backing_off      = false;
}

// Sample StallGuard and publish it into shared state. Rate-limited because the
// read is a blocking UART round trip (~320 us at 500 kbaud) and stepper_tick()
// is also pacing step pulses — polling it every tick would visibly jitter the
// step train. Returns the cached value when it is not yet time to re-read.
//
// Only called while the motor is turning: SG_RESULT is undefined at standstill,
// so holding the last value is more useful than publishing a meaningless 0.
static uint16_t sample_stallguard(uint64_t now_us) {
  static uint16_t cached = 0;
  if (!driver_ok) return 0;
  if (now_us - last_sg_poll_us < (uint64_t)STALL_POLL_MS * 1000ULL) return cached;
  last_sg_poll_us = now_us;
  cached = driver.SG_RESULT();
  mutex_enter_blocking(&state_mutex);
  state.sg_result = cached;
  mutex_exit(&state_mutex);
  return cached;
}

// A homing leg that has only just started would report an instant stall, since
// SG_RESULT reads low until the motor is actually turning. Hold off for
// HOMING_STALL_GUARD_MS after each phase change.
static bool stall_detected(uint64_t now_us) {
  if (now_us - phase_started_us < (uint64_t)HOMING_STALL_GUARD_MS * 1000ULL) return false;
  return sample_stallguard(now_us) < STALL_THRESHOLD;
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
  stepper_disarm();
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool stepper_init() {
  // Claimed from setup1(), so the alarm IRQ is enabled on core 1 and its ISR
  // preempts loop1() — exactly the work that was starving the step train.
  step_alarm = hardware_alarm_claim_unused(false);
  if (step_alarm >= 0) {
    hardware_alarm_set_callback(step_alarm, step_isr);
  }

  pinMode(STEPPER_DIR_PIN,  OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  digitalWrite(STEPPER_DIR_PIN,  LOW);
  digitalWrite(STEPPER_STEP_PIN, LOW);

  // Boot with the output stage off. It comes up on the first commanded move
  // and drops out again LENS_IDLE_DISABLE_MS after the last one.
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);   // active HIGH = disabled
  drv_enabled = false;

  // Pin the UART to the header pads explicitly. Serial1 already defaults to
  // GP0/GP1 on this variant, but stating it means a build against the wrong
  // board target fails here rather than coming up mute on unrouted pins.
  TMC_SERIAL.setTX(TMC_UART_TX_PIN);
  TMC_SERIAL.setRX(TMC_UART_RX_PIN);
  TMC_SERIAL.begin(TMC_UART_BAUD);
  driver.begin();
  driver.toff(5);
  // GCONF.I_scale_analog powers up set, which scales run current by the VREF
  // pin — i.e. by whatever the module's trimpot happens to be at, making
  // rms_current() advisory rather than authoritative. Clear it so the register
  // actually governs; StallGuard is a load measurement and is meaningless if
  // the current it is measured against is unknown.
  driver.I_scale_analog(false);
  // Second argument is the hold multiplier — the library defaults to this same
  // 0.5, but standstill dissipation is the whole reason this block was revised,
  // so it is stated rather than inherited. TPOWERDOWN is how long after the last
  // step the driver waits before ramping down to it.
  driver.rms_current(TMC_RMS_CURRENT_MA, TMC_HOLD_MULTIPLIER);
  driver.TPOWERDOWN(TMC_TPOWERDOWN);
  driver.microsteps(TMC_MICROSTEPS);
  driver.SGTHRS(STALL_THRESHOLD);
  driver.en_spreadCycle(false);            // StealthChop — StallGuard4 needs it
  // Without TCOOLTHRS StallGuard never updates; see the note in config.h.
  driver.TCOOLTHRS(TMC_TCOOLTHRS);

  // test_connection() returns 0 only when DRV_STATUS reads back something
  // other than all-ones / all-zeros, i.e. the half-duplex link really works.
  driver_ok = (driver.test_connection() == 0);

  // Surface it so /lens/state can tell the host why homing will refuse to run.
  mutex_enter_blocking(&state_mutex);
  state.driver_ok      = driver_ok;
  state.driver_enabled = false;
  mutex_exit(&state_mutex);

  return driver_ok;
}

void stepper_tick() {
  const uint64_t now_us = time_us_64();

  // The ISR owns the step count now; publish whatever it produced since the
  // last tick before anything else reads position.
  const int32_t steps = step_count;
  if (steps != published_steps) {
    write_position(steps);
    published_steps = steps;
  }

  // ---- manual range calibration ----------------------------------------
  // Done here, not in the command callback, because step_count is owned by the
  // step ISR on this core. stepper_disarm() first so the ISR cannot fire
  // between the rezero and the state update.
  mutex_enter_blocking(&state_mutex);
  const uint8_t cal_req = state.lens_cal_request;
  state.lens_cal_request = LENS_CAL_NONE;
  mutex_exit(&state_mutex);

  if (cal_req != LENS_CAL_NONE) {
    stepper_disarm();
    // Force write_position() to recompute on the next tick. Both requests
    // change the range, so the cached position_norm is stale even though the
    // step count has not moved -- without this the lens reports 0.0 until
    // something happens to jog it. (write_position() takes the state mutex
    // itself, so it cannot simply be called from inside the block below.)
    published_steps = INT32_MIN;
    mutex_enter_blocking(&state_mutex);
    if (cal_req == LENS_CAL_SET_MIN) {
      step_count           = 0;
      state.lens_steps     = 0;
      state.lens_steps_min = 0;
      // The far end goes with it: a new zero below the old max would otherwise
      // leave a stale range that position seeks would happily honour.
      state.lens_steps_max = 0;
      state.stall_latched  = false;
    } else {
      state.lens_steps_max = step_count;
    }
    state.lens_velocity_cmd = 0.0f;
    state.lens_cmd_mode     = LENS_CMD_VELOCITY;
    state.mode = limits_valid(state.lens_steps_min, state.lens_steps_max)
                   ? SYS_IDLE : SYS_UNCALIBRATED;
    mutex_exit(&state_mutex);
    return;
  }

  mutex_enter_blocking(&state_mutex);
  const SystemMode  mode        = state.mode;
  const HomingPhase phase       = state.homing_phase;
  const uint64_t    last_vel_us = state.last_vel_cmd_us;
  const int32_t     steps_min   = state.lens_steps_min;
  const int32_t     steps_max   = state.lens_steps_max;
  const LensCmdMode cmd_mode    = state.lens_cmd_mode;
  const int32_t     target      = state.lens_target_steps;
  float             velocity    = state.lens_velocity_cmd;
  mutex_exit(&state_mutex);

  // The ISR halts itself on a limit or a position target; policy just has to
  // notice and settle the mode.
  const bool limit_hit = step_limit_hit;
  if (limit_hit) step_limit_hit = false;

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

    if (limit_hit) {          // soft limit or position target reached
      stop_and_idle();
      return;
    }
    if (mode == SYS_LENS_MOVING && cmd_mode == LENS_CMD_VELOCITY) {
      // Velocity watchdog: a jog that stops being refreshed is a dead link or a
      // dead publisher, so drop to a hold rather than running on a stale
      // command. Deliberately not applied in position mode — that goal is sent
      // once and would be killed 150 ms into the move.
      if (now_us - last_vel_us > (uint64_t)VEL_WATCHDOG_MS * 1000ULL) {
        stop_and_idle();
        return;
      }
    } else if (mode == SYS_LENS_MOVING && cmd_mode == LENS_CMD_POSITION &&
               steps == target) {
      stop_and_idle();
      return;
    }
  }

  // ---- acceleration ramp -----------------------------------------------
  // loop1() is not periodic, so derive dt from the tick itself rather than
  // assuming a rate.
  const uint64_t dt_us = (last_ramp_us == 0) ? 0 : (now_us - last_ramp_us);
  last_ramp_us = now_us;
  const float dv = (float)LENS_ACCEL_STEPS_S2 * (float)dt_us * 1e-6f;
  if      (velocity > ramp_velocity) ramp_velocity = fminf(velocity, ramp_velocity + dv);
  else if (velocity < ramp_velocity) ramp_velocity = fmaxf(velocity, ramp_velocity - dv);

  if (fabsf(velocity) < 1.0f) {
    // Commanded stop: decelerate, then halt once we are back inside the
    // pull-in region so the last step does not lose sync either.
    if (fabsf(ramp_velocity) < (float)LENS_START_VELOCITY) {
      stepper_disarm();
      // Standstill. Keep the coils energised for LENS_IDLE_DISABLE_MS so a
      // burst of jogs retains its holding torque between moves, then cut the
      // output stage. Homing is exempt: its phase transitions pass through a
      // zero-velocity tick, and de-energising for one tick mid-home would drop
      // the position the whole procedure is trying to establish.
      if (drv_enabled && mode != SYS_HOMING &&
          now_us - last_motion_us > (uint64_t)LENS_IDLE_DISABLE_MS * 1000ULL) {
        stepper_enable(false);
      }
      return;
    }
  } else if (fabsf(ramp_velocity) < (float)LENS_START_VELOCITY) {
    // Start at the pull-in rate rather than crawling up from zero.
    ramp_velocity = (velocity > 0.0f) ? (float)LENS_START_VELOCITY
                                      : -(float)LENS_START_VELOCITY;
  }
  velocity = ramp_velocity;

  // Hand the ISR the range it may move within. Homing is exempt from the soft
  // limits — it is the thing that discovers them.
  int32_t lo = INT32_MIN, hi = INT32_MAX;
  if (mode != SYS_HOMING) {
    if (limits_valid(steps_min, steps_max)) {
      lo = steps_min;
      hi = steps_max;
    }
    if (cmd_mode == LENS_CMD_POSITION && mode == SYS_LENS_MOVING) {
      // Clamping the far end to the target is what makes the position stop
      // exact: the ISR simply refuses the step that would pass it.
      if (target > steps) { if (target < hi) hi = target; }
      else                { if (target > lo) lo = target; }
    }
  }
  step_stop_min = lo;
  step_stop_max = hi;

  // About to move: power the output stage and restart the idle timer. Both are
  // cheap no-ops on the overwhelmingly common already-moving tick.
  stepper_enable(true);
  last_motion_us = now_us;

  stepper_set_dir((velocity > 0.0f) ? 1 : -1);

  uint32_t interval = (uint32_t)(1000000.0f / fabsf(velocity));
  if (interval < STEPPER_MIN_INTERVAL_US) interval = STEPPER_MIN_INTERVAL_US;
  stepper_arm(interval);

  // Keep StallGuard fresh during ordinary jogs and seeks, not just homing —
  // this is the data the threshold is tuned from, and D4 needs it to notice a
  // lens running into a limit the stored calibration did not predict.
  if (mode != SYS_HOMING) sample_stallguard(now_us);
}
