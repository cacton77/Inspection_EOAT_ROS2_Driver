// Camera Head Controller — macro-PS firmware.
// Current iteration: LSM6DSOX → /camera_head/imu over USB-CDC micro-ROS.
// Other subsystems (LED, stepper, joystick, buttons, XVS) are scaffolded
// as no-op stubs and will be filled in subsequent iterations.

#include <Arduino.h>

#include "config.h"
#include "state.h"
#include "imu.h"
#include "microros.h"
#include "status_led.h"

#include "led_stub.h"
#include "stepper_stub.h"
#include "joystick_stub.h"
#include "buttons_stub.h"
#include "xvs_stub.h"

// -----------------------------------------------------------------------------
// Core 0 — micro-ROS executor + IMU drain + agent lifecycle
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(MICROROS_SERIAL_BAUD);
  state_init();
  microros_init_transport();
  xvs_init();   // ISR lives on core 0 alongside the executor
  // Give the agent transport a moment to settle before the first ping.
  delay(2000);
}

void loop() {
  static uint64_t last_ping_us = 0;
  const uint64_t  now_us = time_us_64();

  switch (agent_state) {
    case AGENT_STATE_WAITING_AGENT:
      if (now_us - last_ping_us > MICROROS_PING_PERIOD_US) {
        last_ping_us = now_us;
        if (microros_ping(MICROROS_PING_TIMEOUT_MS)) {
          agent_state = AGENT_STATE_AVAILABLE;
        }
      }
      break;

    case AGENT_STATE_AVAILABLE:
      if (microros_create_entities()) {
        agent_state = AGENT_STATE_CONNECTED;
      } else {
        microros_destroy_entities();
        agent_state = AGENT_STATE_WAITING_AGENT;
      }
      break;

    case AGENT_STATE_CONNECTED:
      microros_drain_and_publish();
      if (now_us - last_ping_us > MICROROS_PING_PERIOD_US) {
        last_ping_us = now_us;
        if (!microros_ping(MICROROS_PING_TIMEOUT_MS)) {
          agent_state = AGENT_STATE_DISCONNECTED;
        }
      }
      break;

    case AGENT_STATE_DISCONNECTED:
      microros_destroy_entities();
      mutex_enter_blocking(&state_mutex);
      state.lens_velocity_cmd = 0.0f;
      if (state.mode == SYS_HOMING)             state.mode = SYS_UNCALIBRATED;
      else if (state.mode == SYS_PS_CAPTURING)  state.mode = SYS_IDLE;
      mutex_exit(&state_mutex);
      agent_state = AGENT_STATE_WAITING_AGENT;
      break;
  }
}

// -----------------------------------------------------------------------------
// Core 1 — peripherals (IMU now; LED/stepper/joystick/buttons later)
// -----------------------------------------------------------------------------
void setup1() {
  // Wait for core 0's state_init() to finish before touching shared state.
  while (!mutex_is_initialized(&state_mutex)) {
    tight_loop_contents();
  }

  // Bring the status LED up first so agent_state stays visible even if a
  // later peripheral init fails.
  status_led_init();

  if (!imu_init()) {
    // Sensor not responding. Lock core 1 and flash a distinct fault pattern
    // so the failure isn't masked by the agent_state colour.
    while (true) {
      status_led_signal_imu_fault();
      delay(50);
    }
  }
  led_init();
  stepper_init();
  joystick_init();
  buttons_init();
}

void loop1() {
  status_led_tick();
  imu_tick();
  led_tick();
  stepper_tick();
  joystick_tick();
  buttons_tick();
}
