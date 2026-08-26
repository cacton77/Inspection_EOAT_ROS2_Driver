// Camera Head Controller — macro-PS firmware. Target: Adafruit QT Py RP2040.
// Current iteration: LSM6DSOX → /camera_head/imu over USB-CDC micro-ROS,
// plus the NeoPixel ring/NeoKey renderer and the TMC2209 lens stepper.
// Joystick, buttons and XVS remain no-op stubs.

#include <Arduino.h>

#include "config.h"
#include "state.h"
#include "imu.h"
#include "microros.h"
#include "status_led.h"

#include "led.h"
#include "stepper.h"
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
      // Service /lens/command before draining, so a stop issued this cycle
      // reaches core 1 without waiting out an IMU backlog.
      microros_spin();
      microros_drain_and_publish();
      microros_publish_lens_state();
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
// Core 1 — peripherals (IMU, LEDs, stepper now; joystick/buttons later)
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
  // LED failure is cosmetic and stepper failure only costs us homing, so
  // neither is worth trapping core 1 the way a dead IMU is.
  if (!led_init()) {
    // No pixel buffer — carry on dark.
  }
  if (!stepper_init()) {
    // Driver didn't answer over UART. Stepping still works open-loop;
    // homing will abort to SYS_UNCALIBRATED rather than seek blind.
  }

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
