#pragma once

// TMC2209 lens stepper — UART configuration, step pulse generation, soft
// limits, position tracking, and the StallGuard homing FSM.
//
//   stepper_init() — call once from setup1() (core 1). Brings up the TMC UART
//                    and the STEP/DIR/EN pins. Returns false if the driver
//                    doesn't answer over UART; stepping still works in that
//                    case, but homing can't (StallGuard is unreadable).
//   stepper_tick() — call every loop1() iteration. Non-blocking; paces its own
//                    step pulses off time_us_64().
//
// There is no command entry point: micro-ROS callbacks on core 0 write
// state.lens_velocity_cmd / state.mode / state.last_vel_cmd_us under
// state_mutex, and this module reads them on each tick.

bool stepper_init();
void stepper_tick();
