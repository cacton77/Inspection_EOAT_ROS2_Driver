#pragma once

// TMC2209 stepper driver + step pulse generation. Stub — real
// implementation pending. Will own: TMCStepper UART setup, hardware
// timer alarms for STEP, soft-limit enforcement, position tracking,
// homing FSM driven by SG_RESULT.
inline void stepper_init() {}
inline void stepper_tick() {}
