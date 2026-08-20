#pragma once

// Joystick ADC sampling + 2D Gaussian illumination mapping. Stub — real
// implementation pending. Will own: analogRead on JOYSTICK_X/Y_PIN
// (A0 = GP29, A1 = GP28) at 1 kHz, deadzone, and driving the joystick
// override that led.cpp already renders via render_ring_gaussian().
inline void joystick_init() {}
inline void joystick_tick() {}
