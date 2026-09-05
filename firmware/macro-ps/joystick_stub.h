#pragma once

// Joystick ADC sampling + 2D Gaussian illumination mapping. Stub — real
// implementation pending. Will own: analogRead on JOYSTICK_X/Y_PIN
// (A2 = GP28, A3 = GP29) at 1 kHz, deadzone, and driving the joystick
// override that led.cpp already renders via render_ring_gaussian().
inline void joystick_init() {}
inline void joystick_tick() {}
