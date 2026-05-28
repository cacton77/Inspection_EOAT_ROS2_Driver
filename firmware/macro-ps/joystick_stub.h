#pragma once

// Joystick ADC sampling + 2D Gaussian illumination mapping. Stub —
// real implementation pending. Will own: analogRead on JOYSTICK_X/Y_PIN
// at 1 kHz, deadzone, polar mapping into led_table[].
inline void joystick_init() {}
inline void joystick_tick() {}
