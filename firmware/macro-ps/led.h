#pragma once

// External NeoPixel chain — 3 NeoKeys followed by the 72-pixel illumination
// ring (see config.h for the index map).
//
//   led_init() — call once from setup1() (core 1). Builds the polar lookup
//                table and brings the strip up dark. Returns false if the
//                pixel buffer couldn't be allocated.
//   led_tick() — call every loop1() iteration. Self-rate-limits to
//                LED_TICK_HZ, renders the frame for the current SystemMode,
//                and only calls strip.show() when the frame actually changed.
//
// The onboard status NeoPixel is NOT on this chain — it lives on its own
// Adafruit_NeoPixel instance in status_led.cpp, on a different GPIO and
// therefore its own PIO state machine.

bool led_init();
void led_tick();
