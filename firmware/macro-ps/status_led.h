#pragma once

// Onboard status NeoPixel — agent_state visualisation.
// Lives on core 1 (peripheral I/O). Reads agent_state via a single-byte
// load; no mutex needed since core 0 is the sole writer.

void status_led_init();
void status_led_tick();

// Override the agent-state colour with a distinct fault pattern. Used by
// peripheral inits (e.g. imu_init) to make a stuck core 1 visually
// unambiguous instead of inheriting the agent-state colour from the trap
// loop.
void status_led_signal_imu_fault();
