#pragma once

// XVS camera-sync interrupt. Stub — real implementation pending.
// Lives on core 0 (init from setup() so the ISR runs alongside the
// micro-ROS executor). Will set xvs_edge + xvs_timestamp_us on RISING.
inline void xvs_init() {}
