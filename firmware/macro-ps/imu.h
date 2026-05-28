#pragma once

// LSM6DSOX driver:
//   imu_init() — call once from setup1() (core 1). Returns false if the
//                sensor isn't responding on I2C.
//   imu_tick() — call every loop1() iteration. Self-rate-limits to
//                IMU_RATE_HZ. Reads sensor, applies S->C transform,
//                computes lever-arm correction at the camera origin, and
//                pushes an ImuSample onto imu_queue (non-blocking).

bool imu_init();
void imu_tick();
