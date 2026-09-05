#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#include "config.h"
#include "state.h"

static Adafruit_LSM6DSOX imu;

static const float R_CS[3][3] = {
  { R_CS_00, R_CS_01, R_CS_02 },
  { R_CS_10, R_CS_11, R_CS_12 },
  { R_CS_20, R_CS_21, R_CS_22 },
};
static const float t_CS[3] = { T_CS_X, T_CS_Y, T_CS_Z };

static float    omega_C_prev[3] = {0, 0, 0};
static float    alpha_C_prev[3] = {0, 0, 0};
static uint64_t last_imu_us = 0;

static inline void mat3_mul_vec(const float M[3][3], const float v[3], float out[3]) {
  out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
  out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
  out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}

static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

bool imu_init() {
  // Feather RP2040 routes the STEMMA QT connector to the default Wire
  // (GP2/GP3); the QT Py had it on Wire1 (GP22/GP23). Pin Wire to those GPIOs
  // explicitly so this doesn't silently come up on the wrong pads if the build
  // target is ever wrong.
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  if (!imu.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire)) {
    return false;
  }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_416_HZ);
  return true;
}

void imu_tick() {
  static uint64_t next_us = 0;
  const uint64_t now_us = time_us_64();
  if (now_us < next_us) return;
  next_us = now_us + (1000000ULL / IMU_RATE_HZ);

  sensors_event_t accel_evt, gyro_evt, temp_evt;
  imu.getEvent(&accel_evt, &gyro_evt, &temp_evt);

  const float accel_S[3] = {
    accel_evt.acceleration.x,
    accel_evt.acceleration.y,
    accel_evt.acceleration.z,
  };
  const float omega_S[3] = {
    gyro_evt.gyro.x,
    gyro_evt.gyro.y,
    gyro_evt.gyro.z,
  };

  float omega_C[3], accel_C[3];
  mat3_mul_vec(R_CS, omega_S, omega_C);
  mat3_mul_vec(R_CS, accel_S, accel_C);

  // Angular acceleration: finite difference + IIR smoothing
  float alpha_C[3] = {0, 0, 0};
  if (last_imu_us != 0) {
    const float dt = (now_us - last_imu_us) * 1e-6f;
    if (dt > 0.0f) {
      const float inv_dt = 1.0f / dt;
      for (int i = 0; i < 3; i++) {
        const float raw = (omega_C[i] - omega_C_prev[i]) * inv_dt;
        alpha_C[i] = IMU_ALPHA_BETA * raw
                   + (1.0f - IMU_ALPHA_BETA) * alpha_C_prev[i];
      }
    }
  }

  // Lever-arm correction at camera origin: a_cam = R*a_S + alpha x t + omega x (omega x t)
  float tangential[3], inner[3], centripetal[3];
  cross3(alpha_C, t_CS, tangential);
  cross3(omega_C, t_CS, inner);
  cross3(omega_C, inner, centripetal);

  ImuSample sample;
  for (int i = 0; i < 3; i++) {
    sample.omega_C[i]      = omega_C[i];
    sample.a_cam_origin[i] = accel_C[i] + tangential[i] + centripetal[i];
    omega_C_prev[i]        = omega_C[i];
    alpha_C_prev[i]        = alpha_C[i];
  }
  sample.timestamp_us = now_us;
  last_imu_us = now_us;

  // Drop silently if the queue is full (consumer is behind).
  queue_try_add(&imu_queue, &sample);
}
