#include "state.h"

mutex_t     state_mutex;
SystemState state = {};
queue_t     imu_queue;

void state_init() {
  mutex_init(&state_mutex);
  queue_init(&imu_queue, sizeof(ImuSample), IMU_QUEUE_DEPTH);

  state.mode               = SYS_UNCALIBRATED;
  state.homing_phase       = HOMING_NONE;
  state.lens_position_norm = 0.0f;
  state.lens_velocity_cmd  = 0.0f;
}
