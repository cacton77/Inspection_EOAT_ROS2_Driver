#include "microros.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <ps_interfaces/msg/lens_command.h>
#include <ps_interfaces/msg/lens_state.h>

#include "config.h"
#include "state.h"

AgentState agent_state = AGENT_STATE_WAITING_AGENT;

static rcl_allocator_t       allocator;
static rclc_support_t        support;
static rcl_node_t            node;
static rcl_publisher_t       pub_imu;
static sensor_msgs__msg__Imu imu_msg;

static rclc_executor_t                  executor;
static rcl_subscription_t               sub_lens_cmd;
static rcl_publisher_t                  pub_lens_state;
static ps_interfaces__msg__LensCommand  lens_cmd_msg;
static ps_interfaces__msg__LensState    lens_state_msg;

// Executor handles = subscriptions + services + action servers. Publishers are
// not handles, so /lens/state and /camera_head/imu do not count here.
#define MICROROS_EXECUTOR_HANDLES 1

static bool entities_initialised = false;

// USB-CDC transport callbacks. micro_ros_arduino.h forward-declares these
// four symbols at file scope with extern "C" linkage and expects platforms
// without a built-in transport (rp2040 is one of them) to provide bodies.
// set_microros_transports() then wires them into rmw_uros_set_custom_transport.
extern "C" bool arduino_transport_open(struct uxrCustomTransport*) {
  return true;   // Serial is opened by Serial.begin() in setup()
}
extern "C" bool arduino_transport_close(struct uxrCustomTransport*) {
  return true;
}
extern "C" size_t arduino_transport_write(struct uxrCustomTransport*,
                                          const uint8_t* buf, size_t len,
                                          uint8_t*) {
  return Serial.write(buf, len);
}
extern "C" size_t arduino_transport_read(struct uxrCustomTransport*,
                                         uint8_t* buf, size_t len, int timeout,
                                         uint8_t*) {
  const uint32_t deadline = millis() + (uint32_t)timeout;
  size_t got = 0;
  while (got < len) {
    if (Serial.available()) {
      buf[got++] = (uint8_t)Serial.read();
    } else if ((int32_t)(millis() - deadline) >= 0) {
      break;
    }
  }
  return got;
}

// SystemMode is the firmware's own state machine; LensState.status is the
// contract with the host and does not have the same ordering. Map explicitly
// (architecture doc §3.8) rather than casting one to the other.
//
// STALLED wins over everything: it is a latched sub-state of UNCALIBRATED that
// lets the host tell "never homed" apart from "calibration was invalidated by
// hitting something", and apply different policy to each.
static uint8_t lens_status_for(SystemMode mode, bool stalled) {
  if (stalled) return ps_interfaces__msg__LensState__STATUS_STALLED;
  switch (mode) {
    case SYS_UNCALIBRATED: return ps_interfaces__msg__LensState__STATUS_UNCALIBRATED;
    case SYS_HOMING:       return ps_interfaces__msg__LensState__STATUS_HOMING;
    case SYS_LENS_MOVING:  return ps_interfaces__msg__LensState__STATUS_MOVING;
    case SYS_IDLE:         return ps_interfaces__msg__LensState__STATUS_IDLE;
    // The lens is held stationary during a capture; that is a capture state,
    // not a lens state, so the host sees IDLE.
    case SYS_PS_CAPTURING: return ps_interfaces__msg__LensState__STATUS_IDLE;
  }
  return ps_interfaces__msg__LensState__STATUS_UNCALIBRATED;
}

// Runs on core 0 inside the executor. Only ever writes shared state — the step
// pacing that acts on it lives in stepper_tick() on core 1.
static void cb_lens_cmd(const void* msgin) {
  const ps_interfaces__msg__LensCommand* m =
      (const ps_interfaces__msg__LensCommand*)msgin;

  mutex_enter_blocking(&state_mutex);

  // Priority rules: a capture sequence and a homing sweep each own the lens
  // outright, so nothing else may steer it.
  //
  // An uncalibrated lens is deliberately NOT rejected here for velocity jogs.
  // README §3 rejects everything while uncalibrated, but that leaves no way to
  // move the lens off an obstruction after a failed home, and no way to do the
  // open-loop bring-up that StallGuard tuning depends on. Position seeks are
  // still refused below, since a normalised target is meaningless without a
  // recorded range.
  if (state.mode == SYS_PS_CAPTURING || state.mode == SYS_HOMING) {
    mutex_exit(&state_mutex);
    return;
  }

  // Manual range calibration (MODE_SET_MIN = 2, MODE_SET_MAX = 3). Compared
  // against literals rather than the generated constants on purpose: the
  // committed libmicroros.a predates them, and since a .msg constant changes
  // no field and no wire format, rebuilding that archive just to import two
  // names would cost a full cross-compile and buy nothing. The host-side
  // ps_interfaces build does carry the named constants -- see LensCommand.msg.
  //
  // Only recorded here; stepper_tick() on core 1 performs it.
  if (m->mode == 2 || m->mode == 3) {
    state.lens_cal_request  = (m->mode == 2) ? LENS_CAL_SET_MIN : LENS_CAL_SET_MAX;
    state.lens_velocity_cmd = 0.0f;
    mutex_exit(&state_mutex);
    return;
  }

  if (m->mode == ps_interfaces__msg__LensCommand__MODE_POSITION) {
    const int32_t lo = state.lens_steps_min;
    const int32_t hi = state.lens_steps_max;
    if (hi <= lo) {            // no recorded range: nothing to seek against
      mutex_exit(&state_mutex);
      return;
    }
    float t = m->value;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const int32_t target = lo + (int32_t)lroundf(t * (float)(hi - lo));

    state.lens_cmd_mode        = LENS_CMD_POSITION;
    state.lens_position_target = t;
    state.lens_target_steps    = target;
    state.lens_velocity_cmd    = (target > state.lens_steps)
                                   ? (float)LENS_DEFAULT_VELOCITY
                                   : -(float)LENS_DEFAULT_VELOCITY;
    state.mode = (target == state.lens_steps) ? SYS_IDLE : SYS_LENS_MOVING;
  } else {
    state.lens_cmd_mode     = LENS_CMD_VELOCITY;
    state.lens_velocity_cmd = m->value;
    state.last_vel_cmd_us   = time_us_64();
    // Threshold matches stepper_tick()'s idle cut-off, so "too slow to step"
    // and "not moving" cannot disagree.
    if (fabsf(m->value) >= 1.0f) {
      state.mode = SYS_LENS_MOVING;
    } else if (state.mode == SYS_LENS_MOVING) {
      // Same calibration recovery as stepper_tick()'s stop_and_idle().
      state.mode = (state.lens_steps_max > state.lens_steps_min)
                     ? SYS_IDLE : SYS_UNCALIBRATED;
    }
  }

  mutex_exit(&state_mutex);
}

void microros_init_transport() {
  set_microros_transports();
}

bool microros_ping(uint32_t timeout_ms) {
  return rmw_uros_ping_agent(timeout_ms, 1) == RMW_RET_OK;
}

bool microros_create_entities() {
  // frame_id is constant; point the rosidl string at our literal and never
  // free it. Capacity = size + 1 so the receiver sees a NUL-terminated buffer.
  imu_msg.header.frame_id.data     = (char*)IMU_FRAME_ID;
  imu_msg.header.frame_id.size     = strlen(IMU_FRAME_ID);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // ROS convention: -1 in [0] means "unknown / not provided".
  imu_msg.orientation_covariance[0]         = -1.0;
  imu_msg.angular_velocity_covariance[0]    = 0.0;
  imu_msg.linear_acceleration_covariance[0] = 0.0;

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
    return false;
  }
  if (rcl_init_options_set_domain_id(&init_options, MICROROS_DOMAIN_ID) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_init_options_fini(&init_options); (void)rc;
    return false;
  }
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_init_options_fini(&init_options); (void)rc;
    return false;
  }
  rcl_ret_t rc_init = rcl_init_options_fini(&init_options); (void)rc_init;

  if (rclc_node_init_default(&node, MICROROS_NODE_NAME, MICROROS_NAMESPACE,
                             &support) != RCL_RET_OK) {
    rclc_support_fini(&support);
    return false;
  }
  rcl_ret_t rc;

  if (rclc_publisher_init_best_effort(
          &pub_imu, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          IMU_TOPIC) != RCL_RET_OK) {
    rc = rcl_node_fini(&node); (void)rc;
    rclc_support_fini(&support);
    return false;
  }

  // Best effort: /lens/state is a 20 Hz heartbeat of current truth. A dropped
  // sample is replaced 50 ms later, so reliable delivery would only add
  // retransmit latency over a 115200-baud link shared with the IMU stream.
  if (rclc_publisher_init_best_effort(
          &pub_lens_state, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LensState),
          LENS_STATE_TOPIC) != RCL_RET_OK) {
    rc = rcl_publisher_fini(&pub_imu, &node); (void)rc;
    rc = rcl_node_fini(&node);               (void)rc;
    rclc_support_fini(&support);
    return false;
  }

  // Reliable: dropping a jog command leaves the lens running on the previous
  // one until the deadman expires, and dropping a stop is worse still.
  if (rclc_subscription_init_default(
          &sub_lens_cmd, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LensCommand),
          LENS_COMMAND_TOPIC) != RCL_RET_OK) {
    rc = rcl_publisher_fini(&pub_lens_state, &node); (void)rc;
    rc = rcl_publisher_fini(&pub_imu, &node);        (void)rc;
    rc = rcl_node_fini(&node);                       (void)rc;
    rclc_support_fini(&support);
    return false;
  }

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context,
                         MICROROS_EXECUTOR_HANDLES, &allocator) != RCL_RET_OK ||
      rclc_executor_add_subscription(&executor, &sub_lens_cmd, &lens_cmd_msg,
                                     &cb_lens_cmd, ON_NEW_DATA) != RCL_RET_OK) {
    rc = rclc_executor_fini(&executor);              (void)rc;
    rc = rcl_subscription_fini(&sub_lens_cmd, &node);(void)rc;
    rc = rcl_publisher_fini(&pub_lens_state, &node); (void)rc;
    rc = rcl_publisher_fini(&pub_imu, &node);        (void)rc;
    rc = rcl_node_fini(&node);                       (void)rc;
    rclc_support_fini(&support);
    return false;
  }

  entities_initialised = true;
  return true;
}

void microros_destroy_entities() {
  if (!entities_initialised) return;

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // Reverse creation order.
  rcl_ret_t rc;
  rc = rclc_executor_fini(&executor);               (void)rc;
  rc = rcl_subscription_fini(&sub_lens_cmd, &node); (void)rc;
  rc = rcl_publisher_fini(&pub_lens_state, &node);  (void)rc;
  rc = rcl_publisher_fini(&pub_imu, &node);         (void)rc;
  rc = rcl_node_fini(&node);                        (void)rc;
  rclc_support_fini(&support);

  entities_initialised = false;
}

void microros_spin() {
  if (!entities_initialised) return;
  // Zero timeout: never block the agent ping or the IMU drain waiting on a
  // command that may not be coming.
  (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
}

void microros_publish_lens_state() {
  if (!entities_initialised) return;

  static uint64_t next_us = 0;
  const uint64_t now_us = time_us_64();
  if (now_us < next_us) return;
  next_us = now_us + (1000000ULL / LENS_STATE_RATE_HZ);

  mutex_enter_blocking(&state_mutex);
  const SystemMode mode       = state.mode;
  const bool       stalled    = state.stall_latched;
  const float      pos        = state.lens_position_norm;
  const float      vel        = state.lens_velocity_cmd;
  const int32_t    steps      = state.lens_steps;
  const uint16_t   sg         = state.sg_result;
  const bool       drv_ok     = state.driver_ok;
  const bool       drv_en     = state.driver_enabled;
  const bool       calibrated = state.lens_steps_max > state.lens_steps_min;
  mutex_exit(&state_mutex);

  lens_state_msg.stamp.sec     = (int32_t)(now_us / 1000000ULL);
  lens_state_msg.stamp.nanosec = (uint32_t)((now_us % 1000000ULL) * 1000ULL);
  // NaN rather than 0.0 while uncalibrated: 0.0 is a legitimate position and
  // would read as "fully retracted" instead of "unknown".
  lens_state_msg.position_norm = calibrated ? pos : NAN;
  lens_state_msg.velocity      = vel;
  lens_state_msg.status        = lens_status_for(mode, stalled);
  lens_state_msg.steps         = steps;
  lens_state_msg.sg_result     = sg;
  lens_state_msg.driver_ok     = drv_ok;
  lens_state_msg.driver_enabled = drv_en;

  rcl_ret_t rc = rcl_publish(&pub_lens_state, &lens_state_msg, NULL); (void)rc;
}

void microros_drain_and_publish() {
  if (!entities_initialised) return;

  ImuSample sample;
  while (queue_try_remove(&imu_queue, &sample)) {
    imu_msg.header.stamp.sec     = (int32_t)(sample.timestamp_us / 1000000ULL);
    imu_msg.header.stamp.nanosec = (uint32_t)((sample.timestamp_us % 1000000ULL) * 1000ULL);

    imu_msg.angular_velocity.x    = sample.omega_C[0];
    imu_msg.angular_velocity.y    = sample.omega_C[1];
    imu_msg.angular_velocity.z    = sample.omega_C[2];
    imu_msg.linear_acceleration.x = sample.a_cam_origin[0];
    imu_msg.linear_acceleration.y = sample.a_cam_origin[1];
    imu_msg.linear_acceleration.z = sample.a_cam_origin[2];

    rcl_ret_t rc = rcl_publish(&pub_imu, &imu_msg, NULL); (void)rc;
  }
}
