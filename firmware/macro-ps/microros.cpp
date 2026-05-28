#include "microros.h"

#include <Arduino.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "state.h"

AgentState agent_state = AGENT_STATE_WAITING_AGENT;

static rcl_allocator_t       allocator;
static rclc_support_t        support;
static rcl_node_t            node;
static rcl_publisher_t       pub_imu;
static sensor_msgs__msg__Imu imu_msg;

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
  if (rclc_publisher_init_best_effort(
          &pub_imu, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          IMU_TOPIC) != RCL_RET_OK) {
    rcl_ret_t rc = rcl_node_fini(&node); (void)rc;
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

  rcl_ret_t rc;
  rc = rcl_publisher_fini(&pub_imu, &node); (void)rc;
  rc = rcl_node_fini(&node);                (void)rc;
  rclc_support_fini(&support);

  entities_initialised = false;
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
