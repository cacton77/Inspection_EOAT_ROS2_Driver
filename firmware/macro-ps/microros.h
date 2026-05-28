#pragma once

#include <stdint.h>

// micro-ROS lifecycle and IMU publishing.
//
// Transport: USB CDC (Serial), wired up via rmw_uros_set_custom_transport
// because the precompiled humble micro_ros_arduino doesn't ship an RP2040
// transport (architectures list in library.properties excludes rp2040).
//
// Agent invocation on the Pi:
//   ros2 run micro_ros_agent micro_ros_agent serial \
//        --dev /dev/ttyACM0 -b 115200
//
// State machine:
//   WAITING_AGENT     — pinging until the agent answers
//   AGENT_AVAILABLE   — agent reachable; create entities
//   AGENT_CONNECTED   — entities live; drain IMU queue + publish
//   AGENT_DISCONNECTED— ping failed; tear down and reset transient state

enum AgentState {
  AGENT_STATE_WAITING_AGENT,
  AGENT_STATE_AVAILABLE,
  AGENT_STATE_CONNECTED,
  AGENT_STATE_DISCONNECTED,
};

extern AgentState agent_state;

void microros_init_transport();
bool microros_create_entities();
void microros_destroy_entities();
void microros_drain_and_publish();
bool microros_ping(uint32_t timeout_ms);
