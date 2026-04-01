#pragma once

// Shared IPC protocol between move_robot_ros_node (ROS2) and move_robot_bridge (Booster SDK).
// Communication is via a single UDP datagram sent to 127.0.0.1 on MOVE_ROBOT_IPC_PORT.
//
// Joint ordering within each waypoint (10 values):
//   [0]    waist
//   [1..7] right arm  (shoulder_pitch, shoulder_roll, elbow_pitch, elbow_yaw,
//                       wrist_pitch, wrist_yaw, hand_roll)
//   [8]    head yaw
//   [9]    head pitch

#include <cstdint>

static constexpr uint16_t MOVE_ROBOT_IPC_PORT = 9870;
static constexpr int      MOVE_ROBOT_IPC_MAX_POINTS = 64;
static constexpr int      MOVE_ROBOT_IPC_DOF = 10;

#pragma pack(push, 1)
struct WaypointPacket {
    uint32_t num_points;      // number of waypoints in this packet (0 = hold)
    uint32_t is_continuation; // 0 = start of new trajectory (replaces any pending)
                              // 1 = continuation chunk (appended to current buffer)
    float    positions [MOVE_ROBOT_IPC_MAX_POINTS][MOVE_ROBOT_IPC_DOF];
    float    velocities[MOVE_ROBOT_IPC_MAX_POINTS][MOVE_ROBOT_IPC_DOF];
};
#pragma pack(pop)

static_assert(sizeof(WaypointPacket) <= 65507,
              "WaypointPacket exceeds UDP payload limit");
