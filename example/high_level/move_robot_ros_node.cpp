// move_robot_ros_node.cpp
//
// Pure ROS2 process — no Booster SDK headers, no rclcpp-incompatible DDS.
// Subscribes to /planning/trajectory and forwards each trajectory as a single
// UDP datagram to move_robot_bridge.
//
// Usage:
//   ./move_robot_ros_node [--bridge_host <ip>] [--port <udp_port>]
// The bridge host/port default to 127.0.0.1:9870.
//
// Joint ordering expected in trajectory.points[i].positions (10 values):
//   [0]    waist
//   [1..7] right arm  (shoulder_pitch, shoulder_roll, elbow_pitch, elbow_yaw,
//                       wrist_pitch, wrist_yaw, hand_roll)
//   [8]    head yaw
//   [9]    head pitch

#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// POSIX sockets — no Booster types included in this translation unit
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "move_robot_ipc.h"

class MoveRobotRosNode final : public rclcpp::Node {
public:
    MoveRobotRosNode(const std::string &bridge_host, uint16_t port)
        : Node("move_robot_ros_node") {
        // ── UDP socket ────────────────────────────────────────────────────
        udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd_ < 0) {
            throw std::runtime_error("socket() failed");
        }
        std::memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_port   = htons(port);
        if (::inet_pton(AF_INET, bridge_host.c_str(), &dest_.sin_addr) <= 0) {
            ::close(udp_fd_);
            throw std::runtime_error("Invalid bridge host: " + bridge_host);
        }

        // ── ROS subscription ──────────────────────────────────────────────
        sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/planning/trajectory", 10,
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                this->OnTrajectory(msg);
            });

        RCLCPP_INFO(get_logger(),
                    "move_robot_ros_node ready — forwarding to %s:%u",
                    bridge_host.c_str(), static_cast<unsigned>(port));
    }

    ~MoveRobotRosNode() override {
        if (udp_fd_ >= 0) {
            ::close(udp_fd_);
        }
    }

private:
    int            udp_fd_ = -1;
    sockaddr_in    dest_{};
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;

    void OnTrajectory(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr &msg) {
        if (msg->points.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty trajectory, ignoring");
            return;
        }

        const size_t total = msg->points.size();
        size_t offset = 0;
        bool first_chunk = true;

        while (offset < total) {
            const size_t chunk_size =
                std::min(static_cast<size_t>(MOVE_ROBOT_IPC_MAX_POINTS),
                         total - offset);

            WaypointPacket pkt{};
            pkt.num_points      = static_cast<uint32_t>(chunk_size);
            pkt.is_continuation = first_chunk ? 0u : 1u;

            for (uint32_t i = 0; i < pkt.num_points; ++i) {
                const auto &pt    = msg->points[offset + i];
                const size_t n_pos = pt.positions.size();
                const size_t n_vel = pt.velocities.size();

                for (int d = 0; d < MOVE_ROBOT_IPC_DOF; ++d) {
                    pkt.positions[i][d] =
                        (d < static_cast<int>(n_pos))
                            ? static_cast<float>(pt.positions[d])
                            : 0.0f;
                    pkt.velocities[i][d] =
                        (d < static_cast<int>(n_vel))
                            ? static_cast<float>(pt.velocities[d])
                            : 0.0f;
                }
            }

            const ssize_t sent = ::sendto(
                udp_fd_, &pkt, sizeof(pkt), 0,
                reinterpret_cast<const sockaddr *>(&dest_), sizeof(dest_));
            if (sent < 0) {
                RCLCPP_ERROR(get_logger(), "sendto() failed at chunk offset %zu",
                             offset);
                return;
            }

            offset      += chunk_size;
            first_chunk  = false;
        }

        RCLCPP_INFO(get_logger(),
                    "Forwarded trajectory with %zu waypoints (%zu chunk(s)) to bridge",
                    total,
                    (total + MOVE_ROBOT_IPC_MAX_POINTS - 1) / MOVE_ROBOT_IPC_MAX_POINTS);
    }
};

// ── Argument helpers ──────────────────────────────────────────────────────────
static bool GetArg(int argc, char **argv, const std::string &key,
                   std::string &out) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (key == argv[i]) { out = argv[i + 1]; return true; }
    }
    return false;
}

int main(int argc, char **argv) {
    // Parse our own arguments before handing off to rclcpp
    std::string bridge_host = "127.0.0.1";
    std::string port_str;
    uint16_t    port = MOVE_ROBOT_IPC_PORT;

    GetArg(argc, argv, "--bridge_host", bridge_host);
    if (GetArg(argc, argv, "--port", port_str)) {
        try { port = static_cast<uint16_t>(std::stoul(port_str)); }
        catch (...) {}
    }

    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MoveRobotRosNode>(bridge_host, port);
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
