// move_robot_bridge.cpp
//
// Pure Booster SDK process — no ROS headers, no rclcpp.
// Receives joint-trajectory packets from move_robot_ros_node via UDP and
// forwards them to the robot as Booster LowCmd messages.
//
// Usage:
//   ./move_robot_bridge --network_interface <ip_or_iface> [--port <udp_port>]
//                       [--control_freq <Hz>] [--kp <float>] [--kd <float>]
//                       [--init_time <sec>]

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

// POSIX sockets
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// Booster SDK — no ROS types appear below this line
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>

#include "move_robot_ipc.h"

// ── Constants ─────────────────────────────────────────────────────────────────
static constexpr int kHeadYawIndex   = 0;
static constexpr int kHeadPitchIndex = 1;
static constexpr int kWaistIndex     = 16;
static constexpr int kTotalMotors    =
    static_cast<int>(booster::robot::b1::kJointCnt7DofArm);

static const std::array<int, 7> kLeftArmIndices  = {2, 3, 4, 5, 6, 7, 8};
static const std::array<int, 7> kRightArmIndices = {9, 10, 11, 12, 13, 14, 15};
static const std::array<int, 2> kRightWristPitchYaw = {13, 14};
static constexpr int kRightWristRoll = 15;

// ── Options ───────────────────────────────────────────────────────────────────
struct Options {
    std::string network_interface;
    uint16_t    port          = MOVE_ROBOT_IPC_PORT;
    double      control_freq  = 50.0;
    float       kp            = 150.0f;
    float       kp_right_wrist      = 120.0f;
    float       kp_right_wrist_roll = 100.0f;
    float       kd            = 1.2f;
    float       init_time     = 5.0f;
    float       head_yaw_min  = -1.2f;
    float       head_yaw_max  =  1.2f;
    float       head_pitch_min= -0.8f;
    float       head_pitch_max=  0.8f;
};

static float Clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

// ── Bridge ────────────────────────────────────────────────────────────────────
class Bridge {
public:
    explicit Bridge(const Options &opts)
        : opts_(opts),
          control_dt_(1.0 / opts.control_freq),
          state_subscriber_(booster::robot::b1::kTopicLowState) {}

    void Run() {
        // 1. Init Booster DDS
        booster::robot::ChannelFactory::Instance()->Init(0, opts_.network_interface);

        publisher_.reset(
            new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
                booster::robot::b1::kTopicJointCtrl));
        publisher_->InitChannel();

        loco_client_.Init();

        state_subscriber_.InitChannel(
            [this](const void *msg) { this->OnLowState(msg); });

        // Build per-joint kp vector
        kp_per_joint_.assign(kTotalMotors, opts_.kp);
        for (int idx : kRightWristPitchYaw) {
            kp_per_joint_.at(idx) = opts_.kp_right_wrist;
        }
        kp_per_joint_.at(kRightWristRoll) = opts_.kp_right_wrist_roll;

        // Pre-populate motor_cmd vector
        for (int i = 0; i < kTotalMotors; ++i) {
            booster_interface::msg::MotorCmd cmd;
            low_cmd_msg_.motor_cmd().push_back(cmd);
        }

        // 2. Move to retract pose
        InitializeRobotToRetract();

        // 3. Start UDP listener thread
        udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd_ < 0) {
            throw std::runtime_error("socket() failed");
        }
        int reuse = 1;
        ::setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port        = htons(opts_.port);
        if (::bind(udp_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            ::close(udp_fd_);
            throw std::runtime_error("bind() failed on port " + std::to_string(opts_.port));
        }
        std::cout << "[bridge] Listening for trajectory packets on 127.0.0.1:"
                  << opts_.port << std::endl;

        udp_thread_ = std::thread(&Bridge::UdpListener, this);

        // 4. 50 Hz control loop
        auto next = std::chrono::steady_clock::now();
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_dt_));

        while (running_) {
            ControlTick();
            next += period;
            std::this_thread::sleep_until(next);
        }

        udp_thread_.join();
        ::close(udp_fd_);
    }

    void Stop() { running_ = false; }

private:
    Options opts_;
    double  control_dt_;

    // Booster objects
    booster::robot::b1::B1LocoClient loco_client_;
    booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> publisher_;
    booster::robot::ChannelSubscriber<booster_interface::msg::LowState> state_subscriber_;
    booster_interface::msg::LowCmd low_cmd_msg_;

    // Per-joint Kp
    std::vector<float> kp_per_joint_;

    // Robot state feedback
    std::vector<float> current_joint_states_ = std::vector<float>(kTotalMotors, 0.0f);
    bool  state_received_ = false;
    std::mutex state_mutex_;

    // Arm init / hold positions
    std::array<float, 7> left_arm_init_  = {0.5f, -1.0f, 0.0f, -1.4f, 0.0f, 0.0f, 0.0f};
    std::array<float, 7> right_arm_init_ = {0.5f,  1.0f, 0.0f,  1.4f, 0.0f, 0.0f, 0.0f};
    float waist_init_       = 0.0f;
    float head_yaw_init_    = 0.0f;
    float head_pitch_init_  = 0.4f;

    std::array<float, 7> current_right_arm_pos_ = right_arm_init_;
    float current_waist_     = waist_init_;
    float current_head_yaw_  = head_yaw_init_;
    float current_head_pitch_= head_pitch_init_;

    float weight_      = 0.0f;
    float weight_rate_ = 0.2f;
    float tau_ff_      = 0.0f;

    // Trajectory state (written by UDP thread, read by control thread)
    struct TrajStore {
        std::vector<std::array<float, MOVE_ROBOT_IPC_DOF>> positions;
        std::vector<std::array<float, MOVE_ROBOT_IPC_DOF>> velocities;
        size_t idx       = 0;
        bool   executing = false;
    };
    TrajStore  traj_;
    std::mutex traj_mutex_;

    // UDP
    int         udp_fd_     = -1;
    std::thread udp_thread_;
    std::atomic<bool> running_{true};

    // ── LowState callback ──────────────────────────────────────────────────
    void OnLowState(const void *raw) {
        const auto *ls = static_cast<const booster_interface::msg::LowState *>(raw);
        const auto &ms = ls->motor_state_serial();
        if (static_cast<int>(ms.size()) < kTotalMotors) return;
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_received_ = true;
        for (int i = 0; i < kTotalMotors; ++i) {
            current_joint_states_[i] = ms[i].q();
        }
    }

    // ── Initialization ────────────────────────────────────────────────────
    void InitializeRobotToRetract() {
        std::cout << "[bridge] Changing mode to CUSTOM ..." << std::endl;
        loco_client_.ChangeMode(booster::robot::RobotMode::kCustom);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Wait for first LowState
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (true) {
            {
                std::lock_guard<std::mutex> lk(state_mutex_);
                if (state_received_) break;
            }
            if (std::chrono::steady_clock::now() > deadline) {
                throw std::runtime_error("[bridge] Timed out waiting for LowState");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::vector<float> start_joints;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            start_joints = current_joint_states_;
        }

        std::array<float, 7> la_start{}, ra_start{};
        for (size_t i = 0; i < kLeftArmIndices.size(); ++i) {
            la_start[i] = start_joints[kLeftArmIndices[i]];
            ra_start[i] = start_joints[kRightArmIndices[i]];
        }
        float waist_start      = start_joints[kWaistIndex];
        float head_yaw_start   = start_joints[kHeadYawIndex];
        float head_pitch_start = start_joints[kHeadPitchIndex];

        const int steps = std::max(1, static_cast<int>(opts_.init_time / control_dt_));
        const float wm  = weight_rate_ * static_cast<float>(control_dt_);

        for (int i = 0; i < steps; ++i) {
            const float a = static_cast<float>(i + 1) / static_cast<float>(steps);
            weight_ = Clamp(weight_ + wm, 0.0f, 0.5f);

            for (size_t j = 0; j < kLeftArmIndices.size(); ++j) {
                SetMotorCmd(kLeftArmIndices[j],  la_start[j] + (left_arm_init_[j]  - la_start[j]) * a, 0.0f, weight_);
                SetMotorCmd(kRightArmIndices[j], ra_start[j] + (right_arm_init_[j] - ra_start[j]) * a, 0.0f, weight_);
            }
            SetMotorCmd(kWaistIndex,     waist_start      + (waist_init_       - waist_start)      * a, 0.0f, weight_);
            SetMotorCmd(kHeadYawIndex,   head_yaw_start   + (head_yaw_init_    - head_yaw_start)   * a, 0.0f, weight_);
            SetMotorCmd(kHeadPitchIndex, head_pitch_start + (head_pitch_init_  - head_pitch_start) * a, 0.0f, weight_);

            PublishLowCmd();
            std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
        }

        current_right_arm_pos_ = right_arm_init_;
        current_waist_         = waist_init_;
        current_head_yaw_      = head_yaw_init_;
        current_head_pitch_    = head_pitch_init_;
        std::cout << "[bridge] Initialization complete. Waiting for trajectories..." << std::endl;
    }

    // ── UDP listener thread ────────────────────────────────────────────────
    void UdpListener() {
        WaypointPacket pkt{};
        while (running_) {
            ssize_t n = ::recv(udp_fd_, &pkt, sizeof(pkt), 0);
            if (n < 0) break;  // socket closed
            if (n < static_cast<ssize_t>(sizeof(uint32_t) * 2)) continue;

            if (pkt.num_points == 0) continue;  // hold — ignore

            const uint32_t chunk = std::min(
                pkt.num_points,
                static_cast<uint32_t>(MOVE_ROBOT_IPC_MAX_POINTS));

            std::lock_guard<std::mutex> lk(traj_mutex_);
            if (pkt.is_continuation == 0) {
                // New trajectory — reset the buffer
                traj_.positions.clear();
                traj_.velocities.clear();
                traj_.idx       = 0;
                traj_.executing = false;
            }

            for (uint32_t i = 0; i < chunk; ++i) {
                std::array<float, MOVE_ROBOT_IPC_DOF> pos{}, vel{};
                for (int d = 0; d < MOVE_ROBOT_IPC_DOF; ++d) {
                    pos[d] = pkt.positions[i][d];
                    vel[d] = pkt.velocities[i][d];
                }
                traj_.positions.push_back(pos);
                traj_.velocities.push_back(vel);
            }

            if (!traj_.executing) {
                traj_.executing = true;
            }

            std::cout << "[bridge] Buffer now has "
                      << traj_.positions.size() << " waypoints"
                      << (pkt.is_continuation ? " (continuation)" : " (new)")
                      << std::endl;
        }
    }

    // ── Control tick (called at control_freq) ──────────────────────────────
    void ControlTick() {
        bool executing = false;
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            if (traj_.executing) {
                if (traj_.idx >= traj_.positions.size()) {
                    traj_.executing = false;
                    std::cout << "[bridge] Trajectory complete" << std::endl;
                } else {
                    executing = true;
                }
            }
        }

        if (executing) {
            ExecuteOneWaypoint();
        } else {
            SendHoldCommand();
        }
    }

    void ExecuteOneWaypoint() {
        size_t idx;
        std::array<float, MOVE_ROBOT_IPC_DOF> pos{}, vel{};
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            if (traj_.idx >= traj_.positions.size()) return;
            idx = traj_.idx++;
            pos = traj_.positions[idx];
            vel = traj_.velocities[idx];
        }

        // Decode: [0]=waist, [1..7]=right_arm x7, [8]=head_yaw, [9]=head_pitch
        float waist_q      = pos[0], waist_dq = vel[0];
        float head_yaw_q   = Clamp(pos[8], opts_.head_yaw_min,   opts_.head_yaw_max);
        float head_pitch_q = Clamp(pos[9], opts_.head_pitch_min, opts_.head_pitch_max);

        std::array<float, 7> ra_q{}, ra_dq{};
        for (int i = 0; i < 7; ++i) {
            ra_q[i]  = pos[i + 1];
            ra_dq[i] = vel[i + 1];
        }

        for (size_t j = 0; j < kLeftArmIndices.size(); ++j) {
            SetMotorCmd(kLeftArmIndices[j], left_arm_init_[j], 0.0f, weight_);
        }
        for (size_t j = 0; j < kRightArmIndices.size(); ++j) {
            SetMotorCmd(kRightArmIndices[j], ra_q[j], ra_dq[j], weight_);
        }
        SetMotorCmd(kWaistIndex,     waist_q,      waist_dq, weight_);
        SetMotorCmd(kHeadYawIndex,   head_yaw_q,   vel[8],   weight_);
        SetMotorCmd(kHeadPitchIndex, head_pitch_q, vel[9],   weight_);
        PublishLowCmd();

        current_right_arm_pos_ = ra_q;
        current_waist_         = waist_q;
        current_head_yaw_      = head_yaw_q;
        current_head_pitch_    = head_pitch_q;
    }

    void SendHoldCommand() {
        for (size_t j = 0; j < kLeftArmIndices.size(); ++j) {
            SetMotorCmd(kLeftArmIndices[j], left_arm_init_[j], 0.0f, weight_);
        }
        for (size_t j = 0; j < kRightArmIndices.size(); ++j) {
            SetMotorCmd(kRightArmIndices[j], current_right_arm_pos_[j], 0.0f, weight_);
        }
        SetMotorCmd(kWaistIndex,     current_waist_,      0.0f, weight_);
        SetMotorCmd(kHeadYawIndex,   current_head_yaw_,   0.0f, weight_);
        SetMotorCmd(kHeadPitchIndex, current_head_pitch_,  0.0f, weight_);
        PublishLowCmd();
    }

    void SetMotorCmd(int idx, float q, float dq, float weight) {
        auto &cmd = low_cmd_msg_.motor_cmd().at(static_cast<size_t>(idx));
        cmd.q(q);
        cmd.dq(dq);
        cmd.kp(kp_per_joint_.at(static_cast<size_t>(idx)));
        cmd.kd(opts_.kd);
        cmd.tau(tau_ff_);
        cmd.weight(weight);
    }

    void PublishLowCmd() {
        publisher_->Write(&low_cmd_msg_);
    }
};

// ── Argument parsing ──────────────────────────────────────────────────────────
static bool GetArg(int argc, char **argv, const std::string &key, std::string &out) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (key == argv[i]) { out = argv[i + 1]; return true; }
    }
    return false;
}

template <typename T>
static bool GetNumeric(int argc, char **argv, const std::string &key, T &out) {
    std::string s;
    if (!GetArg(argc, argv, key, s)) return false;
    try {
        if constexpr (std::is_same<T, float>::value)    out = std::stof(s);
        else if constexpr (std::is_same<T, double>::value) out = std::stod(s);
        else if constexpr (std::is_same<T, uint16_t>::value) out = static_cast<uint16_t>(std::stoul(s));
    } catch (...) { return false; }
    return true;
}

int main(int argc, char **argv) {
    Options opts;
    GetArg    (argc, argv, "--network_interface",   opts.network_interface);
    GetNumeric(argc, argv, "--port",                opts.port);
    GetNumeric(argc, argv, "--control_freq",        opts.control_freq);
    GetNumeric(argc, argv, "--kp",                  opts.kp);
    GetNumeric(argc, argv, "--kp_right_wrist",      opts.kp_right_wrist);
    GetNumeric(argc, argv, "--kp_right_wrist_roll", opts.kp_right_wrist_roll);
    GetNumeric(argc, argv, "--kd",                  opts.kd);
    GetNumeric(argc, argv, "--init_time",           opts.init_time);
    GetNumeric(argc, argv, "--head_yaw_min",        opts.head_yaw_min);
    GetNumeric(argc, argv, "--head_yaw_max",        opts.head_yaw_max);
    GetNumeric(argc, argv, "--head_pitch_min",      opts.head_pitch_min);
    GetNumeric(argc, argv, "--head_pitch_max",      opts.head_pitch_max);

    if (opts.network_interface.empty()) {
        std::cerr << "Usage: " << argv[0]
                  << " --network_interface <ip_or_iface> [options]\n"
                  << "  --port <udp_port>   (default " << MOVE_ROBOT_IPC_PORT << ")\n"
                  << "  --control_freq <Hz> --kp <float> --kd <float>\n"
                  << "  --init_time <sec>\n";
        return 1;
    }

    Bridge bridge(opts);
    bridge.Run();
    return 0;
}
