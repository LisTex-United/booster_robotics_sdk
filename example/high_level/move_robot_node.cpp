#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>

namespace {

struct Options {
  std::string network_interface;
  double control_freq = 50.0;
  float kp = 150.0f;
  float kp_right_wrist = 120.0f;
  float kp_right_wrist_roll = 100.0f;
  float kd = 1.2f;
  float init_time = 5.0f;
  float head_yaw_min = -1.2f;
  float head_yaw_max = 1.2f;
  float head_pitch_min = -0.8f;
  float head_pitch_max = 0.8f;
};

class BoosterExecutorNode final : public rclcpp::Node {
 public:
  explicit BoosterExecutorNode(const Options &options)
      : Node("booster_executor_cpp_node"),
        options_(options),
        control_dt_(1.0 / options.control_freq),
        state_subscriber_(booster::robot::b1::kTopicLowState) {
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/planning/trajectory", 10,
        std::bind(&BoosterExecutorNode::TrajectoryCallback, this,
                  std::placeholders::_1));

    InitSdk();
    InitializeRobotToRetract();

    using namespace std::chrono_literals;
    auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(control_dt_));
    control_timer_ = create_wall_timer(
        timer_period, std::bind(&BoosterExecutorNode::ControlLoop, this));

    RCLCPP_INFO(get_logger(),
                "Ready: initialized to retract pose, waiting for /planning/trajectory");
  }

 private:
  static constexpr int kHeadYawIndex = 0;
  static constexpr int kHeadPitchIndex = 1;
  static constexpr int kWaistIndex = 16;
  static constexpr int kTotalMotors =
      static_cast<int>(booster::robot::b1::kJointCnt7DofArm);

  const std::array<int, 7> left_arm_joint_indices_{2, 3, 4, 5, 6, 7, 8};
  const std::array<int, 7> right_arm_joint_indices_{9, 10, 11, 12, 13, 14, 15};
  const std::array<int, 2> right_wrist_pitch_yaw_indices_{13, 14};
  const int right_wrist_roll_index_ = 15;

  Options options_;
  double control_dt_;
  float tau_ff_ = 0.0f;
  float weight_ = 0.0f;
  float weight_rate_ = 0.2f;

  std::array<float, 7> left_arm_init_{0.5f, -1.0f, 0.0f, -1.4f, 0.0f, 0.0f,
                                      0.0f};
  std::array<float, 7> right_arm_init_{0.5f, 1.0f, 0.0f, 1.4f, 0.0f, 0.0f,
                                       0.0f};
  float waist_init_ = 0.0f;
  float head_yaw_init_ = 0.0f;
  float head_pitch_init_ = 0.4f;

  std::array<float, 7> current_right_arm_pos_ = right_arm_init_;
  float current_waist_pos_ = waist_init_;
  float current_head_yaw_ = head_yaw_init_;
  float current_head_pitch_ = head_pitch_init_;

  std::vector<float> kp_per_joint_;
  std::vector<float> current_joint_states_ = std::vector<float>(kTotalMotors, 0.0f);
  bool state_received_ = false;
  std::mutex state_mutex_;

  trajectory_msgs::msg::JointTrajectory::SharedPtr pending_trajectory_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr active_trajectory_;
  size_t active_waypoint_index_ = 0;
  bool is_executing_ = false;
  std::mutex trajectory_mutex_;

  booster::robot::b1::B1LocoClient loco_client_;
  booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> publisher_;
  booster::robot::ChannelSubscriber<booster_interface::msg::LowState> state_subscriber_;
  booster_interface::msg::LowCmd low_cmd_msg_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  static float Clamp(float value, float lower, float upper) {
    return std::max(lower, std::min(upper, value));
  }

  void InitSdk() {
    booster::robot::ChannelFactory::Instance()->Init(0,
                                                     options_.network_interface);

    publisher_.reset(
        new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
            booster::robot::b1::kTopicJointCtrl));
    publisher_->InitChannel();

    loco_client_.Init();

    state_subscriber_.InitChannel(
        [this](const void *msg) { this->OnLowState(msg); });

    kp_per_joint_.assign(kTotalMotors, options_.kp);
    for (int idx : right_wrist_pitch_yaw_indices_) {
      kp_per_joint_.at(idx) = options_.kp_right_wrist;
    }
    kp_per_joint_.at(right_wrist_roll_index_) = options_.kp_right_wrist_roll;

    for (int i = 0; i < kTotalMotors; ++i) {
      booster_interface::msg::MotorCmd cmd;
      low_cmd_msg_.motor_cmd().push_back(cmd);
    }
  }

  void OnLowState(const void *msg) {
    const auto *low_state =
        static_cast<const booster_interface::msg::LowState *>(msg);
    const auto &motor_states = low_state->motor_state_serial();
    if (static_cast<int>(motor_states.size()) < kTotalMotors) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_received_ = true;
    for (int i = 0; i < kTotalMotors; ++i) {
      current_joint_states_[i] = motor_states[i].q();
    }
  }

  void InitializeRobotToRetract() {
    RCLCPP_INFO(get_logger(), "Changing robot mode to CUSTOM...");
    const int mode_ret = loco_client_.ChangeMode(booster::robot::RobotMode::kCustom);
    if (mode_ret != 0) {
      RCLCPP_WARN(get_logger(), "ChangeMode returned %d", mode_ret);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_received_) {
          break;
        }
      }
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw std::runtime_error("Timed out waiting for low_state");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::vector<float> start_joints;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start_joints = current_joint_states_;
    }

    std::array<float, 7> left_arm_start{};
    std::array<float, 7> right_arm_start{};
    for (size_t i = 0; i < left_arm_joint_indices_.size(); ++i) {
      left_arm_start[i] = start_joints[left_arm_joint_indices_[i]];
      right_arm_start[i] = start_joints[right_arm_joint_indices_[i]];
    }
    const float waist_start = start_joints[kWaistIndex];
    const float head_yaw_start = start_joints[kHeadYawIndex];
    const float head_pitch_start = start_joints[kHeadPitchIndex];

    const int init_steps = std::max(1, static_cast<int>(options_.init_time / control_dt_));
    const float weight_margin = weight_rate_ * static_cast<float>(control_dt_);

    for (int i = 0; i < init_steps; ++i) {
      const float alpha = static_cast<float>(i + 1) / static_cast<float>(init_steps);
      weight_ = Clamp(weight_ + weight_margin, 0.0f, 0.5f);

      for (size_t j = 0; j < left_arm_joint_indices_.size(); ++j) {
        const float left_q =
            left_arm_start[j] + (left_arm_init_[j] - left_arm_start[j]) * alpha;
        const float right_q =
            right_arm_start[j] + (right_arm_init_[j] - right_arm_start[j]) * alpha;
        SetMotorCmd(left_arm_joint_indices_[j], left_q, 0.0f, weight_);
        SetMotorCmd(right_arm_joint_indices_[j], right_q, 0.0f, weight_);
      }

      const float waist_q = waist_start + (waist_init_ - waist_start) * alpha;
      const float head_yaw_q =
          head_yaw_start + (head_yaw_init_ - head_yaw_start) * alpha;
      const float head_pitch_q =
          head_pitch_start + (head_pitch_init_ - head_pitch_start) * alpha;
      SetMotorCmd(kWaistIndex, waist_q, 0.0f, weight_);
      SetMotorCmd(kHeadYawIndex, head_yaw_q, 0.0f, weight_);
      SetMotorCmd(kHeadPitchIndex, head_pitch_q, 0.0f, weight_);

      PublishLowCmd();
      std::this_thread::sleep_for(
          std::chrono::duration<double>(control_dt_));
    }

    current_right_arm_pos_ = right_arm_init_;
    current_waist_pos_ = waist_init_;
    current_head_yaw_ = head_yaw_init_;
    current_head_pitch_ = head_pitch_init_;

    RCLCPP_INFO(get_logger(), "Initialization complete");
  }

  void TrajectoryCallback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty trajectory, ignoring");
      return;
    }

    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (is_executing_ || pending_trajectory_ != nullptr) {
      RCLCPP_WARN(get_logger(),
                  "Already executing or queued trajectory; ignoring new one");
      return;
    }

    pending_trajectory_ = msg;
    RCLCPP_INFO(get_logger(), "Queued trajectory with %zu waypoints",
                msg->points.size());
  }

  void ControlLoop() {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!is_executing_ && pending_trajectory_ != nullptr) {
        active_trajectory_ = pending_trajectory_;
        pending_trajectory_.reset();
        active_waypoint_index_ = 0;
        is_executing_ = true;
        RCLCPP_INFO(get_logger(), "Starting trajectory execution");
      }
    }

    if (is_executing_) {
      ExecuteOneWaypoint();
    } else {
      SendHoldCommand();
    }
  }

  void ExecuteOneWaypoint() {
    if (!active_trajectory_) {
      is_executing_ = false;
      return;
    }

    if (active_waypoint_index_ >= active_trajectory_->points.size()) {
      is_executing_ = false;
      active_trajectory_.reset();
      RCLCPP_INFO(get_logger(), "Trajectory execution complete");
      return;
    }

    const auto &point = active_trajectory_->points[active_waypoint_index_];
    const size_t n = point.positions.size();
    if (n != 10) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Invalid waypoint dimension: %zu (expected 10)", n);
      ++active_waypoint_index_;
      return;
    }

    std::array<float, 7> target_right_arm = current_right_arm_pos_;
    std::array<float, 7> target_right_arm_dq{};
    float target_waist_q = static_cast<float>(point.positions[0]);
    float target_waist_dq = 0.0f;
    float target_head_yaw = Clamp(static_cast<float>(point.positions[8]),
                                  options_.head_yaw_min, options_.head_yaw_max);
    float target_head_pitch =
        Clamp(static_cast<float>(point.positions[9]), options_.head_pitch_min,
              options_.head_pitch_max);
    float target_head_yaw_dq = 0.0f;
    float target_head_pitch_dq = 0.0f;

    if (point.velocities.size() == point.positions.size()) {
      target_waist_dq = static_cast<float>(point.velocities[0]);
      target_head_yaw_dq = static_cast<float>(point.velocities[8]);
      target_head_pitch_dq = static_cast<float>(point.velocities[9]);
      for (size_t i = 0; i < target_right_arm.size(); ++i) {
        target_right_arm_dq[i] = static_cast<float>(point.velocities[i + 1]);
      }
    }

    for (size_t i = 0; i < target_right_arm.size(); ++i) {
      target_right_arm[i] = static_cast<float>(point.positions[i + 1]);
    }

    for (size_t j = 0; j < left_arm_joint_indices_.size(); ++j) {
      SetMotorCmd(left_arm_joint_indices_[j], left_arm_init_[j], 0.0f, weight_);
    }
    for (size_t j = 0; j < right_arm_joint_indices_.size(); ++j) {
      SetMotorCmd(right_arm_joint_indices_[j], target_right_arm[j],
                  target_right_arm_dq[j], weight_);
    }
    SetMotorCmd(kWaistIndex, target_waist_q, target_waist_dq, weight_);
    SetMotorCmd(kHeadYawIndex, target_head_yaw, target_head_yaw_dq, weight_);
    SetMotorCmd(kHeadPitchIndex, target_head_pitch, target_head_pitch_dq, weight_);

    PublishLowCmd();

    current_right_arm_pos_ = target_right_arm;
    current_waist_pos_ = target_waist_q;
    current_head_yaw_ = target_head_yaw;
    current_head_pitch_ = target_head_pitch;

    ++active_waypoint_index_;
  }

  void SendHoldCommand() {
    for (size_t j = 0; j < left_arm_joint_indices_.size(); ++j) {
      SetMotorCmd(left_arm_joint_indices_[j], left_arm_init_[j], 0.0f, weight_);
    }
    for (size_t j = 0; j < right_arm_joint_indices_.size(); ++j) {
      SetMotorCmd(right_arm_joint_indices_[j], current_right_arm_pos_[j], 0.0f,
                  weight_);
    }

    SetMotorCmd(kWaistIndex, current_waist_pos_, 0.0f, weight_);
    SetMotorCmd(kHeadYawIndex, current_head_yaw_, 0.0f, weight_);
    SetMotorCmd(kHeadPitchIndex, current_head_pitch_, 0.0f, weight_);

    PublishLowCmd();
  }

  void SetMotorCmd(int idx, float q, float dq, float weight) {
    auto &cmd = low_cmd_msg_.motor_cmd().at(static_cast<size_t>(idx));
    cmd.q(q);
    cmd.dq(dq);
    cmd.kp(kp_per_joint_.at(static_cast<size_t>(idx)));
    cmd.kd(options_.kd);
    cmd.tau(tau_ff_);
    cmd.weight(weight);
  }

  void PublishLowCmd() {
    publisher_->Write(&low_cmd_msg_);
  }
};

bool ParseArg(int argc, char **argv, const std::string &key, std::string &out) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (key == argv[i]) {
      out = argv[i + 1];
      return true;
    }
  }
  return false;
}

template <typename T>
bool ParseNumericArg(int argc, char **argv, const std::string &key, T &out) {
  std::string value;
  if (!ParseArg(argc, argv, key, value)) {
    return false;
  }
  try {
    if constexpr (std::is_same<T, float>::value) {
      out = std::stof(value);
    } else if constexpr (std::is_same<T, double>::value) {
      out = std::stod(value);
    }
  } catch (...) {
    return false;
  }
  return true;
}

Options ParseOptions(int argc, char **argv) {
  Options options;
  ParseArg(argc, argv, "--network_interface", options.network_interface);
  ParseNumericArg(argc, argv, "--control_freq", options.control_freq);
  ParseNumericArg(argc, argv, "--kp", options.kp);
  ParseNumericArg(argc, argv, "--kp_right_wrist", options.kp_right_wrist);
  ParseNumericArg(argc, argv, "--kp_right_wrist_roll",
                  options.kp_right_wrist_roll);
  ParseNumericArg(argc, argv, "--kd", options.kd);
  ParseNumericArg(argc, argv, "--init_time", options.init_time);
  ParseNumericArg(argc, argv, "--head_yaw_min", options.head_yaw_min);
  ParseNumericArg(argc, argv, "--head_yaw_max", options.head_yaw_max);
  ParseNumericArg(argc, argv, "--head_pitch_min", options.head_pitch_min);
  ParseNumericArg(argc, argv, "--head_pitch_max", options.head_pitch_max);
  return options;
}

void PrintUsage(const char *prog) {
  std::cerr
      << "Usage: " << prog << " --network_interface <iface_or_ip> [options]\n"
      << "Options:\n"
      << "  --control_freq <Hz>\n"
      << "  --kp <float>\n"
      << "  --kp_right_wrist <float>\n"
      << "  --kp_right_wrist_roll <float>\n"
      << "  --kd <float>\n"
      << "  --init_time <sec>\n"
      << "  --head_yaw_min <rad> --head_yaw_max <rad>\n"
      << "  --head_pitch_min <rad> --head_pitch_max <rad>\n";
}

}  // namespace

int main(int argc, char **argv) {
  const Options options = ParseOptions(argc, argv);
  if (options.network_interface.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<BoosterExecutorNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
