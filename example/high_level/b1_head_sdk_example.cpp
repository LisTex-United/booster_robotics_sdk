#include <array>
#include <chrono>
#include <iostream>
#include <thread>


#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>

static const std::string kTopicHeadSDK = "rt/joint_ctrl";

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd>
      head_sdk_publisher;
  booster_interface::msg::LowCmd msg;

  booster::robot::b1::B1LocoClient client;
  client.Init();

  head_sdk_publisher.reset(
      new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
          kTopicHeadSDK));
  head_sdk_publisher->InitChannel();

  std::array<booster::robot::b1::JointIndex, 2> head_joints = {
      booster::robot::b1::JointIndex::kHeadYaw,
      booster::robot::b1::JointIndex::kHeadPitch
  };

  // init control params
  float weight = 0.f;
  float weight_rate = 0.2f;

  float kp = 55.f;
  float kd = 1.5f;
  float dq = 0.f;
  float tau_ff = 0.f;

  float control_dt = 0.02f;
  float max_joint_velocity = 0.5f;

  float weight_margin = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  // Head joint position array (2 joints: yaw and pitch)
  // Values are in radians. Mapping to head_joints array:
  // Index 0: kHeadYaw   (Left/Right rotation: positive = left, negative = right)
  // Index 1: kHeadPitch (Up/Down rotation: positive = down, negative = up)
  std::array<float, 2> init_pos{0.0f, 0.0f};  // Start at center position

  std::array<float, 2> target_pos{0.0f, 0.3f};

  // wait for init
  std::cout << "Press ENTER to init head ..." << std::endl;
  std::cin.get();

  int32_t ret = client.ChangeMode(booster::robot::RobotMode::kCustom);
  if (ret != 0) {
    std::cout << "Failed to change mode to kCustom, error code: " << ret << std::endl;
    return -1;
  }

  // Initialize message with all joints
  for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
    booster_interface::msg::MotorCmd motor_cmd;
    msg.motor_cmd().push_back(motor_cmd);
  }

  // set init pos
  std::cout << "Initializing head position ..." << std::endl;
  float init_time = 3.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  // init head joints
  for (int i = 0; i < init_time_steps; ++i) {
    // increase weight
    weight += weight_margin;
    weight = std::clamp(weight, 0.f, 0.5f);
    
    if (i % 50 == 0) {
      std::cout << "Weight: " << weight << std::endl;
    }

    // set control for head joints only
    for (int j = 0; j < head_joints.size(); ++j) {
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).q(init_pos.at(j));
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).dq(dq);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kp(kp);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kd(kd);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).tau(tau_ff);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).weight(weight);
    }

    // send dds msg
    head_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;

  // wait for control
  std::cout << "Press ENTER to start head movement ..." << std::endl;
  std::cin.get();

  // start control
  std::cout << "Moving head to target position!" << std::endl;
  float period = 5.f;
  int num_time_steps = static_cast<int>(period / control_dt);

  std::array<float, 2> current_jpos_des = init_pos;

  // move head to target position
  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < head_joints.size(); ++j) {
      current_jpos_des.at(j) +=
          std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                     -max_joint_delta, max_joint_delta);
    }
    
    if (i % 25 == 0) {
      std::cout << "Current Yaw: " << current_jpos_des.at(0) 
                << " rad, Pitch: " << current_jpos_des.at(1) << " rad" << std::endl;
    }

    // set control for head joints
    for (int j = 0; j < head_joints.size(); ++j) {
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).q(current_jpos_des.at(j));
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).dq(dq);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kp(kp);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kd(kd);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).tau(tau_ff);
    }

    // send dds msg
    head_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Head movement complete!" << std::endl;

  // Optional: Return to center position
  std::cout << "Press ENTER to return head to center ..." << std::endl;
  std::cin.get();

  std::array<float, 2> center_pos{0.0f, 0.0f};
  
  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des to return to center
    for (int j = 0; j < head_joints.size(); ++j) {
      current_jpos_des.at(j) +=
          std::clamp(center_pos.at(j) - current_jpos_des.at(j),
                     -max_joint_delta, max_joint_delta);
    }

    // set control for head joints
    for (int j = 0; j < head_joints.size(); ++j) {
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).q(current_jpos_des.at(j));
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).dq(dq);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kp(kp);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).kd(kd);
      msg.motor_cmd().at(static_cast<int>(head_joints.at(j))).tau(tau_ff);
    }

    // send dds msg
    head_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done! Head returned to center." << std::endl;

  return 0;
}
