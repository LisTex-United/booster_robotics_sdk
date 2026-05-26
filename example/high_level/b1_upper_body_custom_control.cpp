#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/common/entities.hpp>
#include <booster/robot/common/robot_shared.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

const char *RobotModeString(booster::robot::RobotMode mode) {
    using booster::robot::RobotMode;
    switch (mode) {
    case RobotMode::kUnknown:
        return "kUnknown";
    case RobotMode::kDamping:
        return "kDamping";
    case RobotMode::kPrepare:
        return "kPrepare";
    case RobotMode::kWalking:
        return "kWalking";
    case RobotMode::kCustom:
        return "kCustom";
    case RobotMode::kSoccer:
        return "kSoccer";
    default:
        return "?(invalid)";
    }
}

} // namespace

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <networkInterface> [start|stop] [no-hand]\n";
        std::cerr << "  GetMode, then UpperBodyCustomControl.\n";
        std::cerr << "  With start (default): enables EE control mode and calls MoveHandEndEffector\n";
        std::cerr << "  (demo pose, right hand, torso frame — same idea as b1_loco_example_client \"hand-down\").\n";
        std::cerr << "  Use no-hand to skip SwitchHandEndEffectorControlMode / MoveHandEndEffector.\n";
        std::cerr << "  stop — UpperBodyCustomControl(false)\n";
        return 1;
    }

    booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    booster::robot::b1::B1LocoClient client;
    client.Init();

    booster::robot::b1::GetModeResponse mode_resp;
    const int32_t mode_ret = client.GetMode(mode_resp);
    std::cout << "GetMode returned " << mode_ret << ", current mode: "
              << static_cast<int>(mode_resp.mode_) << " (" << RobotModeString(mode_resp.mode_) << ")\n";
    if (mode_ret != 0) {
        std::cerr << "Warning: GetMode failed; continuing with UpperBodyCustomControl anyway.\n";
    }

    // using booster::robot::RobotMode;
    // if (mode_ret == 0 && mode_resp.mode_ == RobotMode::kCustom) {
    //     std::cout << "Already in kCustom mode.\n";
    // } else {
    //     std::cout << "Changing mode to kCustom...\n";
    //     const int32_t change_ret = client.ChangeMode(RobotMode::kCustom);
    //     std::cout << "ChangeMode(kCustom) returned " << change_ret
    //               << (change_ret == 0 ? " (ok)\n" : " (error)\n");
    //     if (change_ret != 0) {
    //         return 1;
    //     }
    //     std::this_thread::sleep_for(std::chrono::seconds(1));

    //     booster::robot::b1::GetModeResponse after;
    //     const int32_t after_ret = client.GetMode(after);
    //     std::cout << "GetMode (after change) returned " << after_ret << ", mode: "
    //               << static_cast<int>(after.mode_) << " (" << RobotModeString(after.mode_) << ")\n";
    // }

    bool start = true;
    bool do_hand_move = true;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "stop" || arg == "0" || arg == "false") {
            start = false;
        } else if (arg == "no-hand") {
            do_hand_move = false;
        }
    }

    const int32_t ret = client.UpperBodyCustomControl(start);
    std::cout << "UpperBodyCustomControl(" << (start ? "true" : "false") << ") returned " << ret
              << (ret == 0 ? " (ok)\n" : " (error)\n");

    int exit_code = (ret == 0) ? 0 : 1;

    if (ret == 0 && start && do_hand_move) {
        // API note: end-effector moves expect end-effector control mode (often set on RC; SDK can enable it).
        std::cout << "SwitchHandEndEffectorControlMode(true)...\n";
        const int32_t hcm_ret = client.SwitchHandEndEffectorControlMode(true);
        std::cout << "SwitchHandEndEffectorControlMode(true) returned " << hcm_ret
                  << (hcm_ret == 0 ? " (ok)\n" : " (error)\n");
        if (hcm_ret != 0) {
            exit_code = 1;
        }

        // Demo target in base (torso) frame — meters / rad; matches interactive "hand-down" example.
        booster::robot::Posture tar_posture;
        tar_posture.position_ = booster::robot::Position(0.35f, -0.2f, 0.207f);
        tar_posture.orientation_ = booster::robot::Orientation(3.12f, 0.54f, 2.907f);
        constexpr int k_move_duration_ms = 10000;
        const int32_t move_ret =
            client.MoveHandEndEffector(tar_posture, k_move_duration_ms,
                                       booster::robot::b1::HandIndex::kRightHand);
        std::cout << "MoveHandEndEffector(pos 0.29,-0.26,0.08, rpy 0,0,0, " << k_move_duration_ms
                  << " ms, kRightHand) returned " << move_ret
                  << (move_ret == 0 ? " (ok)\n" : " (error)\n");
        if (move_ret != 0) {
            exit_code = 1;
        }
    }

    return exit_code;
}
