import time
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    RobotMode,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    MotorCmd,
    GetModeResponse,
)


class BoosterHeadControlNode(Node):
    """ROS2 node for head-only control (yaw, pitch) while holding all other joints."""

    HEAD_YAW_INDEX = 0
    HEAD_PITCH_INDEX = 1
    TOTAL_MOTORS = 29

    def __init__(self, args):
        super().__init__("booster_head_control_node")
        self.args = args

        self.state_received = False
        self.current_joint_states = [0.0] * self.TOTAL_MOTORS

        self.target_yaw = args.head_yaw
        self.target_pitch = args.head_pitch
        self.cmd_yaw = args.head_yaw
        self.cmd_pitch = args.head_pitch

        self.max_delta = args.max_speed / args.control_freq

        self.get_logger().info("Initializing Booster SDK...")
        self._init_sdk()
        self.get_logger().info("Booster SDK ready")

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            "/booster/head/command",
            self._command_callback,
            10,
        )

        self.get_logger().info(
            "Subscribed /booster/head/command (Float64MultiArray: [yaw, pitch])"
        )

        self.get_logger().info("Waiting for initial robot state...")
        self._wait_for_state_or_exit(timeout_sec=5.0)

        input("\n[PRESS ENTER] to switch to CUSTOM mode and start head control...\n")

        self._set_custom_mode()

        self.hold_positions = list(self.current_joint_states)
        self.cmd_yaw = self.hold_positions[self.HEAD_YAW_INDEX]
        self.cmd_pitch = self.hold_positions[self.HEAD_PITCH_INDEX]
        self.target_yaw = self.cmd_yaw
        self.target_pitch = self.cmd_pitch

        self.get_logger().info(
            f"Initial head state: yaw={self.cmd_yaw:.3f}, pitch={self.cmd_pitch:.3f}"
        )
        self.get_logger().info(
            f"Safety limits: yaw[{self.args.yaw_min:.2f}, {self.args.yaw_max:.2f}], "
            f"pitch[{self.args.pitch_min:.2f}, {self.args.pitch_max:.2f}]"
        )

        self._run_control_loop()

    def _init_sdk(self):
        ChannelFactory.Instance().Init(0, self.args.network_interface)

        self.publisher = B1LowCmdPublisher()
        self.msg = LowCmd()

        self.client = B1LocoClient()
        self.client.Init()

        self.state_subscriber = B1LowStateSubscriber(self._state_callback)
        self.state_subscriber.InitChannel()

        self.publisher.InitChannel()

        self.control_dt = 1.0 / self.args.control_freq
        self.motor_cmd_list = [MotorCmd() for _ in range(self.TOTAL_MOTORS)]

    def _state_callback(self, msg):
        self.state_received = True
        if len(msg.motor_state_serial) >= self.TOTAL_MOTORS:
            for i in range(self.TOTAL_MOTORS):
                self.current_joint_states[i] = msg.motor_state_serial[i].q

    def _wait_for_state_or_exit(self, timeout_sec):
        start = time.time()
        while not self.state_received and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        if not self.state_received:
            raise RuntimeError("Failed to receive robot state")

    def _set_custom_mode(self):
        ret = self.client.ChangeMode(RobotMode.kCustom)
        self.get_logger().info(f"ChangeMode -> {ret}")
        mode_resp = GetModeResponse()
        ret = self.client.GetMode(mode_resp)
        self.get_logger().info(f"GetMode -> {ret}, mode={mode_resp.mode}")
        time.sleep(0.6)

    def _clamp(self, value, low, high):
        return max(low, min(high, value))

    def _step_toward(self, current, target):
        delta = target - current
        if abs(delta) <= self.max_delta:
            return target
        return current + self.max_delta if delta > 0.0 else current - self.max_delta

    def _command_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                "Ignoring /booster/head/command: expected [yaw, pitch]"
            )
            return

        yaw = self._clamp(msg.data[0], self.args.yaw_min, self.args.yaw_max)
        pitch = self._clamp(msg.data[1], self.args.pitch_min, self.args.pitch_max)

        self.target_yaw = yaw
        self.target_pitch = pitch

        self.get_logger().info(
            f"New head target -> yaw={self.target_yaw:.3f}, pitch={self.target_pitch:.3f}"
        )

    def _run_control_loop(self):
        self.get_logger().info("Starting head control loop")

        try:
            while rclpy.ok():
                if self.state_received:
                    self.hold_positions = list(self.current_joint_states)

                self.cmd_yaw = self._step_toward(self.cmd_yaw, self.target_yaw)
                self.cmd_pitch = self._step_toward(self.cmd_pitch, self.target_pitch)

                for motor_idx in range(self.TOTAL_MOTORS):
                    self.motor_cmd_list[motor_idx].q = self.hold_positions[motor_idx]
                    self.motor_cmd_list[motor_idx].dq = 0.0
                    self.motor_cmd_list[motor_idx].kp = self.args.hold_kp
                    self.motor_cmd_list[motor_idx].kd = self.args.hold_kd
                    self.motor_cmd_list[motor_idx].tau = 0.0

                self.motor_cmd_list[self.HEAD_YAW_INDEX].q = self.cmd_yaw
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self.args.head_kp
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.head_kd

                self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = self.cmd_pitch
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self.args.head_kp
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kd = self.args.head_kd

                self.msg.motor_cmd = self.motor_cmd_list
                self.publisher.Write(self.msg)

                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(self.control_dt)

        except KeyboardInterrupt:
            self.get_logger().info("Stopping head control loop")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--network_interface", type=str, required=True)
    parser.add_argument("--control_freq", type=float, default=100.0)

    parser.add_argument("--head_yaw", type=float, default=0.0)
    parser.add_argument("--head_pitch", type=float, default=0.4)

    parser.add_argument("--head_kp", type=float, default=120.0)
    parser.add_argument("--head_kd", type=float, default=1.2)
    parser.add_argument("--hold_kp", type=float, default=80.0)
    parser.add_argument("--hold_kd", type=float, default=1.0)

    parser.add_argument("--max_speed", type=float, default=0.8, help="rad/s")
    parser.add_argument("--yaw_min", type=float, default=-1.2)
    parser.add_argument("--yaw_max", type=float, default=1.2)
    parser.add_argument("--pitch_min", type=float, default=-0.8)
    parser.add_argument("--pitch_max", type=float, default=0.8)

    args = parser.parse_args()

    rclpy.init()
    try:
        _ = BoosterHeadControlNode(args)
    except Exception as exc:
        print(f"Error: {exc}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
