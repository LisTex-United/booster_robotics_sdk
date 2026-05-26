#!/usr/bin/env python3
import time
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    RobotMode,
    GripperMotionParameter,
    GripperControlMode,
    B1HandIndex,
    GetModeResponse,
)

# From include/booster/robot/rpc/error.hpp (RPC layer, not Python-specific)
_RPC_MEANING = {
    100: "timeout",
    400: "bad request / invalid parameters",
    409: "conflict with current robot state",
    429: "request too frequent",
    500: "internal server error",
    501: "server refused — often wrong mode (e.g. kDamping) or policy on robot",
    502: "state transition failed",
}


class GripperControlNode(Node):
    """ROS2 node that controls the gripper based on Bool messages."""
    
    def __init__(self, args):
        super().__init__('gripper_control_node')
        
        self.args = args
        
        # Initialize SDK
        self.get_logger().info("Initializing Booster SDK...")
        self._init_sdk()
        self.get_logger().info("Booster SDK ready!")
        
        # Create subscriber for gripper commands
        # Commands: "left_open", "left_close", "right_open", "right_close"
        self.gripper_sub = self.create_subscription(
            String,
            '/gripper/command',
            self.gripper_callback,
            10
        )
        
        self.get_logger().info("Gripper control node ready!")
        self.get_logger().info("Subscribed to /gripper/command")
        self.get_logger().info("Commands: 'left_open', 'left_close', 'right_open', 'right_close'")
    
    def _init_sdk(self):
        """Initialize Booster SDK."""
        ChannelFactory.Instance().Init(0, self.args.network_interface)
        self.client = B1LocoClient()
        self.client.Init()
        
        # Check current mode
        mode_resp = GetModeResponse()
        ret = self.client.GetMode(mode_resp)
        self.get_logger().info(f"Current robot mode: {mode_resp.mode}")
        # Check if we need to make it in Custom Mode here.
    
    def gripper_callback(self, msg):
        """Callback when gripper command is received.
        
        Supported commands:
        - "left_open" -> Opens left gripper
        - "left_close" -> Closes left gripper
        - "right_open" -> Opens right gripper
        - "right_close" -> Closes right gripper
        """
        # ret_switch = self.client.SwitchHandEndEffectorControlMode(True)
        # if ret_switch != 0:
        #     self.get_logger().warning(f"SwitchHandEndEffectorControlMode(True) returned: {ret_switch}")
        # time.sleep(0.1)  # Small delay for mode switch
        
        command = msg.data.strip().lower()
        
        # Parse command to determine hand and action
        if command == "left_open":
            hand_index = B1HandIndex.kLeftHand
            action = "open"
            param = GripperMotionParameter(position=1000, force=200, speed=200)
        elif command == "left_close":
            hand_index = B1HandIndex.kLeftHand
            action = "close"
            param = GripperMotionParameter(position=0, force=600, speed=200)
        elif command == "right_open":
            hand_index = B1HandIndex.kRightHand
            action = "open"
            param = GripperMotionParameter(position=1000, force=200, speed=200)
        elif command == "right_close":
            hand_index = B1HandIndex.kRightHand
            action = "close"
            param = GripperMotionParameter(position=0, force=600, speed=200)
        else:
            self.get_logger().error(f"Invalid command: '{command}'. Expected: 'left_open', 'left_close', 'right_open', or 'right_close'")
            return
        
        hand_name = "left" if hand_index == B1HandIndex.kLeftHand else "right"
        self.get_logger().info(f"{action.capitalize()}ing {hand_name} gripper...")
        
        ret = self.client.ControlGripper(
            param,
            GripperControlMode.kPosition,
            hand_index
        )
        
        hint = _RPC_MEANING.get(ret, "see SDK rpc/error.hpp")
        self.get_logger().info(
            f"ControlGripper ({hand_name} {action}) returned: {ret} ({hint})"
        )
        if ret != 0:
            self.get_logger().error(
                f"Failed to {action} {hand_name} gripper: {ret} — {hint}"
            )
        
        # Note: We keep end-effector control mode enabled to allow future gripper commands
        # If you want to disable it after each command, uncomment the line below:
        # self.client.SwitchHandEndEffectorControlMode(False)


def main():
    parser = argparse.ArgumentParser()
    # First positional argument: network interface (e.g. eth0, enp0s1)
    parser.add_argument(
        "network_interface",
        type=str,
        help="Network interface for robot communication (e.g., eth0, enp0s1)",
    )
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        node = GripperControlNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
