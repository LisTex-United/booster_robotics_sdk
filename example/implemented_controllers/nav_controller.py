import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    RobotMode,
    GetModeResponse,
)
import argparse
from remote_control_service import RemoteControlService
from joints_const import JOINT_PARAMETERS
import numpy as np
import time
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

parser = argparse.ArgumentParser()

parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")

args = parser.parse_args()
   
class Controller(Node):
    def __init__(self):
        super().__init__("controller_node")
        self._init_communication()
        self._cleanup_done = False
        
    def _init_communication(self) -> None:
        try:
            self.client = B1LocoClient()
            self.client.Init()
            self.remoteControl = RemoteControlService()
            self.movement_sub = self.create_subscription(Twist, 'cmd_vel', self.move_handler, 1)
            self.joystick_timer = self.create_timer(0.05, self.run_joystick)
            time.sleep(2)  # Wait for channels to initialize
            print("Initialization complete.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize communication: {e}")
            raise
        
    def start_walking_mode(self):
        """Switch the robot to walking mode safely. If already in walking mode, does nothing.
        If not, first switches to prepare mode, waits for it to initialize and then switches to walking mode."""
        current_mode = GetModeResponse()
        pitch_head = 0.2
        if self.client.GetMode(current_mode):
            raise RuntimeError("Could not get current robot mode.")
        
        if  current_mode.mode != RobotMode.kWalking:
            self.get_logger().info("Robot is not in walking mode. Switching to prepare mode first.")
            ret = self.client.ChangeMode(RobotMode.kPrepare)   
            print(f"Switched to prepare mode: {ret}")
            time.sleep(5.0) # Wait for prepare mode to initialize
        
        elif current_mode.mode == RobotMode.kWalking:
            self.get_logger().info("Robot is already in walking mode. Sending zero velocity command to ensure safe state.")
            # self.client.Move(0.0, 0.0, 0.0)  # Send zero velocity command to ensure safe state
            print(f"{self.remoteControl.get_operation_hint()}")
            self.client.RotateHead(pitch_head, 0.0)
            return

        # Wait for user to start walking via remote control
        print(f"{self.remoteControl.get_walk_operation_hint()}")
        while not self.remoteControl.start_walk():
            time.sleep(0.1)

        mode = RobotMode.kWalking
        ret=self.client.ChangeMode(mode)
        print(f"Switched to {mode} mode: {ret}")
        print(f"{self.remoteControl.get_operation_hint()}")
        time.sleep(3.0)
        self.client.RotateHead(pitch_head, 0.0)

    def run_joystick(self):
        if self.remoteControl.send_stop():
            self.client.Move(0.0, 0.0, 0.0)
        
    def move_handler(self, msg: Twist):
        #* Ignore NAV2 commands when joystick is controlling
        if self.remoteControl.is_joystick_controlling():
            return          
        
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        
        if abs(x) > 0.5 or abs(y) > 0.5 or abs(z) > 1.0:  #Debug
            self.get_logger().warning("Received movement command exceeds limits. Ignoring.")
            return
        
        self.client.Move(x, y, z)
    
    def cleanup(self) -> None:
        """Clean up resources (idempotent)."""
        if self._cleanup_done:
            return
        self._cleanup_done = True

        self.get_logger().info("Doing cleanup...")
        
        # close communications
        try:
            self.remoteControl.close()
        except Exception as e:
            self.logger.error(f"Error closing remote control: {e}")

        if rclpy.ok():
            rclpy.shutdown()

        self.get_logger().info("Cleanup complete")
               
    def __exit__(self, *args) -> None:
        self.cleanup()


def main():
    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)
    rclpy.init()
    controller = Controller()
    controller.start_walking_mode()
    rclpy.spin(controller)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
