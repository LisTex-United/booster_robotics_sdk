import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    RobotMode,
    GetModeResponse,
    B1LowStateSubscriber,
    LowState,
    LowCmd,
    LowCmdType,
    MotorCmd,
    B1LowCmdPublisher
)
import argparse
from remote_control_service import RemoteControlService
from joints_const import JOINT_PARAMETERS, ARM_NAMES, TOTAL_DOF
import time
import numpy as np
import logging
import rclpy
from rclpy.node import Node
SLEEP_TIME = 0.05

parser = argparse.ArgumentParser()

parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")

args = parser.parse_args()
   
class Controller(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.dof_pos = np.zeros(TOTAL_DOF, dtype=np.float32)
        self.dof_vel = np.zeros(TOTAL_DOF, dtype=np.float32)
        self._init_communication()
        self._init_motors()
        self._cleanup_done = False
        
    def _init_motors(self):
        self.low_cmd = LowCmd()
        self.low_cmd.cmd_type = LowCmdType.PARALLEL
        motor_cmds = [MotorCmd() for _ in range(TOTAL_DOF)]
        self.low_cmd.motor_cmd = motor_cmds
        for motor_idx in range(TOTAL_DOF):
            self.low_cmd.motor_cmd[motor_idx].q = 0.0
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = 0.0
            self.low_cmd.motor_cmd[motor_idx].kd = 0.0
            self.low_cmd.motor_cmd[motor_idx].weight = 0.0

    def _init_communication(self) -> None:
        try:
            self.low_state_subscriber = B1LowStateSubscriber(self._low_state_handler)
            self.low_cmd_publisher = B1LowCmdPublisher()
            self.client = B1LocoClient()
            
            self.low_state_subscriber.InitChannel()
            self.low_cmd_publisher.InitChannel()
            self.client.Init()
            self.remoteControl = RemoteControlService()
            time.sleep(2)  # Wait for channels to initialize
            print("Initialization complete.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize communication: {e}")
            raise
        
    def start_walking_mode(self):
        """Switch the robot to walking mode safely. If already in walking mode, does nothing.
        If not, first switches to prepare mode, waits for it to initialize and then switches to walking mode."""
        pitch_head = 0.2
        ret = self.client.ChangeMode(RobotMode.kPrepare)   
        print(f"Switched to prepare mode: {ret}")
        time.sleep(5.0) # Wait for prepare mode to initialize
        print(f"{self.remoteControl.get_walk_operation_hint()}")
        while not self.remoteControl.start_walk():
            time.sleep(0.1)
        mode = RobotMode.kWalking
        ret=self.client.ChangeMode(mode)
        print(f"Switched to {mode} mode: {ret}")
        time.sleep(3.0)
        self.client.RotateHead(pitch_head, 0.0)
        print(f"{self.remoteControl.get_operation_hint()}")
        controlled_j = [JOINT_PARAMETERS[name] for name in ARM_NAMES]

        print(f"From subscriber 1: ", [self.dof_pos[joint['idx']] for joint in controlled_j])
        controlled_j = [JOINT_PARAMETERS[name] for name in ARM_NAMES]
        for i, joint in enumerate(controlled_j):
                motor_idx = joint["idx"]
                self.low_cmd.motor_cmd[motor_idx].q = self.dof_pos[motor_idx]
                self.low_cmd.motor_cmd[motor_idx].dq = 0.0
                self.low_cmd.motor_cmd[motor_idx].tau = 0.0
                self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
                self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
                self.low_cmd.motor_cmd[motor_idx].weight = 1.0
        self.low_cmd_publisher.Write(self.low_cmd)
        input("Press Enter to switch to upper body custom control mode...")
        self.client.UpperBodyCustomControl(True)
        time.sleep(0.5)
        self.low_arm()
        

    def low_arm(self):
        final_position = [0.0, -1.25, 0, -0.5, 0, 0, 0, 0.0, 1.25, 0, 0.5, 0, 0, 0] 
        controlled_j = [JOINT_PARAMETERS[name] for name in ARM_NAMES]                        
            
        positions = np.array([np.linspace(self.dof_pos[joint['idx']], 
                                          final_position[i], num=20) for i, joint in enumerate(controlled_j)]).swapaxes(1, 0)
        cmd_idx = 0
        while cmd_idx < positions.shape[0]:
            cmd_state = positions[cmd_idx]
            for i, joint in enumerate(controlled_j):
                motor_idx = joint["idx"]
                self.low_cmd.motor_cmd[motor_idx].q = cmd_state[i]
                self.low_cmd.motor_cmd[motor_idx].dq = 0.0
                self.low_cmd.motor_cmd[motor_idx].tau = 0.0
                self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
                self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
                self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            self.low_cmd_publisher.Write(self.low_cmd)
            cmd_idx += 1
            time.sleep(SLEEP_TIME)
        time.sleep(0.5)
        return
    
    # def run_joystick(self):
    #     if self.remoteControl.send_stop():
    #         self.turn_off_arms()
    
    # def turn_off_arms(self):
    #     arms = [JOINT_PARAMETERS[name] for name in ARM_NAMES]
    #     for _, joint in enumerate(arms):
    #         motor_idx = joint["idx"]
    #         self.low_cmd.motor_cmd[motor_idx].q = 0.0
    #         self.low_cmd.motor_cmd[motor_idx].dq = 0.0
    #         self.low_cmd.motor_cmd[motor_idx].tau = 0.0
    #         self.low_cmd.motor_cmd[motor_idx].kp = 0.0
    #         self.low_cmd.motor_cmd[motor_idx].kd = 3.0
    #         self.low_cmd.motor_cmd[motor_idx].weight = 1.0

    def _low_state_handler(self, low_state_msg: LowState):
        for i, motor in enumerate(low_state_msg.motor_state_parallel):
            self.dof_pos[i] = motor.q
            self.dof_vel[i] = motor.dq
    
    def cleanup(self) -> None:
        """Clean up resources (idempotent)."""
        if self._cleanup_done:
            return
        self._cleanup_done = True

        """Cleanup resources."""
        self.remoteControlService.close()
        if hasattr(self, "low_cmd_publisher"):
            self.low_cmd_publisher.CloseChannel()
        if hasattr(self, "low_state_subscriber"):
            self.low_state_subscriber.CloseChannel()
               
    def __exit__(self, *args) -> None:
        self.cleanup()


def main():
    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)
    rclpy.init()
    controller = Controller()
    controller.start_walking_mode()
    
if __name__ == "__main__":
    main()
