### To use while mapping the map on the stand, which fixes the head in a good position
###! Don't use it while walking as it will enter custom mode and fall to the ground

import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    LowCmdType,
    MotorCmd,
    LowState,
    RobotMode,
)
import argparse
from joints_const import JOINT_PARAMETERS, ARM_NAMES
import numpy as np
import time
import logging

SLEEP_TIME = 0.05
TOTAL_DOF = len(JOINT_PARAMETERS)

parser = argparse.ArgumentParser()

parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")

args = parser.parse_args()
   
class Controller:
    def __init__(self):
        logging.basicConfig(level=logging.WARN)
        self.logger = logging.getLogger(__name__)
        self.dof_pos = np.zeros(TOTAL_DOF, dtype=np.float32)
        self.dof_vel = np.zeros(TOTAL_DOF, dtype=np.float32)
        self._init_communication()
        self._init_motors()
        ret = self.client.ChangeMode(RobotMode.kPrepare)   
        print(f"Switched to prepare mode: {ret}")
              
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
            time.sleep(2)  # Wait for channels to initialize
            print("Initialization complete.")
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise
    
    def _low_state_handler(self, low_state_msg: LowState):
        for i, motor in enumerate(low_state_msg.motor_state_parallel):
            self.dof_pos[i] = motor.q
            self.dof_vel[i] = motor.dq
            
    def cleanup(self) -> None:
        """Cleanup resources."""
        if hasattr(self, "low_cmd_publisher"):
            self.low_cmd_publisher.CloseChannel()
        if hasattr(self, "low_state_subscriber"):
            self.low_state_subscriber.CloseChannel()
        
    
    def start_custom_mode(self):
        arms = [JOINT_PARAMETERS[name] for name in ARM_NAMES]
        for i, joint in enumerate(arms):
            motor_idx = joint["idx"]
            self.low_cmd.motor_cmd[motor_idx].q = 0.0
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = 0.0
            self.low_cmd.motor_cmd[motor_idx].kd = 3.5
            self.low_cmd.motor_cmd[motor_idx].weight = 1.0

        fix_joint_names = ["Head_Yaw", "Head_Pitch", "Waist"]  
        head = [JOINT_PARAMETERS[name] for name in fix_joint_names]
        for i, joint in enumerate(head):
            motor_idx = joint["idx"]
            print(fix_joint_names[i], joint["idx"], joint["kp"], joint["kd"])
            self.low_cmd.motor_cmd[motor_idx].q = 0.0
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
            self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
            self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            
        self.low_cmd_publisher.Write(self.low_cmd)
        time.sleep(2.0)
        mode = RobotMode.kCustom
        input(f"Press ENTER to switch to {mode} mode ...")
        ret=self.client.ChangeMode(mode)
        print(f"Switched to custom mode: {ret}")
        time.sleep(3.0)

        for i, joint in enumerate(arms):
            motor_idx = joint["idx"]
            self.low_cmd.motor_cmd[motor_idx].q = 0.0
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = 0.0
            self.low_cmd.motor_cmd[motor_idx].kd = 0.5
            self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            
        self.low_cmd_publisher.Write(self.low_cmd)
        time.sleep(0.5)
        
    def head_control(self, goal):
        limits = {
            "Head_Yaw": (-0.65, 0.65),
            "Head_Pitch": (-0.20, 0.70)
        }

        for name, (min_limit, max_limit) in limits.items():
            if not (min_limit <= goal[name] <= max_limit):
                raise ValueError(f"Goal for {name} is out of limits: {goal[name]} not in ({min_limit}, {max_limit})")
            
        controlled_j = [JOINT_PARAMETERS[name] for name in limits.keys()]
        
        positions = np.array((np.linspace(self.dof_pos[controlled_j[0]["idx"]],goal["Head_Yaw"], num=15), np.linspace(self.dof_pos[controlled_j[1]["idx"]], goal["Head_Pitch"], num=15))).swapaxes(1, 0)
        print("Starting head motion")
        print(controlled_j)
        print("Plan shape:", positions.shape)
        print("Plan start:", positions[0], "Plan end:", positions[-1])
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
    
    def __exit__(self, *args) -> None:
        self.cleanup()

def main():
    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)
    controller = Controller()
    controller.start_custom_mode()
    
    final_goal = {
            "Head_Yaw": 0.0,
            "Head_Pitch": 0.28 
        }
    controller.head_control(final_goal)
    print("Head movement demo completed.")
if __name__ == "__main__":
    main()
