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
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util_file import (
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    PoseCostMetric,
)
import argparse
from joints_const import JOINT_PARAMETERS
import numpy as np
import time
import logging

# from utils.command import create_prepare_cmd, create_first_frame_rl_cmd
# from utils.remote_control_service import RemoteControlService
# from utils.rotate import rotate_vector_inverse_rpy
# from utils.timer import TimerConfig, Timer


SLEEP_TIME = 0.1
TOTAL_DOF = len(JOINT_PARAMETERS)

parser = argparse.ArgumentParser()

parser.add_argument("--robot", type=str, default="booster_t1_left_arm.yml", help="robot configuration to load")
parser.add_argument(
    "--robot_configs_path",
    type=str,
    default="/home/rods/Desktop/Booster/curobo/src/curobo/content/configs/robot",
    help="Path to robot config when loading the robot",
)
#/home/booster/Documents/curobo/src/curobo/content/configs/robot
parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")

args = parser.parse_args()

class Curobo_Motion_Gen:
    def __init__(self, cfg_path=args.robot_configs_path, robot_name=args.robot):
        robot_cfg = load_yaml(join_path(cfg_path, robot_name))["robot_cfg"]
        self.tensor_args = TensorDeviceType()
        #Motion Gen
        world_config = {
        "cuboid": {
            "cube_1": {"dims": [1.0, 1.0, 0.45], "pose": [0.75, 0.2, -0.45, 1, 0, 0, 0]},
            "cube_2": {"dims": [0.3, 0.065, 0.3], "pose": [0.405, 0.06
                                                           , -0.12485, 1, 0, 0, 0]},
            }
        }
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            WorldConfig.from_dict(world_config),
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            # num_ik_seeds=400,  # Increased from 200 for better IK success rate
            interpolation_dt=SLEEP_TIME,
            collision_cache={"obb": 30, "mesh": 100},
            optimize_dt=True,
            trajopt_dt=None,
            trajopt_tsteps=32,
            trim_steps=None,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
        
        #Plan Configs
        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            enable_graph_attempt=2,  # Enable graph planning after 3 failed attempts
            max_attempts=4, 
            enable_finetune_trajopt=True,
            time_dilation_factor=0.5,
            # partial_ik_opt=True,  # Allow partial IK - helps when robot is in difficult configurations
            # num_ik_seeds=None,  # Use default from MotionGenConfig (400 seeds)
            # timeout=15.0,  # Give more time for difficult IK problems
        )
        reach_vec = self.tensor_args.to_device([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])  # Position-only IK
        self.plan_config.pose_cost_metric = PoseCostMetric(reach_partial_pose=True, reach_vec_weight=reach_vec)
        print("Curobo is Ready")
    
    def get_joint_names(self):
       return self.motion_gen.joint_names
   

    def generate_plan(self, start_js, goal_position, goal_orientation):
        position_js = self.tensor_args.to_device(start_js)
        cu_js = JointState(
            position=position_js,
            velocity=position_js * 0.0,
            acceleration=position_js * 0.0,
            jerk=position_js * 0.0,
            joint_names=self.motion_gen.joint_names
            )
        
        ik_goal = Pose(
            position=self.tensor_args.to_device(goal_position),
            quaternion=self.tensor_args.to_device(goal_orientation)
            )
        
        result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config)
        
        if result.success.item():
            print("Motion generation successful")
            cmd_plan = result.get_interpolated_plan()
            return cmd_plan
        
        print("Motion generation failed")
        return None
   
class Controller:
    def __init__(self):
        logging.basicConfig(level=logging.WARN)
        self.logger = logging.getLogger(__name__)
        self.dof_pos = np.zeros(TOTAL_DOF, dtype=np.float32)
        self.dof_vel = np.zeros(TOTAL_DOF, dtype=np.float32)
        self._init_communication()
        self._init_motors()
        self.motion_gen = None
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
            
    def set_motion_gen(self, motion_gen):
        self.motion_gen = motion_gen
    
    def start_custom_mode(self):
        # Left manipulator is kept at the prep position
        init_position = [0.45, -1.05, 0.0, -1.5, 0.0, 0.0, 0.0] #TODO: CHANGE these hardcoded value 
        joint_names = ["Left_Shoulder_Pitch", "Left_Shoulder_Roll",
                       "Left_Elbow_Pitch", "Left_Elbow_Yaw",
                       "Left_Wrist_Pitch", "Left_Wrist_Yaw", "Left_Hand_Roll"
                    ]  
        controlled_j = [JOINT_PARAMETERS[name] for name in joint_names]
        
        for i, joint in enumerate(controlled_j):
            motor_idx = joint["idx"]
            self.low_cmd.motor_cmd[motor_idx].q = init_position[i]
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
            self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
            self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            
        joint_names = ["Right_Shoulder_Pitch", "Right_Shoulder_Roll",
                        "Right_Elbow_Pitch", "Right_Elbow_Yaw",
                        "Right_Wrist_Pitch", "Right_Wrist_Yaw", "Right_Hand_Roll"
                    ]
        right_arm = [JOINT_PARAMETERS[name] for name in joint_names]
        #To reduce the right arm downward movement
        for i, joint in enumerate(right_arm):
            motor_idx = joint["idx"]
            self.low_cmd.motor_cmd[motor_idx].q = 0.0
            self.low_cmd.motor_cmd[motor_idx].dq = 0.0
            self.low_cmd.motor_cmd[motor_idx].tau = 0.0
            self.low_cmd.motor_cmd[motor_idx].kp = 0.0
            self.low_cmd.motor_cmd[motor_idx].kd = 1.5
            self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            
        self.low_cmd_publisher.Write(self.low_cmd)
        time.sleep(2.0)
        mode = RobotMode.kCustom
        input(f"Press ENTER to switch to {mode} mode ...")
        ret=self.client.ChangeMode(mode)
        print(f"Switched to custom mode: {ret}")
        time.sleep(2.0)
        
    def control_manipulator(self, goal_position, goal_orientation):  
        if self.motion_gen is None:
            print("Motion generator not set.")
            return
        
        controlled_j = [JOINT_PARAMETERS[name] for name in self.motion_gen.get_joint_names()]
        
        #cmd_plan = cmd_plan_full.get_ordered_joint_state(joint_names)
        start_js = [self.dof_pos[joint["idx"]] for joint in controlled_j]
        cmd_plan = self.motion_gen.generate_plan(
            start_js=[0.45, -1.05, 0.0, -1.5, 0.0, 0.0, 0.0], #TODO: REMOVE! IN SIMULATION THIS DOESN'T WORK SO WE HARDCODED THE VALUE
            goal_position=goal_position,
            goal_orientation=goal_orientation
        )
        print("Values from the subscriber:", start_js)
        print("First position:", cmd_plan[0].position.cpu().numpy())
        print("Joint names:", cmd_plan[0].joint_names)
        print("Order: ", [JOINT_PARAMETERS[name]["idx"] for name in cmd_plan[0].joint_names])
        input("Press ENTER to start motion ...")
        cmd_idx = 0
        while cmd_plan is not None:
            cmd_state = cmd_plan[cmd_idx]
            positions = cmd_state.position.cpu().numpy()
            velocities = cmd_state.velocity.cpu().numpy()
            if cmd_idx  >= len(cmd_plan.position) - 1:
                velocities = np.zeros_like(positions) # Stop at the last position
            for i, joint in enumerate(controlled_j):
                motor_idx = joint["idx"]
                self.low_cmd.motor_cmd[motor_idx].q = positions[i]
                self.low_cmd.motor_cmd[motor_idx].dq = velocities[i]
                self.low_cmd.motor_cmd[motor_idx].tau = 0.0
                self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
                self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
                self.low_cmd.motor_cmd[motor_idx].weight = 1.0
            self.low_cmd_publisher.Write(self.low_cmd)
            cmd_idx += 1
            time.sleep(SLEEP_TIME)
            if cmd_idx  >= len(cmd_plan.position):
                print("Reached final position")
                print("Final Configuration", positions)
                print("Final Velocity", velocities)
                cmd_plan = None
        
        start_js = [self.dof_pos[joint["idx"]] for joint in controlled_j]
        cmd_plan = self.motion_gen.generate_plan(
            start_js=positions,
            goal_position=[ 0.39360079, -0.04169631, -0.0278361],
            goal_orientation=[0.848, -0.123, -0.463, 0.226]
        )
        print("Values from the subscriber:", start_js)
        print("First position:", cmd_plan[0].position.cpu().numpy())
        print("Joint names:", cmd_plan[0].joint_names)
        print("Order: ", [JOINT_PARAMETERS[name]["idx"] for name in cmd_plan[0].joint_names])
        input("Press ENTER to start second motion ...")
        
        print("Moving to second position")
        cmd_idx = 0
        while cmd_plan is not None:
            cmd_state = cmd_plan[cmd_idx]
            positions = cmd_state.position.cpu().numpy()
            velocities = cmd_state.velocity.cpu().numpy()
            if cmd_idx  >= len(cmd_plan.position) - 1:
                velocities = np.zeros_like(positions)
            for i, joint in enumerate(controlled_j):
                motor_idx = joint["idx"]
                self.low_cmd.motor_cmd[motor_idx].q = positions[i]
                self.low_cmd.motor_cmd[motor_idx].dq = velocities[i]
                self.low_cmd.motor_cmd[motor_idx].tau = 0.0
                self.low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
                self.low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
                self.low_cmd.motor_cmd[motor_idx].weight = 1.0

            self.low_cmd_publisher.Write(self.low_cmd)
            cmd_idx += 1
            time.sleep(SLEEP_TIME)
            if cmd_idx  >= len(cmd_plan.position):
                print("Reached final position")
                print("Final Configuration", positions)
                print("Final Velocity", velocities)
                cmd_plan = None      
            
        start_js = [self.dof_pos[joint["idx"]] for joint in controlled_j]
        print(start_js)
        return
    
    def __exit__(self, *args) -> None:
        self.cleanup()

def main():
    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)
    controller = Controller()
    motion_gen = Curobo_Motion_Gen()
    controller.set_motion_gen(motion_gen)
    controller.start_custom_mode()
    controller.control_manipulator([ 0.39360079, -0.04169631, -0.0278361], [0.9750, 0.1460, 0.1620, 0.0430])

if __name__ == "__main__":
    main()
