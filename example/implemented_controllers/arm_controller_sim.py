import time
from booster_robotics_sdk_python import ChannelFactory, B1LowCmdPublisher, LowCmd, LowCmdType, MotorCmd
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

args = parser.parse_args()

class Curobo_Motion_Gen:
    def __init__(self, cfg_path=args.robot_configs_path, robot_name=args.robot):
        robot_cfg = load_yaml(join_path(cfg_path, robot_name))["robot_cfg"]
        self.tensor_args = TensorDeviceType()
        #Motion Gen
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            WorldConfig(),
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            # num_ik_seeds=400,  # Increased from 200 for better IK success rate
            interpolation_dt=0.1,
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
   
   
def control_manipulator(motion_gen):  
    ChannelFactory.Instance().Init(0)
    channel_publisher = B1LowCmdPublisher()
    channel_publisher.InitChannel()
    motor_cmds = [MotorCmd() for _ in range(TOTAL_DOF)]
    controlled_j = [JOINT_PARAMETERS[name] for name in motion_gen.get_joint_names()]

    #cmd_plan = cmd_plan_full.get_ordered_joint_state(joint_names)
    low_cmd = LowCmd()
    low_cmd.cmd_type = LowCmdType.PARALLEL
    low_cmd.motor_cmd = motor_cmds
    for motor_idx in range(TOTAL_DOF):
        low_cmd.motor_cmd[motor_idx].q = 0.0
        low_cmd.motor_cmd[motor_idx].dq = 0.0
        low_cmd.motor_cmd[motor_idx].tau = 0.0
        low_cmd.motor_cmd[motor_idx].kp = 0.0
        low_cmd.motor_cmd[motor_idx].kd = 0.0
        low_cmd.motor_cmd[motor_idx].weight = 0.0

    cmd_plan = motion_gen.generate_plan(
        start_js=[0.45, -1.05, 0.0, -1.5, 0.0, 0.0, 0.0],
        goal_position=[0.187,0.51, 0.442],
        goal_orientation=[0.848, -0.123, -0.463, 0.226]
    )
    cmd_idx = 0
    while cmd_plan is not None:
        cmd_state = cmd_plan[cmd_idx]
        positions = cmd_state.position.cpu().numpy()
        velocities = cmd_state.velocity.cpu().numpy()
        for i, joint in enumerate(controlled_j):
            motor_idx = joint["idx"]
            low_cmd.motor_cmd[motor_idx].q = positions[i]
            low_cmd.motor_cmd[motor_idx].dq = velocities[i]
            low_cmd.motor_cmd[motor_idx].tau = 0.0
            low_cmd.motor_cmd[motor_idx].kp = joint["kp"]
            low_cmd.motor_cmd[motor_idx].kd = joint["kd"]
            low_cmd.motor_cmd[motor_idx].weight = 0.7

        channel_publisher.Write(low_cmd)
        cmd_idx += 1
        time.sleep(SLEEP_TIME)
        if cmd_idx  >= len(cmd_plan.position):
            print("Reached final position")
            print("Final Configuration", positions)
            cmd_plan = None
            
    return
def main():
    motion_gen = Curobo_Motion_Gen()
    control_manipulator(motion_gen)

if __name__ == "__main__":
    main()
