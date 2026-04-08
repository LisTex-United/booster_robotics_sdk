import time
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

# Booster SDK imports
from booster_robotics_sdk_python import (
    B1LocoClient, 
    ChannelFactory, 
    RobotMode, 
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    MotorCmd
)


class BoosterExecutorNode(Node):
    """ROS2 node that receives trajectories and executes them on the robot."""
    
    # Head joint indices
    HEAD_YAW_INDEX = 0    # kHeadYaw
    HEAD_PITCH_INDEX = 1  # kHeadPitch
    
    # Left arm joint indices 
    LEFT_ARM_JOINT_INDICES = [
        2,   # kLeftShoulderPitch
        3,   # kLeftShoulderRoll
        4,   # kLeftElbowPitch
        5,   # kLeftElbowYaw
        6,   # kLeftWristPitch
        7,   # kLeftWristYaw
        8    # kLeftHandRoll
    ]
    
    # Right arm joint indices in the Booster SDK motor array
    RIGHT_ARM_JOINT_INDICES = [
        9,   # kRightShoulderPitch
        10,  # kRightShoulderRoll
        11,  # kRightElbowPitch
        12,  # kRightElbowYaw
        13,  # kRightWristPitch
        14,  # kRightWristYaw
        15   # kRightHandRoll
    ]
    
    # Waist joint index (inferred as following upper body)
    WAIST_JOINT_INDEX = 16
    
    TOTAL_MOTORS = 29  # kJointCnt7DofArm
    IGNORE_JOINT_INDICES = []
    
    def __init__(self, args):
        super().__init__('booster_executor_node')
        
        self.args = args
        self.current_trajectory = None
        self.is_executing = False
        
        # Robot state tracking
        self.state_received = False
        self.current_joint_states = [0.0] * self.TOTAL_MOTORS
        
        # Create subscriber for trajectory commands
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            # '/booster/right_arm/joint_trajectory',
            "/planning/trajectory",
            self.trajectory_callback,
            10
        )
        
        self.get_logger().info("Initializing Booster SDK...")
        self._init_sdk()
        self.get_logger().info("Booster SDK ready!")
        
        # Wait for user confirmation before moving robot
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("READY TO INITIALIZE ROBOT")
        self.get_logger().info("="*60)
        input("\n[PRESS ENTER] to initialize robot to retract pose...\n")
        
        # Initialize robot to retract pose
        self.get_logger().info("Moving to retract pose...")
        self._initialize_robot()
        self.get_logger().info("Robot initialized and ready to receive trajectories!")
        
        # Now enter continuous command loop (SDK needs continuous commands!)
        # self._run_continuous_loop()
        
    def _state_callback(self, msg):
        """Callback for robot state."""
        self.state_received = True
        
        # Store serial motor states (where arm joints live)
        # Note: We assume the indices match 1:1 with what we expect for the 29 motors
        if len(msg.motor_state_serial) >= self.TOTAL_MOTORS:
             for i in range(self.TOTAL_MOTORS):
                 self.current_joint_states[i] = msg.motor_state_serial[i].q

    def _is_ignored_joint(self, joint_index):
        """Return True if this joint should be excluded from init/retract targeting."""
        return joint_index in self.IGNORE_JOINT_INDICES
        
    def _init_sdk(self):
        """Initialize Booster SDK."""
        # Initialize SDK - MUST follow exact order
        ChannelFactory.Instance().Init(0, self.args.network_interface)
        
        # Create publisher and message BEFORE InitChannel
        self.publisher = B1LowCmdPublisher()
        self.msg = LowCmd()
        
        # Create and init client
        self.client = B1LocoClient()
        self.client.Init()
        
        # Init state subscriber
        self.state_subscriber = B1LowStateSubscriber(self._state_callback)
        self.state_subscriber.InitChannel()
        
        # Init publisher channel AFTER creating LowCmd
        self.publisher.InitChannel()
        
        # Control parameters
        self.weight = 0.0
        self.weight_rate = 0.2
        self.tau_ff = 0.0
        self.control_dt = 1.0 / self.args.control_freq
        
        # Initial positions for head
        self.head_yaw_init = 0.0      # Initial position for head yaw
        self.head_pitch_init = 0.0    # Initial position for head pitch (0 deg)
        
        # Initial positions for BOTH arms
        # self.left_arm_init = [0.5, -1.0, 0.0, -1.4, 0.0, 0.0, 0.0]
        # self.right_arm_init = [0.5, 1.0, 0.0, 1.4, 0.0, 0.0, 0.0]
        self.left_arm_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_arm_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.waist_init = 0.0  # Initial position for waist
        self.current_waist_pos = self.waist_init
        self.current_pos = self.right_arm_init.copy()
        
        # Create motor command list
        self.motor_cmd_list = []
        for i in range(self.TOTAL_MOTORS):
            motor_cmd = MotorCmd()
            self.motor_cmd_list.append(motor_cmd)
                
    def _initialize_robot(self):
        """Initialize robot: change to custom mode and move to retract position."""
        self.get_logger().info("Changing to CUSTOM mode...")
        ret = self.client.ChangeMode(RobotMode.kCustom)
        self.get_logger().info(f"Mode change returned: {ret}")
        
        # Give time for mode to actually switch
        time.sleep(1.0)
        
        # This becomes optional because we are already doing this in run_continuous_loop() and no matter, it always jumps to the position in 1 step superfast.
        init_time = self.args.init_time
        init_time_steps = int(init_time / self.control_dt)
        weight_margin = self.weight_rate * self.control_dt
        
        # Wait for valid state
        self.get_logger().info("Waiting for initial robot state...")
        timeout = 5.0
        start_wait = time.time()
        while not self.state_received and (time.time() - start_wait) < timeout:
            time.sleep(0.1)
            
        if not self.state_received:
            self.get_logger().error("Failed to receive robot state! Aborting initialization.")
            return

        # Capture start positions from current state
        start_joints = list(self.current_joint_states)
        
        # Prepare start lists for specific groups
        left_arm_start = [start_joints[i] for i in self.LEFT_ARM_JOINT_INDICES]
        right_arm_start = [start_joints[i] for i in self.RIGHT_ARM_JOINT_INDICES]
        waist_start = start_joints[self.WAIST_JOINT_INDEX]
        head_yaw_start = start_joints[self.HEAD_YAW_INDEX]
        head_pitch_start = start_joints[self.HEAD_PITCH_INDEX]

        self.get_logger().info(f"Waist Start State: {waist_start:.4f}")
        self.get_logger().info(f"Interpolating from current pose to retract pose over {init_time}s...")
        
        for i in range(init_time_steps):
            # Calculate interpolation factor (0.0 to 1.0)
            alpha = float(i) / float(init_time_steps)
            
            # Ramp up weight smoothly
            self.weight += weight_margin
            self.weight = max(0.0, min(self.weight, 0.5))
            
            if i % 50 == 0:
                print(f"Weight: {self.weight:.4f}, Progress: {alpha:.2f}")
            
            # Helper to interpolate
            def lerp(start, end, t):
                return start + (end - start) * t
            
            # Set motor commands for LEFT arm (interpolated)
            for j, motor_idx in enumerate(self.LEFT_ARM_JOINT_INDICES):
                if self._is_ignored_joint(motor_idx):
                    continue
                target_q = lerp(left_arm_start[j], self.left_arm_init[j], alpha)
                self.motor_cmd_list[motor_idx].q = target_q
                self.motor_cmd_list[motor_idx].dq = 0.0
                self.motor_cmd_list[motor_idx].kp = self.args.kp
                self.motor_cmd_list[motor_idx].kd = self.args.kd
                self.motor_cmd_list[motor_idx].tau = self.tau_ff
                self.motor_cmd_list[motor_idx].weight = self.weight
            
            # Set motor commands for RIGHT arm (interpolated)
            for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                if self._is_ignored_joint(motor_idx):
                    continue
                target_q = lerp(right_arm_start[j], self.right_arm_init[j], alpha)
                self.motor_cmd_list[motor_idx].q = target_q
                self.motor_cmd_list[motor_idx].dq = 0.0
                self.motor_cmd_list[motor_idx].kp = self.args.kp
                self.motor_cmd_list[motor_idx].kd = self.args.kd
                self.motor_cmd_list[motor_idx].tau = self.tau_ff
                self.motor_cmd_list[motor_idx].weight = self.weight
            
            # Set motor command for WAIST (interpolated)
            target_waist = lerp(waist_start, self.waist_init, alpha)
            if not self._is_ignored_joint(self.WAIST_JOINT_INDEX):
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = target_waist
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = 0.0
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self.args.kp
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].weight = self.weight
            
            # Set motor commands for HEAD (interpolated)
            target_yaw = lerp(head_yaw_start, self.head_yaw_init, alpha)
            if not self._is_ignored_joint(self.HEAD_YAW_INDEX):
                self.motor_cmd_list[self.HEAD_YAW_INDEX].q = target_yaw
                self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self.args.kp
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff
            
            target_pitch = lerp(head_pitch_start, self.head_pitch_init, alpha)
            if not self._is_ignored_joint(self.HEAD_PITCH_INDEX):
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = target_pitch
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self.args.kp
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].tau = self.tau_ff
            
            # CRITICAL: Reassign motor_cmd EVERY iteration (matches working SDK example!)
            self.msg.motor_cmd = self.motor_cmd_list
            self.publisher.Write(self.msg)
            time.sleep(self.control_dt)
        
        self.get_logger().info(f"Initialization complete! (weight: {self.weight:.3f})")
    
    def _run_continuous_loop(self):
        """Continuous command loop (SDK requires this!) - handles trajectory execution inline"""
        self.get_logger().info("Entering continuous command loop...")
        try:
            while True:
                # Check if we have a trajectory to execute
                if self.current_trajectory is not None and not self.is_executing:
                    self._execute_trajectory_inline()
                else:
                    # Hold position - send commands for BOTH arms and waist
                    for j, motor_idx in enumerate(self.LEFT_ARM_JOINT_INDICES):
                        if self._is_ignored_joint(motor_idx):
                            continue
                        self.motor_cmd_list[motor_idx].q = self.left_arm_init[j]
                        self.motor_cmd_list[motor_idx].dq = 0.0
                        self.motor_cmd_list[motor_idx].kp = self.args.kp
                        self.motor_cmd_list[motor_idx].kd = self.args.kd
                        self.motor_cmd_list[motor_idx].tau = self.tau_ff
                    
                    for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                        self.motor_cmd_list[motor_idx].q = self.current_pos[j]
                        self.motor_cmd_list[motor_idx].dq = 0.0
                        self.motor_cmd_list[motor_idx].kp = self.args.kp
                        self.motor_cmd_list[motor_idx].kd = self.args.kd
                        self.motor_cmd_list[motor_idx].tau = self.tau_ff
                    
                    # Hold WAIST position
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = self.current_waist_pos
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = 0.0
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self.args.kp
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff
                    
                    # Hold HEAD position
                    if not self._is_ignored_joint(self.HEAD_YAW_INDEX):
                        self.motor_cmd_list[self.HEAD_YAW_INDEX].q = self.head_yaw_init
                        self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
                        self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self.args.kp
                        self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
                        self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff
                    
                    if not self._is_ignored_joint(self.HEAD_PITCH_INDEX):
                        self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = self.head_pitch_init
                        self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
                        self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self.args.kp
                        self.motor_cmd_list[self.HEAD_PITCH_INDEX].kd = self.args.kd
                        self.motor_cmd_list[self.HEAD_PITCH_INDEX].tau = self.tau_ff
                    
                    self.msg.motor_cmd = self.motor_cmd_list
                    self.publisher.Write(self.msg)
                    time.sleep(self.control_dt)
                    
                    # Process ROS callbacks to check for new trajectories
                    rclpy.spin_once(self, timeout_sec=0)
                    
        except KeyboardInterrupt:
            self.get_logger().info("Stopping continuous loop...")
    
    def trajectory_callback(self, msg):
        """Callback when new trajectory is received."""
        self.get_logger().info(f"Received trajectory with {len(msg.points)} waypoints")
        
        if self.is_executing or self.current_trajectory is not None:
            self.get_logger().warning("Already have a trajectory queued, ignoring new command")
            return
        
        # Store trajectory - will be executed in continuous loop
        self.current_trajectory = msg
    
    def _execute_trajectory_inline(self):
        """Execute the current trajectory inline in the continuous loop."""
        if self.current_trajectory is None:
            return
        
        self.is_executing = True
        trajectory = self.current_trajectory
        
        num_waypoints = len(trajectory.points)
        dt = 1.0 / self.args.control_freq
        
        self.get_logger().info(f"Executing trajectory: {num_waypoints} waypoints at {self.args.control_freq} Hz")
        
        try:
            for idx, point in enumerate(trajectory.points):
                start_time = time.time()
                
                # Get commanded position from planned trajectory
                cmd_position = np.array(point.positions)
                
                # Get commanded velocity if available, else 0
                cmd_velocity = np.zeros_like(cmd_position)
                if len(point.velocities) == len(cmd_position):
                     cmd_velocity = np.array(point.velocities)
                
                # Separate Waist and Arm commands
                target_waist_q = self.current_waist_pos # Default to current holding
                target_waist_dq = 0.0
                target_arm_q = []
                target_arm_dq = []
                
                if len(cmd_position) == 8:
                    # 8-DOF: Index 0 is Waist, 1-7 is Right Arm
                    target_waist_q = cmd_position[0]
                    target_waist_dq = cmd_velocity[0]
                    target_arm_q = cmd_position[1:]
                    target_arm_dq = cmd_velocity[1:]
                elif len(cmd_position) == 7:
                    # 7-DOF: All Right Arm (Backward compatibility)
                    target_arm_q = cmd_position
                    target_arm_dq = cmd_velocity
                else:
                    if idx == 0:
                        self.get_logger().error(f"Unexpected trajectory dimension: {len(cmd_position)}. Expected 8 (Waist+Arm) or 7 (Arm only).")
                    
                
                # Update current positions for holding after trajectory
                if len(target_arm_q) == 7:
                    self.current_pos = target_arm_q.copy()
                self.current_waist_pos = target_waist_q
                
                # Debug: print commanded positions on first iteration
                if idx == 0:
                    self.get_logger().info(f"First waypoint: Waist={target_waist_q:.3f} (Current: {self.current_waist_pos:.3f}), Arm={np.round(target_arm_q, 3).tolist()}")
                    self.get_logger().info(f"Waist Diff: {target_waist_q - self.current_waist_pos:.4f}")
                
                # Set motor commands for LEFT arm (HOLD retract pose)
                for j, motor_idx in enumerate(self.LEFT_ARM_JOINT_INDICES):
                    if self._is_ignored_joint(motor_idx):
                        continue
                    self.motor_cmd_list[motor_idx].q = self.left_arm_init[j]
                    self.motor_cmd_list[motor_idx].dq = 0.0
                    self.motor_cmd_list[motor_idx].kp = self.args.kp
                    self.motor_cmd_list[motor_idx].kd = self.args.kd
                    self.motor_cmd_list[motor_idx].tau = self.tau_ff
                
                # Set motor commands for RIGHT arm (FOLLOW trajectory)
                if len(target_arm_q) == 7:
                    for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                        self.motor_cmd_list[motor_idx].q = target_arm_q[j]
                        self.motor_cmd_list[motor_idx].dq = target_arm_dq[j]
                        self.motor_cmd_list[motor_idx].kp = self.args.kp
                        self.motor_cmd_list[motor_idx].kd = self.args.kd
                        self.motor_cmd_list[motor_idx].tau = self.tau_ff

                # Set motor command for WAIST
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = target_waist_q
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = target_waist_dq
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self.args.kp
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff
                
                # Set motor commands for HEAD (HOLD retract pose)
                if not self._is_ignored_joint(self.HEAD_YAW_INDEX):
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].q = self.head_yaw_init
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self.args.kp
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff
                
                if not self._is_ignored_joint(self.HEAD_PITCH_INDEX):
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = self.head_pitch_init
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self.args.kp
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].kd = self.args.kd
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].tau = self.tau_ff
                
                # CRITICAL: Reassign motor_cmd EVERY iteration!
                self.msg.motor_cmd = self.motor_cmd_list
                self.publisher.Write(self.msg)
                
                # Progress indicator
                if idx % 10 == 0:
                    progress = (idx / num_waypoints) * 100
                    self.get_logger().info(f"Progress: {progress:.1f}% ({idx}/{num_waypoints})")
                
                # Sleep to maintain control frequency
                elapsed = time.time() - start_time
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    self.get_logger().warning(f"Control loop running slow: {elapsed:.4f}s > {dt:.4f}s")
            
            self.get_logger().info("Trajectory execution complete!")
            
        except Exception as e:
            self.get_logger().error(f"Error during trajectory execution: {e}")
        finally:
            self.is_executing = False
            self.current_trajectory = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "network_interface",
        nargs="?",
        default=None,
        help="Network interface for robot communication (positional, e.g., eth0, enp0s1, or 127.0.0.1)",
    )
    parser.add_argument(
        "--network_interface",
        dest="network_interface_option",
        type=str,
        default=None,
        help="Network interface for robot communication (e.g., eth0, enp0s1, or 127.0.0.1)",
    )
    parser.add_argument(
        "--control_freq",
        type=float,
        default=50.0,
        help="Control frequency in Hz (default: 50.0)",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=55.0,
        help="Proportional gain for position control (default: 55.0)",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1.5,
        help="Derivative gain for velocity control (default: 1.5)",
    )
    parser.add_argument(
        "--init_time",
        type=float,
        default=5.0,
        help="Time to ramp up to initial position (seconds, default: 5.0)",
    )
    
    args = parser.parse_args()

    if args.network_interface is None:
        args.network_interface = args.network_interface_option

    if args.network_interface is None:
        parser.error("network_interface is required (use positional argument or --network_interface)")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create executor node - this will init, then enter continuous loop
    try:
        node = BoosterExecutorNode(args)
        # Node runs its own loop in _run_continuous_loop()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

