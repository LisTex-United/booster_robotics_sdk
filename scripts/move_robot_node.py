import time
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray

# Booster SDK imports
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


class BoosterExecutorNode(Node):
    """
    ROS2 node that receives 7-DOF right-arm trajectories and executes them.
    Expects each trajectory point as:
        [7dof right arm]
    """

    # Head joint indices
    HEAD_YAW_INDEX = 0  # kHeadYaw
    HEAD_PITCH_INDEX = 1  # kHeadPitch

    # Left arm joint indices
    LEFT_ARM_JOINT_INDICES = [
        2,  # kLeftShoulderPitch
        3,  # kLeftShoulderRoll
        4,  # kLeftElbowPitch
        5,  # kLeftElbowYaw
        6,  # kLeftWristPitch
        7,  # kLeftWristYaw
        8,  # kLeftHandRoll
    ]

    # Right arm joint indices in the Booster SDK motor array
    RIGHT_ARM_JOINT_INDICES = [
        9,  # kRightShoulderPitch
        10,  # kRightShoulderRoll
        11,  # kRightElbowPitch
        12,  # kRightElbowYaw
        13,  # kRightWristPitch
        14,  # kRightWristYaw
        15,  # kRightHandRoll
    ]

    # Specific joints for wrist/hand kp override
    RIGHT_WRIST_OVERRIDE_INDICES = [13, 14, 15]

    # Waist joint index (inferred as following upper body)
    WAIST_JOINT_INDEX = 16

    TOTAL_MOTORS = 29  # kJointCnt7DofArm
    SUPPORTED_TRAJ_DOFS = (7, 10)

    def __init__(self, args):
        super().__init__("booster_executor_node")

        self.args = args
        self.trajectory_dof = args.trajectory_dof
        self.current_trajectory = None
        self.is_executing = False
        self.current_head_yaw = 0.0
        self.current_head_pitch = 0.0
        self.target_head_yaw = 0.0
        self.target_head_pitch = 0.0

        # Robot state tracking
        self.state_received = False
        self.current_joint_states = [0.0] * self.TOTAL_MOTORS

        # Create subscriber for trajectory commands (7-DOF or 10-DOF, selected by CLI)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/planning/trajectory",
            self.trajectory_callback,
            10,
        )

        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("Initializing Booster SDK...")
        self.get_logger().info(
            f"Trajectory mode selected via CLI: {self.trajectory_dof}-DOF"
        )
        self._init_sdk()
        self.get_logger().info("Booster SDK ready!")

        # Wait for user confirmation before moving robot
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("READY TO INITIALIZE ROBOT")
        self.get_logger().info("=" * 60)
        input("\n[PRESS ENTER] to initialize robot to retract pose...\n")

        # Initialize robot to retract pose
        self.get_logger().info("Moving to retract pose...")
        self._initialize_robot()
        self.get_logger().info("Robot initialized and ready to receive trajectories!")

        # Now enter continuous command loop (SDK needs continuous commands!)
        self._run_continuous_loop()

    def _build_kp_profile(self):
        """Build per-motor kp profile from global kp + optional wrist override."""
        kp_profile = [self.args.kp] * self.TOTAL_MOTORS

        if self.args.kp_right_wrist is not None:
            for motor_idx in self.RIGHT_WRIST_OVERRIDE_INDICES:
                kp_profile[motor_idx] = self.args.kp_right_wrist

        return kp_profile

    def _kp(self, motor_idx):
        return self.kp_per_joint[motor_idx]

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def _step_toward(self, current, target, max_delta):
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        if delta > 0.0:
            return current + max_delta
        return current - max_delta

    def _state_callback(self, msg):
        """Callback for robot state."""
        self.state_received = True

        # Store serial motor states (where arm joints live)
        # Note: We assume the indices match 1:1 with what we expect for the 29 motors
        if len(msg.motor_state_serial) >= self.TOTAL_MOTORS:
            for i in range(self.TOTAL_MOTORS):
                self.current_joint_states[i] = msg.motor_state_serial[i].q

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
        self.kp_per_joint = self._build_kp_profile()
        self.head_max_delta = self.args.head_max_speed * self.control_dt

        # Initial positions for head
        self.head_yaw_init = 0.0  # Initial position for head yaw
        # self.head_pitch_init = 0.4892 # Initial position for head pitch (28.03 deg)
        self.head_pitch_init = 0.4
        self.current_head_yaw = self.head_yaw_init
        self.current_head_pitch = self.head_pitch_init
        self.target_head_yaw = self.head_yaw_init
        self.target_head_pitch = self.head_pitch_init
        # Initial positions for BOTH arms
        self.left_arm_init = [0.5, -1.0, 0.0, -1.4, 0.0, 0.0, 0.0]
        self.right_arm_init = [0.5, 1.0, 0.0, 1.4, 0.0, 0.0, 0.0]
        # self.left_arm_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.right_arm_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.waist_init = 0.0  # Initial position for waist
        self.current_waist_pos = self.waist_init
        self.current_pos = self.right_arm_init.copy()

        # Create motor command list
        self.motor_cmd_list = []
        for _ in range(self.TOTAL_MOTORS):
            motor_cmd = MotorCmd()
            self.motor_cmd_list.append(motor_cmd)

    def _initialize_robot(self):
        """Initialize robot: change to custom mode and move to retract position."""
        self.get_logger().info("Changing to CUSTOM mode...")
        ret = self.client.ChangeMode(RobotMode.kCustom)
        self.get_logger().info(f"Mode change returned: {ret}")

        mode_resp = GetModeResponse()
        ret = self.client.GetMode(mode_resp)
        self.get_logger().info(f"GetMode returned: {ret}, mode: {mode_resp.mode}")

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
        self.get_logger().info(
            f"Interpolating from current pose to retract pose over {init_time}s..."
        )

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
                target_q = lerp(left_arm_start[j], self.left_arm_init[j], alpha)
                self.motor_cmd_list[motor_idx].q = target_q
                self.motor_cmd_list[motor_idx].dq = 0.0
                self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                self.motor_cmd_list[motor_idx].kd = self.args.kd
                self.motor_cmd_list[motor_idx].tau = self.tau_ff
                self.motor_cmd_list[motor_idx].weight = self.weight

            # Set motor commands for RIGHT arm (interpolated)
            for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                target_q = lerp(right_arm_start[j], self.right_arm_init[j], alpha)
                self.motor_cmd_list[motor_idx].q = target_q
                self.motor_cmd_list[motor_idx].dq = 0.0
                self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                self.motor_cmd_list[motor_idx].kd = self.args.kd
                self.motor_cmd_list[motor_idx].tau = self.tau_ff
                self.motor_cmd_list[motor_idx].weight = self.weight

            # Set motor command for WAIST (interpolated)
            target_waist = lerp(waist_start, self.waist_init, alpha)
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = target_waist
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = 0.0
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self._kp(
                self.WAIST_JOINT_INDEX
            )
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff
            self.motor_cmd_list[self.WAIST_JOINT_INDEX].weight = self.weight

            # Set motor commands for HEAD (interpolated)
            target_yaw = lerp(head_yaw_start, self.head_yaw_init, alpha)
            self.motor_cmd_list[self.HEAD_YAW_INDEX].q = target_yaw
            self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
            self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self._kp(self.HEAD_YAW_INDEX)
            self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
            self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff

            target_pitch = lerp(head_pitch_start, self.head_pitch_init, alpha)
            self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = target_pitch
            self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
            self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self._kp(self.HEAD_PITCH_INDEX)
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
                    prev_yaw = self.current_head_yaw
                    prev_pitch = self.current_head_pitch
                    self.current_head_yaw = self._step_toward(
                        self.current_head_yaw,
                        self.target_head_yaw,
                        self.head_max_delta,
                    )
                    self.current_head_pitch = self._step_toward(
                        self.current_head_pitch,
                        self.target_head_pitch,
                        self.head_max_delta,
                    )
                    if prev_yaw != self.current_head_yaw or prev_pitch != self.current_head_pitch:
                        self.get_logger().debug(
                            f"[HOLD] Moving head toward: yaw={self.target_head_yaw:.3f}, pitch={self.target_head_pitch:.3f} | Current: yaw={self.current_head_yaw:.3f}, pitch={self.current_head_pitch:.3f}"
                        )

                    # Hold position - send commands for BOTH arms and waist
                    for j, motor_idx in enumerate(self.LEFT_ARM_JOINT_INDICES):
                        self.motor_cmd_list[motor_idx].q = self.left_arm_init[j]
                        self.motor_cmd_list[motor_idx].dq = 0.0
                        self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                        self.motor_cmd_list[motor_idx].kd = self.args.kd
                        self.motor_cmd_list[motor_idx].tau = self.tau_ff

                    for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                        self.motor_cmd_list[motor_idx].q = self.current_pos[j]
                        self.motor_cmd_list[motor_idx].dq = 0.0
                        self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                        self.motor_cmd_list[motor_idx].kd = self.args.kd
                        self.motor_cmd_list[motor_idx].tau = self.tau_ff

                    # Hold WAIST position
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = self.current_waist_pos
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = 0.0
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self._kp(
                        self.WAIST_JOINT_INDEX
                    )
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
                    self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff

                    # Hold HEAD position
                    self.get_logger().debug(
                        f"[HOLD] Commanding head: yaw={self.current_head_yaw:.3f}, pitch={self.current_head_pitch:.3f}"
                    )
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].q = self.current_head_yaw
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self._kp(self.HEAD_YAW_INDEX)
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
                    self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff

                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = self.current_head_pitch
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
                    self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self._kp(
                        self.HEAD_PITCH_INDEX
                    )
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
        """
        Callback when new trajectory is received.
        In 7-DOF mode, each point must be [7dof right arm].
        In 10-DOF mode, each point must be [waist, 7dof right arm, head_yaw, head_pitch].
        """
        self.get_logger().info(
            f"Received trajectory with {len(msg.points)} waypoints (mode={self.trajectory_dof}-DOF)"
        )
        if len(msg.points) > 0:
            last_point = msg.points[-1]
            self.get_logger().info(
                f"Last trajectory point: positions={last_point.positions}, "
                f"velocities={getattr(last_point, 'velocities', None)}"
            )

            if len(last_point.positions) != self.trajectory_dof:
                self.get_logger().error(
                    f"Trajectory point dimension mismatch: got {len(last_point.positions)}, "
                    f"expected {self.trajectory_dof} for selected mode"
                )
                return
        if self.is_executing or self.current_trajectory is not None:
            self.get_logger().warning(
                "Already have a trajectory queued, ignoring new command"
            )
            return
        # Store trajectory - will be executed in continuous loop
        self.current_trajectory = msg

    # Removed: head_command_callback. Head stays at hold target in this node.


    def _execute_trajectory_inline(self):
        """
        Execute the current trajectory inline in the continuous loop.
        In 7-DOF mode, each point is [7dof right arm].
        In 10-DOF mode, each point is [waist, 7dof right arm, head_yaw, head_pitch].
        """
        if self.current_trajectory is None:
            return

        self.is_executing = True
        trajectory = self.current_trajectory

        num_waypoints = len(trajectory.points)
        dt = 1.0 / self.args.control_freq

        self.get_logger().info(
            f"Executing {self.trajectory_dof}-DOF trajectory: "
            f"{num_waypoints} waypoints at {self.args.control_freq} Hz"
        )

        try:
            # Check if first point is too far from current position
            if num_waypoints > 0:
                first_point = trajectory.points[0]
                cmd_position = np.array(first_point.positions)
                if len(cmd_position) == self.trajectory_dof:
                    if self.trajectory_dof == 7:
                        # Compare only right arm state.
                        current = list(self.current_pos)
                    else:
                        # Compare waist + right arm + head state.
                        current = [
                            self.current_waist_pos,
                            *list(self.current_pos),
                            self.current_head_yaw,
                            self.current_head_pitch,
                        ]
                    dist = np.linalg.norm(cmd_position - np.array(current))
                    if dist > 1.0:  # Threshold in radians, adjust as needed
                        self.get_logger().error(f"First trajectory point is too far from current position (distance={dist:.3f}). Aborting trajectory execution.")
                        return
            last_head_yaw = None
            last_head_pitch = None
            for idx, point in enumerate(trajectory.points):
                start_time = time.time()

                # Get commanded position from planned trajectory
                cmd_position = np.array(point.positions)

                # Get commanded velocity if available, else 0
                cmd_velocity = np.zeros_like(cmd_position)
                if len(point.velocities) == len(cmd_position):
                    cmd_velocity = np.array(point.velocities)

                if len(cmd_position) != self.trajectory_dof:
                    if idx == 0:
                        self.get_logger().error(
                            f"Unexpected trajectory dimension: {len(cmd_position)}. "
                            f"Expected {self.trajectory_dof}."
                        )
                    continue

                if self.trajectory_dof == 7:
                    # Parse 7-DOF right-arm input
                    target_waist_q = self.current_waist_pos
                    target_waist_dq = 0.0
                    target_arm_q = cmd_position
                    target_arm_dq = cmd_velocity
                    target_head_yaw = self.current_head_yaw
                    target_head_pitch = self.current_head_pitch
                else:
                    # Parse 10-DOF input
                    target_waist_q = cmd_position[0]
                    target_waist_dq = cmd_velocity[0]
                    target_arm_q = cmd_position[1:8]
                    target_arm_dq = cmd_velocity[1:8]
                    target_head_yaw = cmd_position[8]
                    target_head_pitch = cmd_position[9]

                # Update current positions for holding after trajectory
                self.current_pos = target_arm_q.copy()
                if self.trajectory_dof == 10:
                    self.current_waist_pos = target_waist_q
                    self.current_head_yaw = target_head_yaw
                    self.current_head_pitch = target_head_pitch
                    last_head_yaw = target_head_yaw
                    last_head_pitch = target_head_pitch
                    self.get_logger().debug(
                        f"[TRAJ] Commanding head: yaw={target_head_yaw:.3f}, pitch={target_head_pitch:.3f} (waypoint {idx+1}/{num_waypoints})"
                    )
                else:
                    self.get_logger().debug(
                        f"[TRAJ] Commanding right arm waypoint {idx+1}/{num_waypoints}: {np.round(target_arm_q, 3).tolist()}"
                    )

                # Debug: print commanded positions on first iteration
                if idx == 0:
                    if self.trajectory_dof == 10:
                        self.get_logger().info(
                            f"First waypoint: Waist={target_waist_q:.3f} (Current: {self.current_waist_pos:.3f}), "
                            f"Arm={np.round(target_arm_q, 3).tolist()}, "
                            f"HeadYaw={target_head_yaw:.3f}, HeadPitch={target_head_pitch:.3f}"
                        )
                        self.get_logger().info(
                            f"Waist Diff: {target_waist_q - self.current_waist_pos:.4f}"
                        )
                    else:
                        self.get_logger().info(
                            f"First waypoint right arm={np.round(target_arm_q, 3).tolist()}"
                        )

                # Set motor commands for LEFT arm (HOLD retract pose)
                for j, motor_idx in enumerate(self.LEFT_ARM_JOINT_INDICES):
                    self.motor_cmd_list[motor_idx].q = self.left_arm_init[j]
                    self.motor_cmd_list[motor_idx].dq = 0.0
                    self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                    self.motor_cmd_list[motor_idx].kd = self.args.kd
                    self.motor_cmd_list[motor_idx].tau = self.tau_ff

                # Set motor commands for RIGHT arm (FOLLOW trajectory)
                for j, motor_idx in enumerate(self.RIGHT_ARM_JOINT_INDICES):
                    self.motor_cmd_list[motor_idx].q = target_arm_q[j]
                    self.motor_cmd_list[motor_idx].dq = target_arm_dq[j]
                    self.motor_cmd_list[motor_idx].kp = self._kp(motor_idx)
                    self.motor_cmd_list[motor_idx].kd = self.args.kd
                    self.motor_cmd_list[motor_idx].tau = self.tau_ff

                # Set motor command for WAIST
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].q = target_waist_q
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].dq = target_waist_dq
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kp = self._kp(
                    self.WAIST_JOINT_INDEX
                )
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.WAIST_JOINT_INDEX].tau = self.tau_ff

                # Set motor commands for HEAD
                self.motor_cmd_list[self.HEAD_YAW_INDEX].q = target_head_yaw
                self.motor_cmd_list[self.HEAD_YAW_INDEX].dq = 0.0
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kp = self._kp(self.HEAD_YAW_INDEX)
                self.motor_cmd_list[self.HEAD_YAW_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.HEAD_YAW_INDEX].tau = self.tau_ff

                self.motor_cmd_list[self.HEAD_PITCH_INDEX].q = target_head_pitch
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].dq = 0.0
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kp = self._kp(
                    self.HEAD_PITCH_INDEX
                )
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].kd = self.args.kd
                self.motor_cmd_list[self.HEAD_PITCH_INDEX].tau = self.tau_ff

                # CRITICAL: Reassign motor_cmd EVERY iteration!
                self.msg.motor_cmd = self.motor_cmd_list
                self.publisher.Write(self.msg)

                # Progress indicator
                if idx % 10 == 0:
                    progress = (idx / num_waypoints) * 100
                    self.get_logger().info(
                        f"Progress: {progress:.1f}% ({idx}/{num_waypoints})"
                    )

                # Sleep to maintain control frequency
                elapsed = time.time() - start_time
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    self.get_logger().warning(
                        f"Control loop running slow: {elapsed:.4f}s > {dt:.4f}s"
                    )

            # After 10-DOF trajectory, keep hold targets at last commanded head pose.
            if (
                self.trajectory_dof == 10
                and last_head_yaw is not None
                and last_head_pitch is not None
            ):
                self.target_head_yaw = last_head_yaw
                self.target_head_pitch = last_head_pitch
                self.get_logger().info(
                    f"Updated hold targets: head_yaw={last_head_yaw:.3f}, "
                    f"head_pitch={last_head_pitch:.3f}"
                )

            self.get_logger().info("Trajectory execution complete!")

        except Exception as e:
            self.get_logger().error(f"Error during trajectory execution: {e}")
        finally:
            self.is_executing = False
            self.current_trajectory = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--trajectory_dof",
        type=int,
        default=7,
        choices=[7, 10],
        help="Incoming trajectory dimensionality: 7 (right arm only) or 10 (waist+arm+head)",
    )
    parser.add_argument(
        "--network_interface",
        type=str,
        required=True,
        help="Network interface for robot communication (e.g., eth0, enp0s1)",
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
        default=150.0,
        help="Global proportional gain for all joints",
    )
    parser.add_argument(
        "--kp_right_wrist",
        type=float,
        default=120,
        help=(
            "Override kp for kRightWristPitch, kRightWristYaw, "
            "kRightHandRoll (motor indices 13,14,15)"
        ),
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1.2,
        help="Derivative gain for velocity control (default: 1.2)",
    )
    parser.add_argument(
        "--init_time",
        type=float,
        default=5.0,
        help="Time to ramp up to initial position (seconds, default: 5.0)",
    )
    parser.add_argument(
        "--head_max_speed",
        type=float,
        default=0.25,
        help="Max head motion speed in rad/s for command smoothing (default: 0.25)",
    )
    parser.add_argument(
        "--head_yaw_min",
        type=float,
        default=-1.2,
        help="Minimum commanded head yaw (rad)",
    )
    parser.add_argument(
        "--head_yaw_max",
        type=float,
        default=1.2,
        help="Maximum commanded head yaw (rad)",
    )
    parser.add_argument(
        "--head_pitch_min",
        type=float,
        default=-0.8,
        help="Minimum commanded head pitch (rad)",
    )
    parser.add_argument(
        "--head_pitch_max",
        type=float,
        default=0.8,
        help="Maximum commanded head pitch (rad)",
    )

    args = parser.parse_args()

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
