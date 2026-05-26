#!/usr/bin/env python3
"""ROS2 node that receives 7-DOF joint trajectories, computes forward kinematics
via Pinocchio, and executes the resulting EE trajectory using the Booster SDK
MoveHandEndEffector API.

Combines the joint trajectory subscription pattern from move_robot_node.py with
the EE execution pattern from b1_upper_body_custom_control.py.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import JointTrajectory

from booster_robotics_sdk_python import (
    B1HandIndex,
    B1LocoClient,
    ChannelFactory,
    GetModeResponse,
    Orientation,
    Position,
    Posture,
    RobotMode,
)

# Joint names in the URDF, ordered to match the SDK 7-DOF convention:
#   [ShoulderPitch, ShoulderRoll, ElbowPitch, ElbowYaw, WristPitch, WristYaw, HandRoll]
RIGHT_ARM_JOINT_NAMES = [
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Right_Wrist_Pitch",
    "Right_Wrist_Yaw",
    "Right_Hand_Roll",
]

LEFT_ARM_JOINT_NAMES = [
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Left_Wrist_Pitch",
    "Left_Wrist_Yaw",
    "Left_Hand_Roll",
]

EE_FRAME = {"right": "right_hand_link", "left": "left_hand_link"}
TRUNK_FRAME = "Trunk"

# Fixed rotation offset between the URDF's right_hand_link frame and the
# firmware's internal EE frame used by MoveHandEndEffector.
# Determined empirically: the firmware EE frame is rotated such that
# X->Z, Y->-X, Z->-Y relative to right_hand_link.
_EE_FRAME_OFFSET_INV = np.array([[ 0,  0,  1],
                                 [-1,  0,  0],
                                 [ 0, -1,  0]], dtype=float)

ROBOT_MODE_NAMES = {
    RobotMode.kUnknown: "kUnknown",
    RobotMode.kDamping: "kDamping",
    RobotMode.kPrepare: "kPrepare",
    RobotMode.kWalking: "kWalking",
    RobotMode.kCustom: "kCustom",
}


def robot_mode_string(mode: RobotMode) -> str:
    return ROBOT_MODE_NAMES.get(mode, "?(invalid)")


class JointToEENode(Node):
    def __init__(self, args):
        super().__init__("joint_to_ee_node")

        self._duration_ms = args.duration_ms
        self._num_segments = args.num_segments
        self._downsample = max(1, args.downsample)
        self._max_poses = args.max_poses
        self._hand = args.hand
        self._hand_index = (
            B1HandIndex.kRightHand if args.hand == "right" else B1HandIndex.kLeftHand
        )
        self._execution_lock = threading.Lock()
        self._executing = False

        # ---- Pinocchio model ----
        urdf_path = args.urdf
        if not os.path.isabs(urdf_path):
            urdf_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", urdf_path
            )
        urdf_path = os.path.abspath(urdf_path)
        self.get_logger().info(f"Loading URDF: {urdf_path}")

        self._pin_model = pin.buildModelFromUrdf(urdf_path)
        self._pin_data = self._pin_model.createData()

        # Map 7-DOF arm joint names -> Pinocchio joint indices (q indices)
        joint_names = (
            RIGHT_ARM_JOINT_NAMES if args.hand == "right" else LEFT_ARM_JOINT_NAMES
        )
        self._arm_q_indices = []
        for jname in joint_names:
            jid = self._pin_model.getJointId(jname)
            if jid >= self._pin_model.njoints:
                raise RuntimeError(f"Joint '{jname}' not found in URDF")
            idx_q = self._pin_model.joints[jid].idx_q
            self._arm_q_indices.append(idx_q)
        self.get_logger().info(
            f"Arm q-indices ({args.hand}): {self._arm_q_indices}"
        )

        # EE frame id
        ee_frame_name = EE_FRAME[args.hand]
        self._ee_frame_id = self._pin_model.getFrameId(ee_frame_name)
        if self._ee_frame_id >= self._pin_model.nframes:
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in URDF")

        # Trunk frame id — we compute EE pose relative to Trunk
        self._trunk_frame_id = self._pin_model.getFrameId(TRUNK_FRAME)
        if self._trunk_frame_id >= self._pin_model.nframes:
            raise RuntimeError(f"Frame '{TRUNK_FRAME}' not found in URDF")

        # ---- Booster SDK ----
        self.get_logger().info("Initializing Booster SDK...")
        ChannelFactory.Instance().Init(0, args.network_interface)

        self.client = B1LocoClient()
        self.client.Init()

        mode_resp = GetModeResponse()
        mode_ret = self.client.GetMode(mode_resp)
        if mode_ret == 0:
            self.get_logger().info(
                f"GetMode returned {mode_ret}, current mode: "
                f"{int(mode_resp.mode)} ({robot_mode_string(mode_resp.mode)})"
            )
        else:
            self.get_logger().warning(
                f"GetMode failed ({mode_ret}); continuing with UpperBodyCustomControl."
            )

        ret = self.client.UpperBodyCustomControl(True)
        if ret != 0:
            raise RuntimeError(f"UpperBodyCustomControl(true) failed with {ret}")
        self.get_logger().info("UpperBodyCustomControl(true) ok")

        # Firmware needs a short transition window before hand EE commands are accepted.
        time.sleep(1.0)

        hcm_ret = self.client.SwitchHandEndEffectorControlMode(True)
        if hcm_ret == 0:
            self.get_logger().info("SwitchHandEndEffectorControlMode(true) ok")
        elif hcm_ret == 400:
            self.get_logger().warning(
                "SwitchHandEndEffectorControlMode(true) -> 400 (bad request); "
                "may already be active — continuing."
            )
        else:
            raise RuntimeError(
                f"SwitchHandEndEffectorControlMode(true) failed with {hcm_ret}"
            )

        # ---- ROS2 publishers ----
        ee_traj_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._ee_traj_pub = self.create_publisher(
            MarkerArray, "/joint_to_ee/ee_trajectory", ee_traj_qos
        )

        # ---- ROS2 subscription ----
        trajectory_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            JointTrajectory, args.topic, self._on_trajectory, trajectory_qos
        )
        self.get_logger().info(
            f"Subscribed to {args.topic} (JointTrajectory), "
            f"hand={args.hand}, duration={self._duration_ms} ms, "
            f"downsample={self._downsample}, max_poses={self._max_poses}."
        )

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def _fk(self, arm_joint_positions: np.ndarray) -> pin.SE3:
        """Compute EE pose in Trunk frame given 7 arm joint positions."""
        q = pin.neutral(self._pin_model)
        for i, idx_q in enumerate(self._arm_q_indices):
            q[idx_q] = arm_joint_positions[i]

        pin.forwardKinematics(self._pin_model, self._pin_data, q)
        pin.updateFramePlacements(self._pin_model, self._pin_data)

        oMee = self._pin_data.oMf[self._ee_frame_id]
        oMtrunk = self._pin_data.oMf[self._trunk_frame_id]
        # EE pose relative to Trunk
        trunkMee = oMtrunk.actInv(oMee)
        return trunkMee

    @staticmethod
    def _se3_to_posture(se3: pin.SE3) -> Posture:
        """Convert an SE3 to Booster SDK Posture, correcting for the firmware's
        EE frame offset: R_corrected = R_fk * R_offset_inv."""
        p = se3.translation
        R_corrected = se3.rotation @ _EE_FRAME_OFFSET_INV
        rpy = pin.rpy.matrixToRpy(R_corrected)
        tar = Posture()
        tar.position = Position(float(p[0]), float(p[1]), float(p[2]))
        tar.orientation = Orientation(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        return tar

    def _build_ee_markers(self, ee_poses: list[pin.SE3], stamp) -> MarkerArray:
        """Build a MarkerArray: spheres at each EE pose + a line strip connecting them."""
        ma = MarkerArray()

        # Line strip through all positions
        line = Marker()
        line.header.frame_id = TRUNK_FRAME
        line.header.stamp = stamp
        line.ns = "ee_trajectory"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.005
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0
        for se3 in ee_poses:
            pt = Point(
                x=float(se3.translation[0]),
                y=float(se3.translation[1]),
                z=float(se3.translation[2]),
            )
            line.points.append(pt)
        ma.markers.append(line)

        # Spheres at each waypoint (single SPHERE_LIST marker)
        spheres = Marker()
        spheres.header.frame_id = TRUNK_FRAME
        spheres.header.stamp = stamp
        spheres.ns = "ee_waypoints"
        spheres.id = 0
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.pose.orientation.w = 1.0
        spheres.scale.x = 0.01
        spheres.scale.y = 0.01
        spheres.scale.z = 0.01
        spheres.color.r = 1.0
        spheres.color.g = 0.0
        spheres.color.b = 0.0
        spheres.color.a = 1.0
        spheres.points = list(line.points)
        ma.markers.append(spheres)

        return ma

    # ------------------------------------------------------------------
    # Trajectory handling
    # ------------------------------------------------------------------

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning("Received empty JointTrajectory; nothing to do.")
            return

        with self._execution_lock:
            if self._executing:
                self.get_logger().warning(
                    f"Dropping JointTrajectory with {len(msg.points)} points "
                    "while another trajectory is executing."
                )
                return
            self._executing = True

        worker = threading.Thread(
            target=self._execute_trajectory,
            args=(msg,),
            daemon=True,
        )
        worker.start()

    def _execute_trajectory(self, msg: JointTrajectory) -> None:
        try:
            # Re-enable hand end effector control mode for this trajectory
            hcm_ret = self.client.SwitchHandEndEffectorControlMode(True)
            if hcm_ret != 0 and hcm_ret != 400:
                self.get_logger().error(
                    f"Failed to enable SwitchHandEndEffectorControlMode: {hcm_ret}"
                )
                return

            points = list(msg.points)

            # Validate DOF
            if points and len(points[0].positions) != 7:
                self.get_logger().error(
                    f"Expected 7-DOF trajectory, got {len(points[0].positions)}-DOF. Aborting."
                )
                return

            if self._max_poses is not None and self._max_poses < len(points):
                points = points[: self._max_poses]
                self.get_logger().info(
                    f"Truncated to {len(points)} poses (--max-poses {self._max_poses})."
                )

            n_orig = len(points)

            # Downsample: keep every Nth point, always include first and last.
            if self._downsample > 1:
                ds_indices = list(range(0, len(points), self._downsample))
                if ds_indices[-1] != len(points) - 1:
                    ds_indices.append(len(points) - 1)
                points = [points[i] for i in ds_indices]
                self.get_logger().info(
                    f"Downsampled {n_orig} -> {len(points)} points (step={self._downsample})."
                )

            n = len(points)

            # Compute all FK poses and publish as MarkerArray for visualization
            ee_poses = []
            for point in points:
                arm_q = np.array(point.positions[:7], dtype=float)
                se3 = self._fk(arm_q)
                ee_poses.append(se3)
            stamp = self.get_clock().now().to_msg()
            self._ee_traj_pub.publish(self._build_ee_markers(ee_poses, stamp))
            self.get_logger().info(
                f"Published {len(ee_poses)} EE markers to /joint_to_ee/ee_trajectory"
            )

            # Split trajectory into N evenly-spaced segments
            num_seg = min(self._num_segments, n)
            if num_seg <= 1:
                seg_indices = [n - 1]
            else:
                seg_indices = [round(i * (n - 1) / (num_seg - 1)) for i in range(num_seg)]
            n_seg = len(seg_indices)
            segment_duration_ms = self._duration_ms // n_seg

            self.get_logger().info(
                f"Sending {n_seg} segments, {segment_duration_ms} ms each"
            )

            for seg_i, pose_idx in enumerate(seg_indices):
                posture = self._se3_to_posture(ee_poses[pose_idx])
                p = posture.position
                o = posture.orientation
                self.get_logger().info(
                    f"Segment {seg_i + 1}/{n_seg} (pose {pose_idx + 1}/{n}): "
                    f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                    f"rpy=({o.roll:.4f}, {o.pitch:.4f}, {o.yaw:.4f}), "
                    f"duration={segment_duration_ms} ms"
                )

                ret = self.client.MoveHandEndEffector(
                    posture, segment_duration_ms, self._hand_index
                )

                if ret != 0:
                    self.get_logger().error(
                        f"MoveHandEndEffector failed at segment {seg_i + 1}/{n_seg} with code {ret}"
                    )
                else:
                    self.get_logger().info(f"Segment {seg_i + 1}/{n_seg} sent")

                # Wait for segment to complete before sending next
                if seg_i < n_seg - 1:
                    time.sleep(segment_duration_ms / 1000.0)

            self.get_logger().info("Trajectory execution complete.")

            # Wait 2s, then move 8cm in EE's local X direction
            time.sleep(2.0)
            last_se3 = ee_poses[-1]
            ee_x_in_trunk = last_se3.rotation @ np.array([0.0, -0.090, 0.0])
            push_se3 = pin.SE3(last_se3.rotation, last_se3.translation + ee_x_in_trunk)
            push_posture = self._se3_to_posture(push_se3)

            p = push_posture.position
            o = push_posture.orientation
            self.get_logger().info(
                f"Pushing 8cm in EE Y: "
                f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                f"rpy=({o.roll:.4f}, {o.pitch:.4f}, {o.yaw:.4f}), "
                f"duration=2000 ms"
            )
            ret = self.client.MoveHandEndEffector(push_posture, 2000, self._hand_index)
            if ret != 0:
                self.get_logger().error(f"Push move failed with code {ret}")
            else:
                self.get_logger().info("Push move sent")
        finally:
            # Disable hand end effector control mode after trajectory completes
            hcm_ret = self.client.SwitchHandEndEffectorControlMode(False)
            if hcm_ret != 0:
                self.get_logger().warning(
                    f"Failed to disable SwitchHandEndEffectorControlMode: {hcm_ret}"
                )

            with self._execution_lock:
                self._executing = False


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Receive 7-DOF joint trajectories, compute FK with Pinocchio, "
            "and execute via MoveHandEndEffector."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--network_interface",
        type=str,
        required=True,
        help="Network interface for robot communication (e.g. eth0, 127.0.0.1)",
    )
    parser.add_argument(
        "--hand",
        type=str,
        choices=("right", "left"),
        default="right",
        help="Which hand to command (default: right)",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default="robot.urdf",
        help="Path to URDF file (default: robot.urdf relative to repo root)",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/planning/trajectory",
        help="JointTrajectory topic to subscribe to (default: /planning/trajectory)",
    )
    parser.add_argument(
        "--duration-ms",
        type=int,
        default=5000,
        help="Total move duration in ms, split evenly across segments (default: 5000)",
    )
    parser.add_argument(
        "--num-segments",
        type=int,
        default=3,
        help="Number of evenly-spaced waypoints to send (default: 3)",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=1,
        help="Keep every Nth waypoint for FK and visualization "
             "(first and last always included). (default: 1)",
    )
    parser.add_argument(
        "--max-poses",
        type=int,
        default=None,
        help="Execute only the first N waypoints (default: all)",
    )

    # Strip --ros-args so argparse doesn't choke on them.
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        split = argv.index("--ros-args")
        args = parser.parse_args(argv[:split])
    else:
        args = parser.parse_args(argv)

    rclpy.init()
    node = None
    try:
        node = JointToEENode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(f"Startup failed: {ex}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
