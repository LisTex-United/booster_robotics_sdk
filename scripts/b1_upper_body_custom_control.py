#!/usr/bin/env python3
"""Python port of example/high_level/b1_upper_body_custom_control.cpp

Also supports subscribing to a PoseArray trajectory topic (--topic) to command
the hand end-effector through a sequence of poses.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

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

ROBOT_MODE_NAMES = {
    RobotMode.kUnknown: "kUnknown",
    RobotMode.kDamping: "kDamping",
    RobotMode.kPrepare: "kPrepare",
    RobotMode.kWalking: "kWalking",
    RobotMode.kCustom: "kCustom",
}

_TRUNK_FRAME = "Trunk"


def _euler_from_quaternion(x: float, y: float, z: float, w: float):
    """Quaternion (x,y,z,w) -> (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def robot_mode_string(mode: RobotMode) -> str:
    return ROBOT_MODE_NAMES.get(mode, "?(invalid)")


# ---------------------------------------------------------------------------
# Core: faithful port of the C++ example (no ROS2 required)
# ---------------------------------------------------------------------------

def run_core(network_interface: str, start: bool) -> int:
    """Enable/disable UpperBodyCustomControl. No movement commands."""
    ChannelFactory.Instance().Init(0, network_interface)

    client = B1LocoClient()
    client.Init()

    mode_resp = GetModeResponse()
    mode_ret = client.GetMode(mode_resp)
    print(
        f"GetMode returned {mode_ret}, current mode: "
        f"{int(mode_resp.mode)} ({robot_mode_string(mode_resp.mode)})"
    )
    if mode_ret != 0:
        print("Warning: GetMode failed; continuing with UpperBodyCustomControl anyway.", file=sys.stderr)

    ret = client.UpperBodyCustomControl(start)
    print(
        f"UpperBodyCustomControl({str(start).lower()}) returned {ret}"
        f" {'(ok)' if ret == 0 else '(error)'}"
    )

    return 0 if ret == 0 else 1


# ---------------------------------------------------------------------------
# ROS2 trajectory mode: subscribe to PoseArray and execute sequentially
# ---------------------------------------------------------------------------

def run_trajectory(network_interface: str, topic: str, hand: str, duration_ms: int, downsample: int = 1, max_poses: int | None = None) -> int:
    import rclpy
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    trajectory_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    class UpperBodyTrajectoryNode(Node):
        def __init__(self):
            super().__init__("b1_upper_body_custom_control")

            self._duration_ms = duration_ms
            self._downsample = max(1, downsample)
            self._max_poses = max_poses
            self._hand_index = (
                B1HandIndex.kRightHand if hand == "right" else B1HandIndex.kLeftHand
            )
            self._execution_lock = threading.Lock()
            self._executing = False

            self._tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

            self.get_logger().info("Initializing Booster SDK...")
            ChannelFactory.Instance().Init(0, network_interface)
            self.client = B1LocoClient()
            self.client.Init()

            self._move_api = self.client.MoveHandEndEffector
            self._move_api_name = "MoveHandEndEffector"

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
                    "may already be active after UpperBodyCustomControl — continuing."
                )
            else:
                raise RuntimeError(
                    f"SwitchHandEndEffectorControlMode(true) failed with {hcm_ret}"
                )

            self.create_subscription(PoseArray, topic, self._on_trajectory, trajectory_qos)
            self.get_logger().info(
                f"Subscribed to {topic} (PoseArray, TRANSIENT_LOCAL), "
                f"frame transform to {_TRUNK_FRAME}, "
                f"duration={self._duration_ms} ms, hand={hand}, "
                f"move_api={self._move_api_name}."
            )

        def _on_trajectory(self, msg: PoseArray) -> None:
            if not msg.poses:
                self.get_logger().warning("Received empty PoseArray; nothing to execute.")
                return

            with self._execution_lock:
                if self._executing:
                    self.get_logger().warning(
                        f"Dropping PoseArray with {len(msg.poses)} poses "
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

        def _execute_trajectory(self, msg: PoseArray) -> None:
            try:
                source_frame = msg.header.frame_id.strip()
                if not source_frame:
                    self.get_logger().error(
                        "PoseArray header.frame_id is empty; refusing to execute trajectory."
                    )
                    return

                poses = msg.poses
                if self._max_poses is not None and self._max_poses < len(poses):
                    poses = poses[:self._max_poses]
                    self.get_logger().info(
                        f"Truncated to {len(poses)} poses (--max-poses {self._max_poses})."
                    )
                msg.poses = poses
                n = len(poses)
                self.get_logger().info(
                    f"Executing PoseArray with {n} poses from frame '{source_frame}'."
                )

                # Look up the transform once for the whole array.
                tfm = self._lookup_transform(msg, source_frame)
                if source_frame != _TRUNK_FRAME and tfm is None:
                    self.get_logger().error(
                        f"TF lookup {source_frame} -> {_TRUNK_FRAME} failed; aborting trajectory."
                    )
                    return
                if tfm is not None:
                    t = tfm.transform.translation
                    r = tfm.transform.rotation
                    self.get_logger().info(
                        f"TF {source_frame} -> {_TRUNK_FRAME}: "
                        f"t=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) "
                        f"r=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})"
                    )

                # Downsample: keep every Nth pose, always include first and last.
                all_poses = msg.poses
                if self._downsample > 1:
                    indices = list(range(0, len(all_poses), self._downsample))
                    if indices[-1] != len(all_poses) - 1:
                        indices.append(len(all_poses) - 1)
                    all_poses = [all_poses[i] for i in indices]
                    self.get_logger().info(
                        f"Downsampled {n} -> {len(all_poses)} poses (step={self._downsample})."
                    )

                # Scale duration proportionally to downsample step.
                move_duration_ms = self._duration_ms * self._downsample
                init_duration_ms = move_duration_ms * 100
                n = len(all_poses)
                for idx, pose in enumerate(all_poses, start=1):
                    is_first = idx == 1
                    dur = init_duration_ms if is_first else move_duration_ms
                    loop_period = dur / 1000.0
                    t0 = time.monotonic()

                    pose_trunk = self._pose_to_trunk(pose, source_frame, tfm)
                    if pose_trunk is None:
                        self.get_logger().error(
                            f"Aborting trajectory: TF transform failed at pose {idx}/{n}."
                        )
                        return

                    ret = self._move_to_pose(pose_trunk, dur)
                    p = pose_trunk.pose.position
                    q = pose_trunk.pose.orientation
                    if ret != 0:
                        self.get_logger().error(
                            f"Move failed for pose {idx}/{n} with code {ret}: "
                            f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                            f"quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
                        )
                    else:
                        self.get_logger().info(
                            f"Pose {idx}/{n} sent at "
                            f"({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) in {_TRUNK_FRAME}."
                        )

                    # Pace to fixed interval, absorbing call overhead
                    if idx < n:
                        elapsed = time.monotonic() - t0
                        remaining = loop_period - elapsed
                        if remaining > 0:
                            time.sleep(remaining)
            finally:
                with self._execution_lock:
                    self._executing = False

        def _lookup_transform(self, msg: PoseArray, source_frame: str):
            """Look up the transform from source_frame to Trunk. Returns None if same frame or on failure."""
            if source_frame == _TRUNK_FRAME:
                return None
            try:
                # Use time(0) to get the latest available transform.
                # The PoseArray may be a stale latched message with a timestamp
                # older than the TF buffer, so requesting an exact time would fail.
                return self._tf_buffer.lookup_transform(
                    _TRUNK_FRAME, source_frame, Time(), timeout=Duration(seconds=5.0),
                )
            except TransformException as ex:
                self.get_logger().error(f"TF {source_frame} -> {_TRUNK_FRAME} failed: {ex}")
                return None

        def _pose_to_trunk(self, pose, source_frame: str, tfm) -> PoseStamped | None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.pose = pose

            if tfm is None and source_frame == _TRUNK_FRAME:
                return pose_stamped
            if tfm is None:
                return None

            q_in = pose_stamped.pose.orientation
            rpy_in = _euler_from_quaternion(q_in.x, q_in.y, q_in.z, q_in.w)
            out = do_transform_pose_stamped(pose_stamped, tfm)
            out.header.frame_id = _TRUNK_FRAME
            q_out = out.pose.orientation
            rpy_out = _euler_from_quaternion(q_out.x, q_out.y, q_out.z, q_out.w)
            self.get_logger().info(
                f"TF orient: "
                f"before=({q_in.x:.4f},{q_in.y:.4f},{q_in.z:.4f},{q_in.w:.4f}) "
                f"rpy=({rpy_in[0]:.4f},{rpy_in[1]:.4f},{rpy_in[2]:.4f}) | "
                f"after=({q_out.x:.4f},{q_out.y:.4f},{q_out.z:.4f},{q_out.w:.4f}) "
                f"rpy=({rpy_out[0]:.4f},{rpy_out[1]:.4f},{rpy_out[2]:.4f})"
            )
            return out

        def _move_to_pose(self, pose_trunk: PoseStamped, duration_ms: int | None = None) -> int:
            dur = duration_ms if duration_ms is not None else self._duration_ms
            p = pose_trunk.pose.position
            q = pose_trunk.pose.orientation
            roll, pitch, yaw = _euler_from_quaternion(q.x, q.y, q.z, q.w)

            self.get_logger().debug(
                f"EE cmd: pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                f"quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) "
                f"rpy=({roll:.4f},{pitch:.4f},{yaw:.4f}) "
                f"dur={dur}ms"
            )

            tar = Posture()
            tar.position = Position(float(p.x), float(p.y), float(p.z))
            tar.orientation = Orientation(float(roll), float(pitch), float(yaw))

            return self._move_api(tar, dur, self._hand_index)

    rclpy.init()
    node = None
    try:
        node = UpperBodyTrajectoryNode()
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


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Python port of b1_upper_body_custom_control.cpp.\n"
            "Default: GetMode, UpperBodyCustomControl, SwitchHandEndEffectorControlMode, "
            "MoveHandEndEffector (demo pose).\n"
            "With --topic: subscribe to a PoseArray and execute each pose sequentially."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "network_interface",
        type=str,
        help="Network interface for robot communication (e.g. eth0, 127.0.0.1)",
    )

    # Core mode args
    parser.add_argument(
        "action",
        nargs="*",
        default=[],
        help="Optional: 'stop' to disable UpperBodyCustomControl",
    )

    # Trajectory mode args
    parser.add_argument(
        "--topic",
        type=str,
        default="/planning/ee_trajectory",
        help="PoseArray topic to subscribe to (enables trajectory mode, requires ROS2)",
    )
    parser.add_argument(
        "--hand",
        type=str,
        choices=("right", "left"),
        default="right",
        help="Which hand to command (trajectory mode only)",
    )
    parser.add_argument(
        "--duration-ms",
        type=int,
        default=20,
        help="Move duration per pose in ms (trajectory mode only, default 20)",
    )
    parser.add_argument(
        "--max-poses",
        type=int,
        default=None,
        help="Execute only the first N poses (default: all)",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=1,
        help="Send every Nth pose (first and last always included). "
             "Duration is scaled by N automatically. (default: 1 = no downsampling)",
    )

    # Split off --ros-args so argparse doesn't choke on them.
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        split = argv.index("--ros-args")
        args = parser.parse_args(argv[:split])
    else:
        args = parser.parse_args(argv)

    # Trajectory mode
    if args.topic is not None:
        sys.exit(run_trajectory(args.network_interface, args.topic, args.hand, args.duration_ms, args.downsample, args.max_poses))

    # Core mode: just enable/disable UpperBodyCustomControl, no movement
    start = True
    for a in args.action:
        if a in ("stop", "0", "false"):
            start = False

    sys.exit(run_core(args.network_interface, start))


if __name__ == "__main__":
    main()
