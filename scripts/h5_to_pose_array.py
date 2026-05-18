#!/usr/bin/env python3
"""Replay the EE-delta trajectory from a Tiago-style H5 file onto a
user-specified starting pose, and publish the resulting absolute poses as a
PoseArray on /planning/ee_trajectory for ``b1_upper_body_custom_control.py``
to consume.

Deltas come from ``actions[:, 3:9]``:
  cols 3-5  position deltas (m)
  cols 6-8  rotation deltas (axis-angle, body frame)

These are commanded velocity-style values that overshoot if integrated raw;
``--delta-scale 0.1`` empirically reproduces the recorded ``obs/{side}`` path
on left_1.h5.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import h5py
import numpy as np


def quat_mul(q1, q2):
    """Hamilton product (x,y,z,w) * (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def axis_angle_to_quat(omega):
    """3-vector axis-angle -> quaternion (x, y, z, w)."""
    theta = float(np.linalg.norm(omega))
    if theta < 1e-9:
        return np.array([omega[0] / 2.0, omega[1] / 2.0, omega[2] / 2.0, 1.0])
    axis = omega / theta
    half = theta / 2.0
    s = math.sin(half)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)])


def integrate(start_pose, dpos, drot):
    n = len(dpos)
    out = np.zeros((n + 1, 7))
    out[0] = start_pose
    p = np.asarray(start_pose[:3], dtype=float)
    q = np.asarray(start_pose[3:7], dtype=float)
    q = q / np.linalg.norm(q)
    for i in range(n):
        p = p + dpos[i]
        q = quat_mul(q, axis_angle_to_quat(drot[i]))
        q = q / np.linalg.norm(q)
        out[i + 1, :3] = p
        out[i + 1, 3:7] = q
    return out


def publish(poses, topic, frame_id, hold_secs):
    import rclpy
    from geometry_msgs.msg import Pose, PoseArray
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    rclpy.init()
    node = Node("h5_delta_replay_publisher")
    pub = node.create_publisher(PoseArray, topic, qos)

    msg = PoseArray()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    for row in poses:
        p = Pose()
        p.position.x = float(row[0])
        p.position.y = float(row[1])
        p.position.z = float(row[2])
        p.orientation.x = float(row[3])
        p.orientation.y = float(row[4])
        p.orientation.z = float(row[5])
        p.orientation.w = float(row[6])
        msg.poses.append(p)

    pub.publish(msg)
    node.get_logger().info(
        f"Published {len(msg.poses)} poses on '{topic}' (frame_id='{frame_id}'). "
        f"Holding for {hold_secs:.1f}s."
    )
    end_t = time.monotonic() + hold_secs
    while rclpy.ok() and time.monotonic() < end_t:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("h5_path")
    parser.add_argument(
        "--start-pose", type=float, nargs=7, required=True,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="B1 starting pose in Trunk frame: x y z qx qy qz qw (required)",
    )
    parser.add_argument(
        "--side", choices=("left", "right"), default="left",
        help="Which arm's actions slice to use (default: left). "
             "NOTE: actions[:, 3:9] is a single arm channel — the H5 schema "
             "for left/right per-arm action splits isn't documented; this flag "
             "currently affects only the start-index lookup, not column slicing.",
    )
    parser.add_argument(
        "--delta-scale", type=float, default=0.1,
        help="Scale each delta before integrating. 0.1 empirically reproduces "
             "the recorded path on left_1.h5 (default: 0.1)",
    )
    parser.add_argument(
        "--start", type=int, default=None,
        help="Start index into actions (default: H5 attr 'man_start_index')",
    )
    parser.add_argument(
        "--end", type=int, default=None,
        help="End index exclusive (default: end of file)",
    )
    parser.add_argument("--frame-id", default="Trunk")
    parser.add_argument("--topic", default="/planning/ee_trajectory")
    parser.add_argument("--hold-secs", type=float, default=5.0)
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print summary only, do not publish",
    )
    args = parser.parse_args()

    with h5py.File(args.h5_path, "r") as f:
        actions = f["actions"][:]
        msi = int(f.attrs.get("man_start_index", 0))

    n_total = actions.shape[0]
    s = msi if args.start is None else args.start
    e = n_total if args.end is None else args.end
    s = max(0, min(s, n_total - 1))
    e = max(s + 1, min(e, n_total))

    dpos = actions[s:e, 3:6] * args.delta_scale
    drot = actions[s:e, 6:9] * args.delta_scale
    poses = integrate(args.start_pose, dpos, drot)

    print(f"H5: {args.h5_path}")
    print(f"  slice [{s}:{e}] of {n_total} (man_start_index={msi}), "
          f"delta_scale={args.delta_scale}")
    print(f"  start pose: pos={poses[0, 0:3]}, quat={poses[0, 3:7]}")
    print(f"  end   pose: pos={poses[-1, 0:3]}, quat={poses[-1, 3:7]}")
    print(f"  pos range:  min={poses[:, 0:3].min(axis=0)}")
    print(f"              max={poses[:, 0:3].max(axis=0)}")

    if args.dry_run:
        print("[dry-run] not publishing")
        return 0

    publish(poses, args.topic, args.frame_id, args.hold_secs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
