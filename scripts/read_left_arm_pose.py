#!/usr/bin/env python3
"""Read the left arm's current joint angles via B1LowStateSubscriber, run
forward kinematics on robot_fixed.urdf, and print the EE pose in a form
that can be pasted directly into h5_to_pose_array.py --start-pose.

Pose the arm physically (e.g. in kDamping with the robot supported), switch
back to kPrepare to lock joints, then run this.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading

import numpy as np

from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber


# Left arm chain from robot_fixed.urdf (Trunk -> left_hand_link).
CHAIN = [
    ("Left_Shoulder_Pitch", [0.0575, 0.1063, 0.219], [0, 0.00088113, 0], [0, 1, 0]),
    ("Left_Shoulder_Roll",  [0,      0.047,  0],     [0, 0, 0],          [1, 0, 0]),
    ("Left_Elbow_Pitch",    [0.00025,0.0605, 0],     [0, 0, 0],          [0, 1, 0]),
    ("Left_Elbow_Yaw",      [0,      0.1471, 0],     [0, 0, 0],          [0, 0, 1]),
    ("Left_Wrist_Pitch",    [0,      0.105,  0.00025],[0,0, 0],          [0, 1, 0]),
    ("Left_Wrist_Yaw",      [0,      0.042,  0],     [0, 0, 0],          [0, 0, 1]),
    ("Left_Hand_Roll",      [0.00475,0.07,   0.0015],[0, 0, 0],          [1, 0, 0]),
]

# Per launch/real_joint_publisher.py: motor_state_serial indices 2..8 are the
# left arm 7-DOF chain.
LEFT_ARM_MOTOR_INDICES = list(range(2, 9))


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_R(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def fk(q):
    T = np.eye(4)
    for i, (_, xyz, rpy, axis) in enumerate(CHAIN):
        T_origin = np.eye(4)
        T_origin[:3, :3] = rpy_to_R(*rpy)
        T_origin[:3, 3] = xyz
        T_rot = np.eye(4)
        T_rot[:3, :3] = axis_R(axis, q[i])
        T = T @ T_origin @ T_rot
    return T


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("network_interface", help="e.g. 127.0.0.1, eth0")
    parser.add_argument(
        "--timeout", type=float, default=5.0,
        help="Seconds to wait for a LowState message (default: 5)",
    )
    args = parser.parse_args()

    ChannelFactory.Instance().Init(0, args.network_interface)

    received = threading.Event()
    state = {"joints": None}

    def handler(msg):
        if state["joints"] is not None:
            return
        motors = msg.motor_state_serial
        if len(motors) <= LEFT_ARM_MOTOR_INDICES[-1]:
            return
        state["joints"] = [float(motors[i].q) for i in LEFT_ARM_MOTOR_INDICES]
        received.set()

    sub = B1LowStateSubscriber(handler)
    sub.InitChannel()

    if not received.wait(timeout=args.timeout):
        sub.CloseChannel()
        print(f"Timed out after {args.timeout}s waiting for LowState.", file=sys.stderr)
        sys.exit(1)

    sub.CloseChannel()

    q = state["joints"]
    T = fk(q)
    pos = T[:3, 3]
    quat = R_to_quat(T[:3, :3])

    print("Left arm joint angles (rad):")
    for (name, *_), v in zip(CHAIN, q):
        print(f"  {name:25s} {v:+.6f}")

    print()
    print("Left EE pose (Trunk -> left_hand_link):")
    print(f"  position             ({pos[0]:+.6f}, {pos[1]:+.6f}, {pos[2]:+.6f})")
    print(f"  quaternion (x,y,z,w) ({quat[0]:+.6f}, {quat[1]:+.6f}, {quat[2]:+.6f}, {quat[3]:+.6f})")

    print()
    print("Drop-in CLI snippet for h5_to_pose_array.py:")
    print(
        f"  --start-pose {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
        f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"
    )


if __name__ == "__main__":
    main()
