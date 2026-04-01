#!/usr/bin/env python3
import sys
import argparse
import time

from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    RobotMode,
    B1HandIndex,
    Position,
    Orientation,
    Posture,
    GetModeResponse,
)

def main():
    parser = argparse.ArgumentParser(
        description="MoveHandEndEffector demo: small move away from a retract-like pose."
    )
    parser.add_argument("--network_interface", required=True, help="e.g., eth0, enp0s1")
    parser.add_argument("--hand", choices=["right", "left"], default="right", help="Which hand to move")
    parser.add_argument("--dx", type=float, default=0.05, help="Offset in X (meters) from base pose")
    parser.add_argument("--dy", type=float, default=0.00, help="Offset in Y (meters) from base pose")
    parser.add_argument("--dz", type=float, default=0.02, help="Offset in Z (meters) from base pose")
    parser.add_argument("--duration_ms", type=int, default=1500, help="Motion duration in milliseconds")
    parser.add_argument("--enable_hcm", action="store_true", help="Enable Hand End Effector Control Mode (if required)")
    args = parser.parse_args()

    # Initialize SDK
    ChannelFactory.Instance().Init(0, args.network_interface)
    client = B1LocoClient()
    client.Init()

    # Switch to a controllable mode
    mode_resp = GetModeResponse()
    ret = client.GetMode(mode_resp)
    print(f"GetMode at start: {ret}, mode: {mode_resp.mode}")
    client.ChangeMode(RobotMode.kCustom)
    mode_resp = GetModeResponse()
    ret = client.GetMode(mode_resp)
    print(f"GetMode at after change: {ret}, mode: {mode_resp.mode}")
    

    # Optional: enable end-effector control mode if your firmware expects it
    if args.enable_hcm:
        alpha = client.SwitchHandEndEffectorControlMode(True)
        print("SwitchHandEndEffectorControlMode(True) returned:", alpha)

    # Base poses close to examples in the repo (body frame, meters/radians)
    if args.hand == "right":
        base_pos = (0.35, -0.20, 0.10)
        base_ori = (1.57, -1.57, 0.0)  # roll, pitch, yaw
        hand_index = B1HandIndex.kRightHand
    else:
        base_pos = (0.35, 0.25, 0.10)
        base_ori = (-1.57, -1.57, 0.0)
        hand_index = B1HandIndex.kLeftHand

    # Target = base + small offset
    tx = base_pos[0] + args.dx
    ty = base_pos[1] + args.dy
    tz = base_pos[2] + args.dz

    tar = Posture()
    tar.position = Position(tx, ty, tz)
    tar.orientation = Orientation(*base_ori)

    print(f"Moving {args.hand} hand to: "
          f"pos=({tx:.3f}, {ty:.3f}, {tz:.3f}), "
          f"ori=({base_ori[0]:.2f}, {base_ori[1]:.2f}, {base_ori[2]:.2f}), "
          f"duration={args.duration_ms} ms")

    input("Press ENTER to execute the move...")


    # Use MoveHandEndEffectorV2 (same arguments as C++: posture, duration_ms, hand_index)
    res = client.MoveHandEndEffectorV2(tar, args.duration_ms, hand_index)
    print("MoveHandEndEffectorV2 result:", res)

    if args.enable_hcm:
        client.SwitchHandEndEffectorControlMode(False)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")