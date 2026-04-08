#!/usr/bin/env python3
"""High-level head control via B1LocoClient.RotateHead only — does not change robot mode."""

import argparse
import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from booster_robotics_sdk_python import (
    RobotMode,
    B1LocoClient,
    ChannelFactory,
    GetModeResponse,
)

# API limits (rad): pitch down positive; yaw left positive
PITCH_MIN = -0.3
PITCH_MAX = 1.0
YAW_MIN = -0.785
YAW_MAX = 0.785


def clamp_pitch_yaw(pitch: float, yaw: float) -> Tuple[float, float]:
    return (
        max(PITCH_MIN, min(PITCH_MAX, pitch)),
        max(YAW_MIN, min(YAW_MAX, yaw)),
    )


def get_mode_with_warmup(client: B1LocoClient, attempts: int = 2, delay_s: float = 0.5) -> Tuple[int, GetModeResponse]:
    """First RPC after Init often times out (100) while DDS matches; retry once after a short pause."""
    mode_resp = GetModeResponse()
    last = -1
    for i in range(attempts):
        last = client.GetMode(mode_resp)
        if last == 0:
            break
        if i + 1 < attempts:
            time.sleep(delay_s)
    return last, mode_resp


class RotateHeadNode(Node):
    """ROS 2 node: forwards /booster/head/rotate to RotateHead(pitch, yaw)."""

    def __init__(self, network_interface: str):
        super().__init__("rotate_head_node")

        self.get_logger().info("Initializing Booster SDK (RotateHead API, no mode change)...")
        ChannelFactory.Instance().Init(0, network_interface)
        self.client = B1LocoClient()
        self.client.Init()

        ret, mode_resp = get_mode_with_warmup(self.client)
        if ret == 0:
            self.get_logger().info(f"Current robot mode (unchanged): {mode_resp.mode}")
        else:
            self.get_logger().warning(f"GetMode failed ({ret}); continuing anyway.")

        self.create_subscription(
            Float64MultiArray,
            "/booster/head/rotate",
            self._on_command,
            10,
        )
        self.get_logger().info(
            "Subscribed to /booster/head/rotate — Float64MultiArray [pitch, yaw] (rad). "
            f"Clamped to pitch[{PITCH_MIN}, {PITCH_MAX}], yaw[{YAW_MIN}, {YAW_MAX}]."
        )

    def _on_command(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warning("Expected [pitch, yaw]; ignoring message.")
            return
        pitch, yaw = float(msg.data[0]), float(msg.data[1])
        pitch_c, yaw_c = clamp_pitch_yaw(pitch, yaw)
        if (pitch_c, yaw_c) != (pitch, yaw):
            self.get_logger().info(
                f"Clamped pitch {pitch:.4f}->{pitch_c:.4f}, yaw {yaw:.4f}->{yaw_c:.4f}"
            )
        ret = self.client.RotateHead(pitch_c, yaw_c)
        if ret != 0:
            self.get_logger().error(f"RotateHead returned {ret}")
        else:
            self.get_logger().info(f"RotateHead({pitch_c:.4f}, {yaw_c:.4f}) ok")


def send_once(network_interface: str, pitch: float, yaw: float) -> int:
    ChannelFactory.Instance().Init(0, network_interface)
    client = B1LocoClient()
    client.Init()
    gm, mode_resp = get_mode_with_warmup(client)
    print(f"GetMode -> {gm}" + (f", mode={mode_resp.mode}" if gm == 0 else ""))
    pitch_c, yaw_c = clamp_pitch_yaw(pitch, yaw)
    ret = client.RotateHead(pitch_c, yaw_c)
    print(f"RotateHead({pitch_c}, {yaw_c}) -> {ret}")
    return int(ret)


def main():
    parser = argparse.ArgumentParser(
        description="Head control via RotateHead only; robot mode is never changed."
    )
    parser.add_argument(
        "network_interface",
        type=str,
        help="Network interface for robot communication (e.g. eth0)",
    )
    parser.add_argument(
        "--once",
        nargs=2,
        type=float,
        metavar=("PITCH", "YAW"),
        help="Send a single RotateHead(pitch, yaw) in rad and exit (no ROS 2)",
    )
    args = parser.parse_args()

    if args.once is not None:
        pitch, yaw = args.once
        raise SystemExit(0 if send_once(args.network_interface, pitch, yaw) == 0 else 1)

    rclpy.init()
    try:
        node = RotateHeadNode(args.network_interface)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
