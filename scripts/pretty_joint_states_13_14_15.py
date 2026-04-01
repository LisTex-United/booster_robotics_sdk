#!/usr/bin/env python3
import argparse
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrettyPrinter(Node):
    def __init__(self, topic_name: str, indices: list[int], decimals: int):
        super().__init__("joint_state_pretty_printer")
        self.indices = indices
        self.decimals = decimals
        self.last_msg_time = None

        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self._callback,
            10,
        )

        self.get_logger().info(
            f"Listening on {topic_name} and printing joint indices: {self.indices}"
        )

    def _fmt(self, value: float) -> str:
        return f"{value:.{self.decimals}f}"

    def _callback(self, msg: JointState):
        now = self.get_clock().now()
        hz_text = "n/a"
        if self.last_msg_time is not None:
            dt_ns = (now - self.last_msg_time).nanoseconds
            if dt_ns > 0:
                hz_text = f"{1e9 / dt_ns:.1f}"
        self.last_msg_time = now

        print("\033[2J\033[H", end="")
        print("=" * 72)
        print("/joint_states monitor (indices 13, 14, 15)")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}    Rate: {hz_text} Hz")
        print("=" * 72)
        print(f"{'Index':<8}{'Name':<30}{'Position (rad)':>16}{'Position (deg)':>16}")
        print("-" * 72)

        name_len = len(msg.name)
        pos_len = len(msg.position)

        for idx in self.indices:
            if idx < 0:
                name = "(invalid index)"
                pos_rad = None
            elif idx >= pos_len:
                name = msg.name[idx] if idx < name_len else "(out of range)"
                pos_rad = None
            else:
                name = msg.name[idx] if idx < name_len else "(unnamed)"
                pos_rad = msg.position[idx]

            if pos_rad is None:
                rad_str = "N/A"
                deg_str = "N/A"
            else:
                rad_str = self._fmt(pos_rad)
                deg_str = self._fmt(math.degrees(pos_rad))

            print(f"{idx:<8}{name:<30}{rad_str:>16}{deg_str:>16}")

        print("-" * 72)
        print("Press Ctrl+C to stop")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pretty-print selected joints from /joint_states"
    )
    parser.add_argument(
        "--topic",
        default="/joint_states",
        help="JointState topic name (default: /joint_states)",
    )
    parser.add_argument(
        "--indices",
        nargs="+",
        type=int,
        default=[13, 14, 15],
        help="Joint indices to display (default: 13 14 15)",
    )
    parser.add_argument(
        "--decimals",
        type=int,
        default=4,
        help="Decimal places for values (default: 4)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = JointStatePrettyPrinter(
        topic_name=args.topic,
        indices=args.indices,
        decimals=args.decimals,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
