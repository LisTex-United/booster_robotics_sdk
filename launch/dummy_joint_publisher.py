#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import sys
import time

class DummyJointPublisher(Node):
    def __init__(self, urdf_path):
        super().__init__('dummy_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_names = self.parse_urdf(urdf_path)
        self.get_logger().info(f'Publishing 0 for joints: {self.joint_names}')

    def parse_urdf(self, urdf_path):
        joints = []
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for joint in root.findall('joint'):
                jtype = joint.get('type')
                if jtype != 'fixed':
                    name = joint.get('name')
                    if name:
                        joints.append(name)
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF: {e}')
        return joints

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        self.publisher_.publish(msg)

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: dummy_joint_publisher.py <urdf_file>")
        return

    rclpy.init(args=args)
    node = DummyJointPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
