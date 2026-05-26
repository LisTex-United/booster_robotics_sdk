#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import sys
import time
import argparse

# Booster SDK imports
from booster_robotics_sdk_python import (
    ChannelFactory, 
    B1LowStateSubscriber,
    MotorState
)

class RealJointPublisher(Node):
    # Mapping from SDK Motor Index to URDF Joint Name
    # Based on T1_fixed.urdf and SDK binding inspection
    JOINT_MAPPING = {
        0: 'AAHead_yaw',
        1: 'Head_pitch',
        2: 'Left_Shoulder_Pitch',
        3: 'Left_Shoulder_Roll',
        4: 'Left_Elbow_Pitch',
        5: 'Left_Elbow_Yaw',
        # Indices 6, 7, 8 are wrist/hand in 7DOF, but T1_fixed.urdf is 4DOF arm.
        # We define them but they might be filtered out if not in URDF.
        # 6: 'Left_Wrist_Pitch', 
        # 7: 'Left_Wrist_Yaw',
        # 8: 'Left_Hand_Roll',
        
        9: 'Right_Shoulder_Pitch',
        10: 'Right_Shoulder_Roll',
        11: 'Right_Elbow_Pitch',
        12: 'Right_Elbow_Yaw',
        # 13, 14, 15 right wrist/hand
        
        16: 'Waist',
        
        17: 'Left_Hip_Pitch',
        18: 'Left_Hip_Roll',
        19: 'Left_Hip_Yaw',
        20: 'Left_Knee_Pitch',
        21: 'Left_Ankle_Pitch', # Assuming mapping from CrankUpLeft
        22: 'Left_Ankle_Roll',  # Assuming mapping from CrankDownLeft
        
        23: 'Right_Hip_Pitch',
        24: 'Right_Hip_Roll',
        25: 'Right_Hip_Yaw',
        26: 'Right_Knee_Pitch',
        27: 'Right_Ankle_Pitch',
        28: 'Right_Ankle_Roll',
    }
    
    TOTAL_MOTORS = 29

    def __init__(self, urdf_path, network_interface):
        super().__init__('real_joint_publisher')
        
        self.get_logger().info(f"Connecting to robot on interface: {network_interface}")
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Parse URDF to find which joints we should publish
        self.urdf_joints = self.parse_urdf(urdf_path)
        self.get_logger().info(f"Found {len(self.urdf_joints)} active joints in URDF file.")
        
        # Intersection of mapped joints and URDF joints
        self.active_mapping = {}
        for idx, name in self.JOINT_MAPPING.items():
            if name in self.urdf_joints:
                self.active_mapping[idx] = name
            else:
                self.get_logger().debug(f"Joint '{name}' (Index {idx}) not found in URDF, skipping.")
        
        self.current_joint_pos = {} # name -> position

        # Init SDK
        ChannelFactory.Instance().Init(0, network_interface)
        self.state_subscriber = B1LowStateSubscriber(self._state_callback)
        self.state_subscriber.InitChannel()
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states) # 50Hz
        
        self.last_state_time = time.time()

    def parse_urdf(self, urdf_path):
        joints = set()
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for joint in root.findall('joint'):
                jtype = joint.get('type')
                if jtype != 'fixed':
                    name = joint.get('name')
                    if name:
                        joints.add(name)
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF: {e}')
        return joints

    def _state_callback(self, msg):
        self.last_state_time = time.time()
        # Update local state cache
        if len(msg.motor_state_serial) >= self.TOTAL_MOTORS:
             for idx, name in self.active_mapping.items():
                 # access the motor state at this index
                 # Note: msg.motor_state_serial is a vector of MotorState
                 if idx < len(msg.motor_state_serial):
                     self.current_joint_pos[name] = msg.motor_state_serial[idx].q

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if not self.current_joint_pos:
            return

        names = []
        positions = []
        
        for name, pos in self.current_joint_pos.items():
            names.append(name)
            positions.append(pos)
            
        msg.name = names
        msg.position = positions
        self.publisher_.publish(msg)

def main(args=None):
    parser = argparse.ArgumentParser(description="Real time joint publisher for Booster Robot")
    parser.add_argument("urdf_file", help="Path to URDF file")
    parser.add_argument("--network_interface", default="lo", help="Network interface (default: lo)")
    
    # We need to filter out ROS args before parsing with argparse
    ros_args = None
    if args is None:
        args = sys.argv[1:]
        
    # Separate ROS args if any (they start with --ros-args)
    clean_args = []
    # Simple filtering: take everything until --ros-args
    # but rclpy.init(args=args) handles ros args, so we just want to NOT pass them to argparse
    # argparse will complain about unknown args if we pass ros args.
    
    # Better strategy: Let rclpy consume its args first? 
    # Actually, argparse usually handles what it knows.
    # We will try to parse known args only.
    
    parsed_args, unknown = parser.parse_known_args(args)

    rclpy.init(args=args)
    
    node = RealJointPublisher(parsed_args.urdf_file, parsed_args.network_interface)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
