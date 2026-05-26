import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            idx_pitch = msg.name.index('Right_Hand_Roll')
            # idx_yaw = msg.name.index('Right_wrist_yaw')
            pitch = msg.position[idx_pitch]
            # yaw = msg.position[idx_yaw]
            print(f"Right_wrist_roll: {pitch:.4f}")
        except ValueError:
            # Joint names not found in this message
            pass

def main():
    rclpy.init()
    node = JointStatePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()