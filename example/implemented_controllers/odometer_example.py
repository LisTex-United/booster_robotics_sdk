from booster_robotics_sdk_python import ChannelFactory, B1OdometerStateSubscriber
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return (0.0, 0.0, qz, qw)

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_frame = "odom"
        self.base_frame = "base_link"
        self.publish_tf = False

        self.pose_cov_x = 0.03
        self.pose_cov_y = 0.03
        self.pose_cov_yaw = 0.08

        self.twist_cov_vx = 0.025
        self.twist_cov_vy = 0.08
        self.twist_cov_wz = 0.18

        self.unused_pose_cov = 1e6
        self.unused_twist_cov = 1e6

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def callback(self, msg):
        now = self.get_clock().now()
        stamp = now.to_msg()

        x = float(msg.x)
        y = float(msg.y)
        theta = float(msg.theta)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-4:
                dx = x - self.prev_x
                dy = y - self.prev_y
                dtheta = normalize_angle(theta - self.prev_theta)

                vx_world = dx / dt
                vy_world = dy / dt
                wz = dtheta / dt

                cos_t = math.cos(theta)
                sin_t = math.sin(theta)

                vx =  cos_t * vx_world + sin_t * vy_world
                vy = -sin_t * vx_world + cos_t * vy_world

        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        self.prev_time = now

        qx, qy, qz, qw = yaw_to_quaternion(theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = [
            self.pose_cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.pose_cov_y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.unused_pose_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.unused_pose_cov, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.unused_pose_cov, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.pose_cov_yaw
        ]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

        odom.twist.covariance = [
            self.twist_cov_vx, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.twist_cov_vy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.unused_twist_cov, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.unused_twist_cov, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.unused_twist_cov, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.twist_cov_wz
        ]

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init()   
    ChannelFactory.Instance().Init(0)
    node = OdomRepublisher()
    channel_subscriber = B1OdometerStateSubscriber(node.callback)
    channel_subscriber.InitChannel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()