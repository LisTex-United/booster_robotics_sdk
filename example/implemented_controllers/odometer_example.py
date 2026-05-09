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
        self.base_frame = "base_footprint"
        self.publish_tf = False

        # Pose is not fused from /odom in robot_localization, so keep it de-emphasized
        self.pose_cov_x = 1e2
        self.pose_cov_y = 1e2
        self.pose_cov_yaw = 1e2

        # Estimated constant twist covariance from residual analysis
        self.twist_cov_matrix = [
            0.007162, -0.001448, 0.0, 0.0, 0.0,  0.002641,
           -0.001448, 0.008955, 0.0, 0.0, 0.0, -0.008650,
            0.0,      0.0,      1e6, 0.0, 0.0,  0.0,
            0.0,      0.0,      0.0, 1e6, 0.0,  0.0,
            0.0,      0.0,      0.0, 0.0, 1e6,  0.0,
            0.002641, -0.008650, 0.0, 0.0, 0.0, 0.017941
        ]

        self.unused_pose_cov = 1e6

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None

        self.sample_count = 0
        self.sample_window = 10
        self.accum_x = 0.0
        self.accum_y = 0.0
        self.accum_theta_sin = 0.0
        self.accum_theta_cos = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.alpha = 0.2
        
    def callback(self, msg):
        # Accumulate for downsampling

        x = float(msg.x)
        y = float(msg.y)
        theta = float(msg.theta)

        self.accum_x += x
        self.accum_y += y
        self.accum_theta_sin += math.sin(theta)
        self.accum_theta_cos += math.cos(theta)
        self.sample_count += 1

        if self.sample_count < self.sample_window:
            return  # Wait until enough samples

        # Compute mean/accumulated values
        mean_x = self.accum_x / self.sample_window
        mean_y = self.accum_y / self.sample_window
        mean_theta = math.atan2(self.accum_theta_sin / self.sample_window, self.accum_theta_cos / self.sample_window)

        now = self.get_clock().now()
        stamp = now.to_msg()
        
        vx = 0.0
        vy = 0.0
        wz = 0.0
        
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-4:
                dx = mean_x - self.prev_x
                dy = mean_y - self.prev_y
                dtheta = normalize_angle(mean_theta - self.prev_theta)

                vx_world = dx / dt
                vy_world = dy / dt
                wz = dtheta / dt

                cos_t = math.cos(self.prev_theta)
                sin_t = math.sin(self.prev_theta)
                vx =  cos_t * vx_world + sin_t * vy_world
                vy = -sin_t * vx_world + cos_t * vy_world
        
        vx_new = max(min(vx, 0.6), -0.6)
        vy_new = max(min(vy, 0.3), -0.3)
        wz_new = max(min(wz, 1.1), -1.1)

        self.vx = self.alpha * vx_new + (1 - self.alpha) * self.vx
        self.vy = self.alpha * vy_new + (1 - self.alpha) * self.vy
        self.wz = self.alpha * wz_new + (1 - self.alpha) * self.wz

        vx = self.vx
        vy = self.vy
        wz = self.wz
        
        if abs(vx) < 0.01:
            vx = 0.0
            self.vx = 0.0

        if abs(vy) < 0.01:
            vy = 0.0
            self.vy = 0.0

        if abs(wz) < 0.01:
            wz = 0.0
            self.wz = 0.0
        
        self.prev_x = mean_x
        self.prev_y = mean_y
        self.prev_theta = mean_theta

        if self.prev_time is None:
            self.prev_time = now

            # Reset accumulators before returning
            self.accum_x = 0.0
            self.accum_y = 0.0
            self.accum_theta_sin = 0.0
            self.accum_theta_cos = 0.0
            self.sample_count = 0
            return

        self.prev_time = now
        
        qx, qy, qz, qw = yaw_to_quaternion(mean_theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = mean_x
        odom.pose.pose.position.y = mean_y
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
        odom.twist.covariance = self.twist_cov_matrix

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = mean_x
            tf_msg.transform.translation.y = mean_y
            tf_msg.transform.translation.z = 0.0

            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)

        # Reset accumulators
        self.accum_x = 0.0
        self.accum_y = 0.0
        self.accum_theta_sin = 0.0
        self.accum_theta_cos = 0.0
        self.sample_count = 0

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