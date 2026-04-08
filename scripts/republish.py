import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

class TrajectoryMarkerArray(Node):
    def __init__(self):
        super().__init__('ee_marker_array_publisher')

        # QoS to match TransientLocal (Latched) topics
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.sub = self.create_subscription(
            PoseArray, 
            '/planning/ee_trajectory', 
            self.callback, 
            qos_profile)

        # Publisher
        self.pub = self.create_publisher(
            MarkerArray, 
            '/visualization/ee_markers', 
            qos_profile)

        self.get_logger().info('MarkerArray Republisher Node Started')

    def callback(self, msg):
        marker_array = MarkerArray()
        
        # Optional: Add a DELETEALL marker to clear previous visualization in RViz
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, pose in enumerate(msg.poses):
            marker = Marker()
            marker.header = msg.header
            marker.ns = "ee_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position from the PoseArray
            marker.pose.position = pose.position
            
            # Orientation is required for the message structure but ignored by SPHERE
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0 
            
            # Size of the spheres
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            
            # Color: Solid Pink/Purple (easily visible)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)

        self.pub.publish(marker_array)
        self.get_logger().info(f'Published {len(msg.poses)} spheres to /visualization/ee_markers')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMarkerArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()