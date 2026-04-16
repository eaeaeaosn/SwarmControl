import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.msg import RigidBodyArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class MocapRepublisher(Node):
    def __init__(self):
        super().__init__('mocap_republisher')
        self.sub = self.create_subscription(
            RigidBodyArray, '/mocap_rigid_bodies', self.callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot1/pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/robot1/marker', 10)

    def callback(self, msg):
        for rb in msg.rigid_bodies:
            if rb.id == 13:  # robot1
                # Publish PoseStamped
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'
                pose.pose = rb.pose_stamped.pose
                self.pose_pub.publish(pose)

                # Publish RViz marker (sphere)
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'world'
                marker.ns = 'robot1'
                marker.id = 13
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose = rb.pose_stamped.pose
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MocapRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()