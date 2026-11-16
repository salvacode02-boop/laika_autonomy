import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().error(
                'Marcos inesperados: %r -> %r' % (msg.header.frame_id, msg.child_frame_id),
                throttle_duration_sec=5.0)
            return
        t = TransformStamped()
        t.header = msg.header  # stamp original, nunca now()
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.pub.publish(TFMessage(transforms=[t]))


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
