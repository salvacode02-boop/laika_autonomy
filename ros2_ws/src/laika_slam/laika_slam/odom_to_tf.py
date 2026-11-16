from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.odom_pub = self.create_publisher(Odometry, '/laika/odom', 10)  # R3: stamp corregido para Nav2
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.cb, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/laika/cloud_deskewed', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.cloud_cb, qos_profile_sensor_data)
        for n, d in (('window_size', 10), ('publish_rate', 5.0), ('z_min_rel', -0.12),
                     ('z_max_rel', 0.50), ('min_dist', 0.6)):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.win = deque(maxlen=int(g('window_size')))
        self.z_min, self.z_max, self.min_dist = g('z_min_rel'), g('z_max_rel'), g('min_dist')
        self.base = None  # (x, y, z) de base_link en odom
        self.last_header = None
        self.counts = []
        self.create_timer(1.0 / g('publish_rate'), self.publish_window)
        self.offset = None  # ns; now() - stamp. Los stamps de la .161 van ~2060 s atrasados.

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def shift(self, stamp):
        ns = stamp.sec * 1000000000 + stamp.nanosec + self.offset
        stamp.sec, stamp.nanosec = divmod(ns, 1000000000)
        return ns

    _NP = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2',
           PointField.UINT16: 'u2', PointField.INT32: 'i4', PointField.UINT32: 'u4',
           PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}

    def cloud_cb(self, msg):
        if self.offset is None or self.base is None:
            return
        dt = np.dtype({'names': [f.name for f in msg.fields],
                       'formats': [self._NP[f.datatype] for f in msg.fields],
                       'offsets': [f.offset for f in msg.fields],
                       'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32)
        bx, by, bz = self.base
        dz = xyz[:, 2] - bz
        d = np.hypot(xyz[:, 0] - bx, xyz[:, 1] - by)
        keep = np.isfinite(xyz).all(axis=1) & (dz >= self.z_min) & (dz <= self.z_max) & (d >= self.min_dist)
        xyz = xyz[keep]
        self.shift(msg.header.stamp)
        self.last_header = msg.header
        self.win.append(xyz)
        self.counts.append(len(xyz))
        if len(self.counts) >= 150:  # ~10 s a 15 Hz
            self.get_logger().info('Puntos por nube filtrada (prom): %.0f' % np.mean(self.counts))
            self.counts = []

    def publish_window(self):
        if not self.win:
            return
        pts = np.concatenate(self.win)
        out = PointCloud2()
        out.header = self.last_header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.is_bigendian, out.point_step = False, 12
        out.row_step, out.is_dense = 12 * len(pts), True
        out.data = pts.tobytes()
        self.cloud_pub.publish(out)

    def cb(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().error(
                'Marcos inesperados: %r -> %r' % (msg.header.frame_id, msg.child_frame_id),
                throttle_duration_sec=5.0)
            return
        raw = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if self.offset is None or abs(self.now_ns() - (raw + self.offset)) > 500000000:
            if self.offset is not None:
                self.get_logger().warn('Offset de reloj desfasado >0.5 s; recalculando',
                                       throttle_duration_sec=5.0)
            self.offset = self.now_ns() - raw
        p = msg.pose.pose.position
        self.base = (p.x, p.y, p.z)
        t = TransformStamped()
        t.header = msg.header
        self.shift(t.header.stamp)  # stamp original + offset comun (D3 revisada)
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.pub.publish(TFMessage(transforms=[t]))
        msg.header.stamp = t.header.stamp
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
