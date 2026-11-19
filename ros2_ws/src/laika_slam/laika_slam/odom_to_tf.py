import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger


# Referencia (x, y, yaw) del reset de sesion: lo publicado es Rz(-yaw)(p - p_ref), yaw - yaw_ref.
def quat_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rebase_xy(x, y, ref):
    c, s = math.cos(ref[2]), math.sin(ref[2])
    dx, dy = x - ref[0], y - ref[1]
    return c * dx + s * dy, -s * dx + c * dy


def rebase_quat(q, ref):
    """q = (x, y, z, w); r = Rz(-yaw_ref); devuelve r*q."""
    rz, rw = math.sin(-ref[2] / 2), math.cos(-ref[2] / 2)
    x, y, z, w = q
    return (rw * x - rz * y, rw * y + rz * x, rw * z + rz * w, rw * w - rz * z)


def inject_jump(ref, raw_xy, deg):
    """Ref nueva que gira el yaw publicado `deg` grados dejando igual la posicion publicada actual."""
    px, py = rebase_xy(raw_xy[0], raw_xy[1], ref)
    yaw = ref[2] - math.radians(deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return (raw_xy[0] - (c * px - s * py), raw_xy[1] - (s * px + c * py), yaw)


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        for n, d in (('odom_topic', '/utlidar/robot_odom'), ('cloud_topic', '/utlidar/cloud_deskewed'),
                     ('publish_tf', True), ('test_yaw_jump_deg', 0.0)):
            self.declare_parameter(n, d)
        self.publish_tf = self.get_parameter('publish_tf').value
        odom_topic, cloud_topic = self.get_parameter('odom_topic').value, self.get_parameter('cloud_topic').value
        self.pub = self.create_publisher(TFMessage, '/tf', 100) if self.publish_tf else None
        self.odom_pub = self.create_publisher(Odometry, '/laika/odom', 10)  # R3: stamp corregido para Nav2
        self.create_subscription(Odometry, odom_topic, self.cb, qos_profile_sensor_data)
        if cloud_topic:
            self.cloud_pub = self.create_publisher(PointCloud2, '/laika/cloud_deskewed', qos_profile_sensor_data)
            self.create_subscription(PointCloud2, cloud_topic, self.cloud_cb, qos_profile_sensor_data)
        for n, d in (('window_size', 10), ('publish_rate', 5.0), ('z_min_rel', -0.12),
                     ('z_max_rel', 0.50), ('min_dist', 0.6)):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.win = deque(maxlen=int(g('window_size')))
        self.z_min, self.z_max, self.min_dist = g('z_min_rel'), g('z_max_rel'), g('min_dist')
        self.base = None  # (x, y, z) de base_link en odom
        self.last_header = None
        self.counts = []
        if cloud_topic:
            self.create_timer(1.0 / g('publish_rate'), self.publish_window)
        self.offset = None  # ns; now() - stamp. Los stamps de la .161 van ~2060 s atrasados.
        self.ref = None  # (x, y, yaw) del reset; None = identidad (comportamiento original)
        self.raw = None  # ultima pose cruda (x, y, yaw)
        if self.publish_tf:
            self.create_service(Trigger, '/laika/reset_odom', self.reset_cb)
            if g('test_yaw_jump_deg') != 0.0:
                self.jump = g('test_yaw_jump_deg')
                self.create_service(Trigger, '/laika/inject_yaw_jump', self.jump_cb)

    def reset_cb(self, _req, res):
        if self.raw is None:
            res.success, res.message = False, 'sin odometria todavia'
            return res
        self.ref = self.raw
        self.win.clear()
        self.offset = None
        res.success, res.message = True, 'ref=(%.3f, %.3f, %.1f deg)' % (
            self.ref[0], self.ref[1], math.degrees(self.ref[2]))
        return res

    def jump_cb(self, _req, res):
        if self.raw is None:
            res.success, res.message = False, 'sin odometria todavia'
            return res
        self.ref = inject_jump(self.ref or (0.0, 0.0, 0.0), self.raw, self.jump)
        self.win.clear()
        res.success, res.message = True, 'salto de yaw %.1f deg' % self.jump
        return res

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
        if self.ref is not None:
            xyz = xyz.copy()
            xyz[:, 0], xyz[:, 1] = rebase_xy(xyz[:, 0].astype(np.float64), xyz[:, 1].astype(np.float64), self.ref)
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
        o = msg.pose.pose.orientation
        self.raw = (p.x, p.y, quat_yaw(o.x, o.y, o.z, o.w))
        if self.ref is not None:  # no toca z, roll, pitch ni twist
            p.x, p.y = rebase_xy(p.x, p.y, self.ref)
            o.x, o.y, o.z, o.w = rebase_quat((o.x, o.y, o.z, o.w), self.ref)
        t = TransformStamped()
        t.header = msg.header
        self.shift(t.header.stamp)  # stamp original + offset comun (D3 revisada)
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        if self.pub:
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
