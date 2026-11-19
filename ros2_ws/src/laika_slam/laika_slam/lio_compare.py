"""Modo sombra de FAST-LIO (P5.4): compara la pose 2D de LIO con map->base_link de slam_toolbox. No publica nada.

LIO da la pose del IMU del cuerpo en camera_init; base_link = p + R*T (T = origen de base_link en el marco del IMU,
parametro base_in_imu; mismo valor que extrinsic_T de fast_lio_l1.yaml). Se asume camera_init alineado con la
gravedad (shortcut: sin R_align; la pose 2D solo pierde precision si el piso inicial esta inclinado).
El primer par valido fija la alineacion 2D; cada log_period s imprime dxy/dyaw y se escribe un CSV."""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
import tf2_ros


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rotz(a, x, y):
    return math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def base_pose_2d(p, q, t):
    """Pose 2D (x, y, yaw) de base_link: p + R*t con R de cuaternion q (x, y, z, w)."""
    x, y, z, w = q
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)]]
    bx = p[0] + sum(r[0][i] * t[i] for i in range(3))
    by = p[1] + sum(r[1][i] * t[i] for i in range(3))
    return bx, by, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class LioCompare(Node):
    def __init__(self):
        super().__init__('lio_compare')
        for n, d in (('odom_topic', '/lio/odometry'), ('map_frame', 'map'), ('base_frame', 'base_link'),
                     ('base_in_imu', [0.02557, 0.0, -0.04232]), ('log_period', 5.0),
                     ('csv_dir', os.path.expanduser('~/laika_bags'))):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.t_b, self.mf, self.bf, self.period = g('base_in_imu'), g('map_frame'), g('base_frame'), g('log_period')
        self.buf = tf2_ros.Buffer(Duration(seconds=30))
        self.lis = tf2_ros.TransformListener(self.buf, self)
        os.makedirs(g('csv_dir'), exist_ok=True)
        self.path = os.path.join(g('csv_dir'), 'lio_compare_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        self.csv = open(self.path, 'w')
        self.csv.write('t,lio_x,lio_y,lio_yaw,map_x,map_y,map_yaw\n')
        self.al = None  # (dyaw, tx, ty): lio_en_map = Rz(dyaw)*lio + t
        self.n_ok = self.n_fail = 0
        self.last = None
        self.t_log = self.get_clock().now()
        self.create_subscription(Odometry, g('odom_topic'), self.cb, 50)
        self.get_logger().info('lio_compare: CSV en %s' % self.path)

    def cb(self, m):
        o = m.pose.pose
        lx, ly, ly_ = base_pose_2d((o.position.x, o.position.y, o.position.z),
                                   (o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w), self.t_b)
        try:
            tr = self.buf.lookup_transform(self.mf, self.bf, Time.from_msg(m.header.stamp)).transform
        except Exception:  # sin map->odom todavia o stamp fuera del buffer
            self.n_fail += 1
            return
        mx, my, myaw = tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        if self.al is None:
            dyaw = wrap(myaw - ly_)
            rx, ry = rotz(dyaw, lx, ly)
            self.al = (dyaw, mx - rx, my - ry)
            self.get_logger().info('Alineacion 2D fijada con el primer par valido.')
        rx, ry = rotz(self.al[0], lx, ly)
        ax, ay, ayaw = rx + self.al[1], ry + self.al[2], wrap(ly_ + self.al[0])
        self.n_ok += 1
        self.last = (math.hypot(ax - mx, ay - my), math.degrees(wrap(ayaw - myaw)))
        self.csv.write('%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f\n' % (
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, ax, ay, ayaw, mx, my, myaw))
        now = self.get_clock().now()
        if (now - self.t_log).nanoseconds * 1e-9 >= self.period:
            self.t_log = now
            self.csv.flush()
            self.get_logger().info('dxy %.3f m, dyaw %.2f deg (pares %d, sin TF %d)' % (*self.last, self.n_ok, self.n_fail))

    def close(self):
        self.csv.close()


def main():
    rclpy.init()
    n = LioCompare()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.close()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
