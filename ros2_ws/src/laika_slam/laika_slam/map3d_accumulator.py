import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

_NP = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2',
       PointField.UINT16: 'u2', PointField.INT32: 'i4', PointField.UINT32: 'u4',
       PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
_OFF = 1 << 20  # 21 bits por eje: con vóxel de 5 cm cubre +-52 km


def pack_keys(ijk):
    """ijk: (N,3) int64 con índices de vóxel (pueden ser negativos) -> (N,) int64."""
    u = ijk.astype(np.int64) + _OFF
    return (u[:, 0] << 42) | (u[:, 1] << 21) | u[:, 2]


def unpack_keys(k):
    m = (1 << 21) - 1
    return np.stack([(k >> 42) & m, (k >> 21) & m, k & m], axis=1) - _OFF


def transform_2d(xyz, yaw, tx, ty):
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.array(xyz, dtype=np.float64)
    out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1] + tx
    out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1] + ty
    return out


def write_pcd(path, xyz):
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    n = len(xyz)
    hdr = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
           'TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\n'
           'DATA binary\n' % (n, n))
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(hdr.encode())
        f.write(xyz.tobytes())
    os.replace(tmp, path)


def read_pcd(path):
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode()
            if line.startswith('POINTS'):
                n = int(line.split()[1])
            if line.startswith('DATA'):
                break
        return np.frombuffer(f.read(), dtype=np.float32, count=n * 3).reshape(n, 3)


def merge(gk, gc, new_keys):
    """Suma a (gk, gc) las claves nuevas (cada una = 1 nube distinta)."""
    allk = np.concatenate([gk, new_keys])
    w = np.concatenate([gc, np.ones(len(new_keys), dtype=np.int64)])
    uk, inv = np.unique(allk, return_inverse=True)
    return uk, np.bincount(inv, weights=w).astype(np.int64)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Map3dAccumulator(Node):
    def __init__(self):
        super().__init__('map3d_accumulator')
        for n, d in (('voxel_size', 0.05), ('min_views', 2), ('z_min', -0.5), ('z_max', 2.5),
                     ('merge_rate', 1.0), ('publish_rate', 0.5), ('tf_timeout', 1.0),
                     ('jump_dist', 0.10), ('jump_deg', 2.0), ('pcd_path', '~/laika_maps/map3d.pcd')):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.vox, self.min_views = g('voxel_size'), int(g('min_views'))
        self.z_min, self.z_max, self.tf_timeout = g('z_min'), g('z_max'), g('tf_timeout')
        self.jump_dist, self.jump_rad = g('jump_dist'), math.radians(g('jump_deg'))
        self.pcd_path = os.path.expanduser(g('pcd_path'))
        self.tf = None  # (x, y, yaw, t_recepción)
        self.prev_used = None
        self.pending = []
        self.gk = np.zeros(0, dtype=np.int64)
        self.gc = np.zeros(0, dtype=np.int64)
        self.valid_pts = np.zeros((0, 3), dtype=np.float32)
        self.ok = self.dropped = 0
        self.last_warn = 0.0
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 100)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.cloud_cb,
                                 # Reliable: con best-effort se pierden casi todas las nubes (fragmentadas);
                                 # el publicador real y el bag son reliable.
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.pub = self.create_publisher(PointCloud2, '/laika/map3d', QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/laika/map3d/save', self.save_cb)
        self.create_timer(1.0 / g('merge_rate'), self.merge_pending)
        self.create_timer(1.0 / g('publish_rate'), self.publish)
        self.create_timer(10.0, lambda: self.get_logger().info(
            'nubes procesadas=%d descartadas(sin TF)=%d vóxeles válidos=%d'
            % (self.ok, self.dropped, len(self.valid_pts))))

    def tf_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                tr = t.transform.translation
                self.tf = (tr.x, tr.y, yaw_of(t.transform.rotation), time.monotonic())

    def cloud_cb(self, msg):
        now = time.monotonic()
        if self.tf is None or now - self.tf[3] > self.tf_timeout:
            self.dropped += 1
            if now - self.last_warn > 5.0:
                self.last_warn = now
                self.get_logger().warn('sin map->odom reciente: nube descartada')
            return
        x, y, yaw, _ = self.tf
        if self.prev_used is not None:
            px, py, pyaw, _ = self.prev_used
            dyaw = abs(math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)))
            if math.hypot(x - px, y - py) > self.jump_dist or dyaw > self.jump_rad:
                self.get_logger().warn('map->odom saltó (cierre de lazo?): lo ya acumulado no se corrige')
        self.prev_used = self.tf
        dt = np.dtype({'names': [f.name for f in msg.fields[:3]],
                       'formats': [_NP[f.datatype] for f in msg.fields[:3]],
                       'offsets': [f.offset for f in msg.fields[:3]],
                       'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)]
        p = transform_2d(xyz, yaw, x, y)
        self.pending.append(np.unique(pack_keys(np.floor(p / self.vox).astype(np.int64))))
        self.ok += 1

    def merge_pending(self):
        if not self.pending:
            return
        self.gk, self.gc = merge(self.gk, self.gc, np.concatenate(self.pending))
        self.pending = []
        v = unpack_keys(self.gk[self.gc >= self.min_views])
        self.valid_pts = ((v + 0.5) * self.vox).astype(np.float32)

    def publish(self):
        if len(self.valid_pts) == 0:
            return
        m = PointCloud2()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, len(self.valid_pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')]
        m.is_bigendian, m.point_step, m.is_dense = False, 12, True
        m.row_step = 12 * m.width
        m.data = self.valid_pts.tobytes()
        self.pub.publish(m)

    def save(self):
        self.merge_pending()
        os.makedirs(os.path.dirname(self.pcd_path) or '.', exist_ok=True)
        write_pcd(self.pcd_path, self.valid_pts)
        return len(self.valid_pts)

    def save_cb(self, _req, res):
        try:
            n = self.save()
            res.success, res.message = True, '%d puntos en %s' % (n, self.pcd_path)
        except Exception as e:  # noqa
            res.success, res.message = False, str(e)
        return res


def main():
    rclpy.init()
    node = Map3dAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('guardado al cerrar: %d puntos' % node.save())
        except Exception as e:  # noqa
            print('fallo al guardar:', e)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
