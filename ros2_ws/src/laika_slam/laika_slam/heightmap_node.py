import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

_NP = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2',
       PointField.UINT16: 'u2', PointField.INT32: 'i4', PointField.UINT32: 'u4',
       PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class HeightGrid:
    """Rejilla alineada con odom. Sin np.add.at (numpy 1.17 en el robot)."""

    def __init__(self, res=0.05, half=4.0, alpha=0.15, stale=3.0, band=0.5, memory=2.0,
                 min_pts=5, max_h=1.5, foot=(0.38, 0.18), recentre=1.0):
        self.res, self.n = res, int(round(2 * half / res))
        self.pct = 0.2
        self.alpha, self.stale, self.band, self.memory = alpha, stale, band, memory
        self.min_pts, self.max_h, self.foot, self.recentre = min_pts, max_h, foot, recentre
        self.ox = self.oy = None  # índice global (celdas) de la celda [0,0]
        self._alloc()

    def _alloc(self):
        n2 = self.n * self.n
        self.ground = np.full(n2, np.nan)
        self.g_t = np.full(n2, -np.inf)
        self.top = np.full(n2, np.nan)
        self.top_t = np.full(n2, -np.inf)
        self.over = np.full(n2, np.nan)
        self.over_t = np.full(n2, -np.inf)
        self.count = np.zeros(n2, dtype=np.int64)

    def _shift(self, dx, dy):
        """Desplaza la rejilla (dx, dy) celdas enteras; lo nuevo queda NaN."""
        old = (self.ground, self.g_t, self.top, self.top_t, self.over, self.over_t, self.count)
        self._alloc()
        new = (self.ground, self.g_t, self.top, self.top_t, self.over, self.over_t, self.count)
        n = self.n
        ys, xs = slice(max(0, -dy), min(n, n - dy)), slice(max(0, -dx), min(n, n - dx))
        yd, xd = slice(max(0, dy), min(n, n + dy)), slice(max(0, dx), min(n, n + dx))
        # dato global = viejo[i] ; nuevo índice = i - shift, con shift = (dx, dy) de origen
        for o, nw in zip(old, new):
            nw.reshape(n, n)[ys, xs] = o.reshape(n, n)[yd, xd]

    def centre(self, px, py):
        cx, cy = int(math.floor(px / self.res)), int(math.floor(py / self.res))
        if self.ox is None:
            self.ox, self.oy = cx - self.n // 2, cy - self.n // 2
            return
        if math.hypot((self.ox + self.n // 2 - cx), (self.oy + self.n // 2 - cy)) * self.res > self.recentre:
            nx, ny = cx - self.n // 2, cy - self.n // 2
            self._shift(nx - self.ox, ny - self.oy)
            self.ox, self.oy = nx, ny

    def update(self, xyz, pose, t):
        """xyz: (N,3) en odom; pose=(x, y, z, yaw); t en segundos."""
        px, py, pz, yaw = pose
        self.centre(px, py)
        # el L1 rellena ~90 % de la nube con puntos (0,0,0): inválidos
        xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] <= pz + self.max_h) & (xyz != 0).any(axis=1)]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = xyz[:, 0] - px, xyz[:, 1] - py
        bx, by = c * dx + s * dy, -s * dx + c * dy
        xyz = xyz[(np.abs(bx) > self.foot[0]) | (np.abs(by) > self.foot[1])]
        ix = np.floor(xyz[:, 0] / self.res).astype(np.int64) - self.ox
        iy = np.floor(xyz[:, 1] / self.res).astype(np.int64) - self.oy
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        if not ok.any():
            return 0
        idx, z = iy[ok] * self.n + ix[ok], xyz[ok, 2]
        o = np.lexsort((z, idx))  # por celda, z ascendente
        idx, z = idx[o], z[o]
        cells, start, cnt = np.unique(idx, return_index=True, return_counts=True)
        self.count[cells] = cnt
        g = cnt >= self.min_pts
        gc, third = cells[g], z[start[g] + self.min_pts - 1]
        old = self.ground[gc]
        fresh = np.isnan(old) | (t - self.g_t[gc] > self.stale)
        self.ground[gc] = np.where(fresh, third, old + self.alpha * (third - old))
        self.g_t[gc] = t
        gnd = self.ground[idx]
        dz = z - gnd  # NaN donde no hay suelo: las comparaciones dan False
        for lo_hi, arr, tarr, take_max in (((0.0, self.band), self.top, self.top_t, True),
                                           ((self.band, np.inf), self.over, self.over_t, False)):
            m = (dz > lo_hi[0]) & (dz <= lo_hi[1]) if take_max else dz > lo_hi[0]
            if not m.any():
                continue
            ci, cz = idx[m], z[m]
            u, st, ct = np.unique(ci, return_index=True, return_counts=True)
            arr[u] = cz[st + ct - 1] if take_max else cz[st]
            tarr[u] = t
        return len(xyz)

    def expire(self, t):
        self.top[t - self.top_t > self.memory] = np.nan
        self.over[t - self.over_t > self.memory] = np.nan

    def cloud(self, t):
        """(M,5) x, y, ground, top, overhead (NaN si no hay)."""
        self.expire(t)
        v = np.nonzero(~np.isnan(self.ground))[0]
        iy, ix = np.divmod(v, self.n)
        return np.stack([(ix + self.ox + 0.5) * self.res, (iy + self.oy + 0.5) * self.res,
                         self.ground[v], self.top[v], self.over[v]], axis=1)

    def floor_stats(self, px, py, radius, band=0.07):
        """(MAD*1.4826 del residuo de un plano ajustado, altura media del suelo) en las celdas dentro
        de radius y a +-band de la mediana. Con el L1 las paredes verticales también dan 'suelo' alto
        y sin top (no hay puntos más bajos en su celda), por eso no sirve filtrar por top-ground."""
        v = np.nonzero(~np.isnan(self.ground))[0]
        iy, ix = np.divmod(v, self.n)
        x, y = (ix + self.ox + 0.5) * self.res, (iy + self.oy + 0.5) * self.res
        gz = self.ground[v]
        k = np.hypot(x - px, y - py) <= radius
        if k.sum() < 10:
            return float('nan'), float('nan')
        k &= np.abs(gz - np.median(gz[k])) <= band
        if k.sum() < 10:
            return float('nan'), float('nan')
        A = np.column_stack([x[k], y[k], np.ones(k.sum())])
        r = gz[k] - A.dot(np.linalg.lstsq(A, gz[k], rcond=None)[0])
        return float(1.4826 * np.median(np.abs(r - np.median(r)))), float(gz[k].mean())


def cloud_to_xyz(msg):
    dt = np.dtype({'names': [f.name for f in msg.fields[:3]],
                   'formats': [_NP[f.datatype] for f in msg.fields[:3]],
                   'offsets': [f.offset for f in msg.fields[:3]],
                   'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)


class HeightmapNode(Node):
    def __init__(self):
        super().__init__('heightmap_node')
        for n, d in (('resolution', 0.05), ('half_size', 4.0), ('recentre_dist', 1.0), ('rate', 5.0),
                     ('max_height', 1.5), ('foot_x', 0.38), ('foot_y', 0.18), ('ground_alpha', 0.15),
                     ('ground_stale', 3.0), ('band', 0.5), ('memory', 2.0), ('min_pts', 5),
                     ('stats_radius', 4.0)):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.grid = HeightGrid(g('resolution'), g('half_size'), g('ground_alpha'), g('ground_stale'),
                               g('band'), g('memory'), int(g('min_pts')), g('max_height'),
                               (g('foot_x'), g('foot_y')), g('recentre_dist'))
        self.radius = g('stats_radius')
        self.pose = None
        self.pending = []
        self.ms = []
        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, rel)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.cloud_cb, rel)
        self.pub = self.create_publisher(PointCloud2, '/laika/heightmap_cloud', 1)
        self.create_timer(1.0 / g('rate'), self.tick)
        self.create_service(Trigger, '~/reset', self.reset_cb)
        self.create_timer(10.0, self.log)

    def reset_cb(self, _req, res):
        self.grid.ox = self.grid.oy = None
        self.grid._alloc()
        self.pending = []
        res.success, res.message = True, 'heightmap vacio'
        return res

    def odom_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))

    def cloud_cb(self, msg):
        if self.pose is not None:  # fuera de lotes: sin pose no se puede filtrar el cuerpo
            self.pending.append(cloud_to_xyz(msg))

    def tick(self):
        if not self.pending:
            return
        t0 = time.monotonic()
        xyz, self.pending = np.concatenate(self.pending), []
        self.grid.update(xyz, self.pose, t0)
        out = self.grid.cloud(t0).astype(np.float32)
        m = PointCloud2()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = 1, len(out)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z', 'top', 'overhead'))]
        m.is_bigendian, m.point_step, m.is_dense = False, 20, False
        m.row_step = 20 * m.width
        m.data = out.tobytes()
        self.pub.publish(m)
        self.ms.append((time.monotonic() - t0) * 1e3)

    def log(self):
        ms, self.ms = self.ms, []
        if not ms or self.pose is None:
            return
        mad, gmean = self.grid.floor_stats(self.pose[0], self.pose[1], self.radius)
        self.get_logger().info('ms/lote media=%.1f max=%.1f celdas=%d suelo_mad=%.2f cm suelo_medio=%.3f odom_z=%.3f' % (
            sum(ms) / len(ms), max(ms), int((~np.isnan(self.grid.ground)).sum()), 100 * mad, gmean,
            self.pose[2]))


def main():
    rclpy.init()
    node = HeightmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
