import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
# Importar solo las funciones puras; rclpy debe estar disponible (source del entorno ROS).
from laika_slam.map3d_accumulator import (pack_keys, unpack_keys, transform_2d,
                                          write_pcd, read_pcd, merge)

ijk = np.array([[-1, -5, 3], [0, 0, 0], [1000, -1000, -10], [-(1 << 20), (1 << 20) - 1, 7]], dtype=np.int64)
k = pack_keys(ijk)
assert len(set(k.tolist())) == 4 and (unpack_keys(k) == ijk).all()

p = transform_2d(np.array([[1.0, 0.0, 0.7]]), math.pi / 2, 2.0, 3.0)
assert np.allclose(p, [[2.0, 4.0, 0.7]])

d = tempfile.mkdtemp()
f = os.path.join(d, 'a.pcd')
pts = np.array([[1, 2, 3], [-1.5, 0, 0.25]], dtype=np.float32)
write_pcd(f, pts)
hdr = open(f, 'rb').read().split(b'DATA binary\n')[0]
assert b'POINTS 2' in hdr and os.path.getsize(f) == len(hdr) + len(b'DATA binary\n') + 2 * 12
assert (read_pcd(f) == pts).all() and not os.path.exists(f + '.tmp')

gk, gc = merge(np.zeros(0, np.int64), np.zeros(0, np.int64), np.array([1, 5], np.int64))
gk, gc = merge(gk, gc, np.array([5, 9], np.int64))
assert gk.tolist() == [1, 5, 9] and gc.tolist() == [1, 2, 1]
print('OK')
