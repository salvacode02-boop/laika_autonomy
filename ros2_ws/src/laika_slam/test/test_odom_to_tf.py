import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.odom_to_tf import inject_jump, quat_yaw, rebase_quat, rebase_xy


def q_of(yaw, pitch=0.1):  # Rz(yaw)*Ry(pitch)
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def test_reset_and_jump_roundtrip():
    raw = (3.0, -2.0, 0.7)
    q = q_of(raw[2])
    assert abs(quat_yaw(*q) - raw[2]) < 1e-9
    ref = raw  # reset en la pose actual: identidad, z/pitch intactos
    x, y = rebase_xy(raw[0], raw[1], ref)
    q2 = rebase_quat(q, ref)
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(quat_yaw(*q2)) < 1e-9
    assert abs(sum(v * v for v in q2) - 1) < 1e-9
    # un punto a 1 m de frente (en el marco del robot) queda en (1, 0)
    px, py = raw[0] + math.cos(raw[2]), raw[1] + math.sin(raw[2])
    bx, by = rebase_xy(px, py, ref)
    assert abs(bx - 1) < 1e-9 and abs(by) < 1e-9
    # salto de yaw: posicion publicada igual, yaw +20 deg; segundo reset deja identidad otra vez
    cur = (3.5, -1.0, 1.2)
    ref2 = inject_jump(ref, cur[:2], 20.0)
    before, after = rebase_xy(cur[0], cur[1], ref), rebase_xy(cur[0], cur[1], ref2)
    assert abs(before[0] - after[0]) < 1e-9 and abs(before[1] - after[1]) < 1e-9
    qc = q_of(cur[2])
    dyaw = wrap(quat_yaw(*rebase_quat(qc, ref2)) - quat_yaw(*rebase_quat(qc, ref)))
    assert abs(abs(dyaw) - math.radians(20)) < 1e-9
    xr, yr = rebase_xy(cur[0], cur[1], cur)
    assert abs(xr) < 1e-9 and abs(yr) < 1e-9
