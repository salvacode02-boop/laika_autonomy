#!/usr/bin/env python3
"""Calibracion offline de la IMU del L1 (paso 3, fase 2). Uso: analizar_imu.py <ruta_bag>
Solo lee el .db3; no publica nada. Sale con 1 si ninguna candidata (A/B) da corr_z >= 0.9."""
import glob, os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

c, s = np.cos(np.radians(15.1)), np.sin(np.radians(15.1))
CAND = {'A': np.array([[c, 0, s], [0, -1, 0], [s, 0, -c]]),
        'B': np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])}


def leer(db, topic, maxn=None):
    cur = db.execute('SELECT type FROM topics WHERE name=?', (topic,))
    msg_t = get_message(cur.fetchone()[0].replace('/msg/', '/msg/'))
    q = ('SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) '
         'ORDER BY timestamp' + (f' LIMIT {maxn}' if maxn else ''))
    for (d,) in db.execute(q, (topic,)):
        yield deserialize_message(bytes(d), msg_t)


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    bag = sys.argv[1]
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])

    imu = list(leer(db, '/utlidar/imu'))
    ti = np.array([st(m.header) for m in imu])
    w_raw = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu])
    a_raw = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    od = list(leer(db, '/utlidar/robot_odom'))
    to = np.array([st(m.header) for m in od])
    tw = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in od])
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in od])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in od])
    yaw = np.unwrap(np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]),
                               1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    yr_q = np.gradient(yaw, to)  # yaw rate derivado de la orientacion
    usa_twist = np.abs(tw[:, 2]).max() > 1e-3
    yr = tw[:, 2] if usa_twist else yr_q
    print(f'odom: twist.angular {"con datos" if usa_twist else "EN CERO -> yaw rate derivado de la orientacion"}; '
          f'max|twist.z|={np.abs(tw[:,2]).max():.3f}, max|yr_q|={np.abs(yr_q).max():.3f}')

    # Segmentacion con odom: movimiento = |yaw rate| suavizado > 0.1 rad/s.
    k = max(1, int(1.0 / np.median(np.diff(to))))
    sm = np.convolve(np.abs(yr_q), np.ones(k) / k, mode='same')
    act = sm > 0.1
    idx = np.flatnonzero(act)
    ini = idx[0]
    # giros = desde el primer movimiento hasta que el robot sale a >0.5 m de su sitio (con pausas internas)
    lejos = np.flatnonzero(np.linalg.norm(pos - pos[ini], axis=1) > 0.5)
    lim = lejos[0] if len(lejos) else len(to)
    fin = idx[idx < lim][-1]
    t_q0, t_q1 = to[0] + 1.0, to[ini] - 1.0       # quieto inicial
    t_g0, t_g1 = to[ini], to[fin]                 # giros
    print(f'tramos (s desde inicio): quieto {t_q0-to[0]:.1f}-{t_q1-to[0]:.1f}, '
          f'giros {t_g0-to[0]:.1f}-{t_g1-to[0]:.1f}; desplaz. max en giros '
          f'{np.ptp(pos[ini:fin+1], axis=0).max():.2f} m; yaw odom en giros {yaw[fin]-yaw[ini]:.3f} rad (4pi=12.566)')
    mq = (ti >= t_q0) & (ti <= t_q1)
    mg = (ti >= t_g0) & (ti <= t_g1)
    yr_i = np.interp(ti, to, yr)
    tw_i = np.column_stack([np.interp(ti, to, tw[:, j]) for j in range(3)])
    if not usa_twist:
        tw_i = np.column_stack([np.zeros_like(ti), np.zeros_like(ti), yr_i])

    res, ok = {}, False
    ydo = yaw[fin] - yaw[ini]
    for n, R in CAND.items():
        wb = w_raw @ R.T
        bias = wb[mq].mean(axis=0)
        cor = [np.corrcoef(wb[mg][:, j], tw_i[mg][:, j])[0, 1] if tw_i[mg][:, j].std() > 1e-6 else np.nan
               for j in range(3)]
        wz = wb[:, 2] - bias[2]
        integ = np.trapz(wz[mg], ti[mg])
        m = (wb[mg] - bias).mean(axis=0)
        acc = R @ a_raw[mq].mean(axis=0)
        res[n] = dict(cor=cor, bias=bias, integ=integ, z2x=-m[0] / m[2], z2y=-m[1] / m[2], acc=acc)
        ok |= cor[2] >= 0.9

    dt = np.diff(ti)
    print('\n=== IMU ===')
    print(f'msgs {len(ti)}; periodo medio {dt.mean()*1e3:.3f} ms, max {dt.max()*1e3:.1f} ms, '
          f'huecos>0.02 s: {(dt>0.02).sum()}; w_raw quieta {w_raw[mq].mean(0).round(4)}; '
          f'|a| quieta {np.linalg.norm(a_raw[mq].mean(0)):.3f} (m/s2 si ~9.8)')
    print(f'\n{"":24s}{"A":>26s}{"B":>26s}')
    row = lambda name, f: print(f'{name:24s}' + ''.join(f'{f(res[n]):>26s}' for n in "AB"))
    fa = lambda v: np.array2string(np.asarray(v), precision=4, suppress_small=True)
    row('corr (x, y, z)', lambda r: fa(r['cor']))
    row('integral w.z giros [rad]', lambda r: f"{r['integ']:.3f}")
    row('integral / yaw odom', lambda r: f"{r['integ']/ydo:.3f} (esperado 1.00+-0.05)")
    row('gyro_bias (body)', lambda r: fa(r['bias']))
    row('ang_z2x_proj', lambda r: f"{r['z2x']:.4f}")
    row('ang_z2y_proj', lambda r: f"{r['z2y']:.4f}")
    row('acc quieta en body', lambda r: fa(r['acc']))
    for n in "AB":
        for nm, v in (('z2x', res[n]['z2x']), ('z2y', res[n]['z2y'])):
            if abs(v) > 0.3:
                print(f'AVISO: cand {n} |{nm}|={abs(v):.2f} > 0.3 -> imu_R sospechosa')

    # Nubes: sincronia con la IMU y stamp inicio/fin del barrido
    cl = list(leer(db, '/utlidar/cloud', 40))
    tc = np.array([st(m.header) for m in cl])
    print('\n=== NUBE ===')
    print(f'primer stamp nube - IMU mas cercana: {(tc[0]-ti[np.abs(ti-tc[0]).argmin()])*1e3:.2f} ms; '
          f'primer stamp nube - primer stamp IMU: {tc[0]-ti[0]:.3f} s; delta entre nubes {np.diff(tc).mean():.4f} s')
    m = cl[5]
    f = {x.name: x for x in m.fields}
    print('campos:', [(x.name, x.datatype) for x in m.fields], 'frame', m.header.frame_id)
    if 'time' in f:
        dts = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
        mx = []
        for mm in cl[:30]:
            fl = {x.name: x for x in mm.fields}['time']
            dt_ = np.dtype(dts[fl.datatype]).newbyteorder('>' if mm.is_bigendian else '<')
            raw = np.frombuffer(bytes(mm.data), dtype=np.uint8).reshape(-1, mm.point_step)
            v = raw[:, fl.offset:fl.offset + dt_.itemsize].copy().view(dt_).ravel().astype(float)
            mx.append((v.min(), v.max()))
        mx = np.array(mx)
        print(f'campo time por punto: min {mx[:,0].mean():.4f}, max {mx[:,1].mean():.4f} (unidad del campo, ver datatype)')
        print(f'-> stamp = {"INICIO" if mx[:,0].mean() >= -1e-6 and abs(mx[:,1].mean()-np.diff(tc).mean()) < 0.02 else "FIN (o revisar)"} '
              'del barrido (time por punto >= 0 y max ~ delta entre nubes)')
    if not ok:
        print('\nNINGUNA candidata cumple corr_z >= 0.9')
        sys.exit(1)


main()
