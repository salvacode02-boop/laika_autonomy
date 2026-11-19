#!/usr/bin/env python3
"""B3 (P5.2-A): rotacion del acelerometro del L1. Uso: analizar_acel.py <ruta_bag>
Solo lee el .db3. Candidatas: B y Rz(pi)*B. Tres pruebas con el residuo r = a_b - R_o^T g
(brazo de palanca, curvas, arranques/paradas). Sale con 1 si no hay decision."""
import glob, os, sqlite3, sys
import numpy as np
from analizar_imu import leer, st, CAND

CANDS = {'B': CAND['B'], 'RzB': np.diag([-1.0, -1.0, 1.0]) @ CAND['B']}
T_STATIC, T_SPIN = (1.0, 36.0), (37.6, 90.3)  # s desde el inicio del bag (de fase2_cuarto1)
LEVER_X = 0.28945


def lp(x, fs, fc):  # media movil ~ pasabajas
    k = max(1, int(fs / fc))
    return np.apply_along_axis(lambda c: np.convolve(c, np.ones(k) / k, mode='same'), 0, x)


def quat_R(q):  # q: (n,4) xyzw -> (n,3,3)
    x, y, z, w = q.T
    return np.stack([1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w),
                     2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
                     2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)], axis=1).reshape(-1, 3, 3)


def reg(x, y):
    """pendiente, t-stat, N (con intercepto)"""
    n = len(x)
    if n < 4 or np.std(x) < 1e-9:
        return np.nan, np.nan, n
    A = np.column_stack([x, np.ones(n)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ c
    se = np.sqrt(res @ res / (n - 2) / ((x - x.mean()) ** 2).sum())
    return c[0], c[0] / se if se > 0 else np.inf, n


def windows(t, sig, t0, t1, w=1.0):
    """medias de cada señal en ventanas de w s dentro de [t0,t1]"""
    out = []
    for s in np.arange(t0, t1 - w, w):
        m = (t >= s) & (t < s + w)
        if m.sum() > 5:
            out.append([f[m].mean() for f in sig] + [s])
    return np.array(out)


IMU_A = np.array([[0.9655, 0, 0.2605], [0, -1, 0], [0.2605, 0, -0.9655]])  # imu_R de l1_to_body.yaml (A)


def tilt(bag, thr=0.1):
    """Cabeceo/alabeo de ~10 deg: angulos del acel por candidata vs giro (derivada) y vs odom."""
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
    imu = list(leer(db, '/utlidar/imu'))
    od = list(leer(db, '/utlidar/robot_odom'))
    ti = np.array([st(m.header) for m in imu]); to = np.array([st(m.header) for m in od])
    t0 = ti[0]; ti, to = ti - t0, to - t0
    a_raw = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    w = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu]) @ IMU_A.T
    fs = 1 / np.median(np.diff(ti))
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in od])
    Ro = quat_R(q)
    o_roll = np.interp(ti, to, np.unwrap(np.arctan2(Ro[:, 2, 1], Ro[:, 2, 2])))
    o_pitch = np.interp(ti, to, -np.arcsin(np.clip(Ro[:, 2, 0], -1, 1)))
    wm = lp(np.hypot(w[:, 0], w[:, 1])[:, None], fs, 2.0)[:, 0] > thr
    idx = np.flatnonzero(wm)
    segs = []
    for k in idx:  # une huecos < 2 s
        if segs and ti[k] - segs[-1][1] < 2.0:
            segs[-1][1] = ti[k]
        else:
            segs.append([ti[k], ti[k]])
    segs = [(a - 1.0, b + 1.0) for a, b in segs if b - a > 0.5]
    print(f'IMU {len(ti)} msgs fs {fs:.0f} Hz; umbral |w_xy| {thr} rad/s; segmentos de giro:')
    for a, b in segs:
        print(f'  {a:7.1f} - {b:7.1f} s  ({b-a:.1f} s)')
    if not segs:
        print('INCONCLUSO: sin segmentos'); sys.exit(1)
    m = np.zeros(len(ti), bool)
    for a, b in segs:
        m |= (ti >= a) & (ti <= b)
    ok_all = {}
    for n, R in CANDS.items():
        ab = a_raw @ R.T
        roll = np.arctan2(ab[:, 1], ab[:, 2])
        pitch = np.arctan2(-ab[:, 0], np.hypot(ab[:, 1], ab[:, 2]))
        print(f'\n{n}:')
        ok = True
        for nm, ang, gi, oa in (('roll', roll, 0, o_roll), ('pitch', pitch, 1, o_pitch)):
            f = lp(ang[:, None], fs, 2.0)[:, 0]
            d = np.gradient(f, ti)
            gl = lp(w[:, gi:gi + 1], fs, 2.0)[:, 0]
            c1 = np.corrcoef(d[m], gl[m])[0, 1]; s1 = reg(gl[m], d[m])[0]
            c2 = np.corrcoef(f[m], oa[m])[0, 1]; s2 = reg(oa[m], f[m])[0]
            print(f'  {nm}: d(ang)/dt vs gyro  corr {c1:+.3f} pend {s1:+.3f} | ang vs odom  corr {c2:+.3f} pend {s2:+.3f}')
            ok &= bool(s1 > 0.7 and c1 > 0.8)
        ok_all[n] = ok
    win = [n for n, v in ok_all.items() if v]
    if len(win) != 1:
        print(f'INCONCLUSO: ganadoras {win} (se exige pend>0.7 y corr>0.8 en roll y pitch)'); sys.exit(1)
    print(f'\nElegida: {win[0]}; acc_R = {CANDS[win[0]].ravel().round(4).tolist()}')


def main():
    if sys.argv[1] == '--tilt':
        return tilt(sys.argv[2])
    db = sqlite3.connect(glob.glob(os.path.join(sys.argv[1], '*.db3'))[0])
    imu = list(leer(db, '/utlidar/imu'))
    ti = np.array([st(m.header) for m in imu])
    a_raw = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    od = list(leer(db, '/utlidar/robot_odom'))
    to = np.array([st(m.header) for m in od])
    t0 = to[0]
    ti, to = ti - t0, to - t0
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in od])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m in od])
    wz_o = np.array([m.twist.twist.angular.z for m in od])
    # quaternion interpolado a la IMU: nlerp con signo continuo
    q *= np.where((q[:, 3] < 0)[:, None], -1, 1)
    qi = np.column_stack([np.interp(ti, to, q[:, j]) for j in range(4)])
    qi /= np.linalg.norm(qi, axis=1, keepdims=True)
    Ro = quat_R(qi)
    yaw = np.unwrap(np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
    fs_o = 1 / np.median(np.diff(to))
    fs = 1 / np.median(np.diff(ti))
    if np.abs(wz_o).max() < 1e-3:
        wz_o = np.gradient(yaw, to)
        print('AVISO: twist.angular.z en cero; omega_z derivado del yaw')
    v_w = lp(np.gradient(pos, to, axis=0), fs_o, 1.0)
    v_bx = np.cos(yaw) * v_w[:, 0] + np.sin(yaw) * v_w[:, 1]
    v_by = -np.sin(yaw) * v_w[:, 0] + np.cos(yaw) * v_w[:, 1]
    I = lambda x: np.interp(ti, to, x)
    vbx, vby, wz = I(v_bx), I(v_by), I(wz_o)
    mst = (ti >= T_STATIC[0]) & (ti <= T_STATIC[1])
    mwalk = (np.hypot(vbx, vby) > 0.1) | (np.abs(wz) > 0.2)
    print(f'IMU {len(ti)} msgs fs {fs:.0f} Hz; odom fs {fs_o:.0f} Hz; |v| max {np.hypot(vbx, vby).max():.2f}')

    verdict = {}
    for n, R in CANDS.items():
        ab = a_raw @ R.T
        g = np.linalg.norm(ab[mst].mean(0))
        r = ab - np.einsum('nji,j->ni', Ro, [0, 0, g])  # R_o^T g
        r -= r[mst].mean(0)
        # Test 1: brazo de palanca
        W = windows(ti, [r[:, 0], wz ** 2], *T_SPIN)
        s1 = reg(W[:, 1], W[:, 0])
        # Test 2: curvas
        W = windows(ti, [r[:, 1], vbx, wz, vbx * wz], T_SPIN[0], ti[-1])
        W = W[(W[:, 1] > 0.15) & (np.abs(W[:, 2]) > 0.2)]
        s2 = reg(W[:, 3], W[:, 0])
        # Test 3: arranques/paradas, |v_b| cruza 0.1 m/s
        sp = np.hypot(vbx, vby)
        cr = np.flatnonzero(np.diff((sp > 0.1).astype(int)) != 0)
        dX, I1, ok = [], [], 0
        last = -9.0
        for k in cr:
            tk = ti[k]
            if tk - last < 3.0 or tk < 1.5 or tk > ti[-1] - 1.5:
                continue
            last = tk
            m = (ti >= tk - 1.5) & (ti <= tk + 1.5)
            I1.append(np.trapz(r[m, 0], ti[m]))
            dX.append(np.interp(tk + 1.5, ti, vbx) - np.interp(tk - 1.5, ti, vbx))
        dX, I1 = np.array(dX), np.array(I1)
        s3 = reg(dX, I1)
        frac = np.mean(np.sign(I1) == np.sign(dX)) if len(dX) else np.nan
        rw = lp(r, fs, 1.0)[mwalk & ~((ti >= T_SPIN[0]) & (ti <= T_SPIN[1]))]
        rms = np.sqrt((rw ** 2).sum(1).mean()) if len(rw) else np.nan
        verdict[n] = (s1, s2, s3, frac, rms)
        print(f'\n{n}: |a_static| {g:.3f}')
        print(f'  T1 palanca  r_x vs w^2  : pend {s1[0]:+.3f} (esp. -0.29) t {s1[1]:+.1f} N {s1[2]}')
        print(f'  T2 curvas   r_y vs vx*w : pend {s2[0]:+.3f} (esp. +1)    t {s2[1]:+.1f} N {s2[2]}')
        print(f'  T3 arr/par  int r_x vs dv: pend {s3[0]:+.3f} (esp. +1)    t {s3[1]:+.1f} N {s3[2]}  mismo signo {frac*100:.0f} %')
        print(f'  RMS de r (marcha, lp 1 Hz): {rms:.3f} m/s2')

    def votos(n):  # +1 a favor, -1 en contra, 0 no significativo
        s1, s2, s3, frac, _ = verdict[n]
        out = []
        for sl, t, ok in ((-s1[0], -s1[1], -0.45 < s1[0] < -0.15), (s2[0], s2[1], True), (s3[0], s3[1], frac >= 0.7)):
            out.append(1 if (t > 3 and ok) else (-1 if t < -3 else 0))
        return out
    best = None
    for n in CANDS:
        v = votos(n)
        print(f'votos {n}: {v}')
        if sum(x == 1 for x in v) >= 2 and -1 not in v:
            best = n
    if best is None:
        print('INCONCLUSO: sin candidata con >=2 pruebas a favor (|t|>3) y ninguna en contra')
        sys.exit(1)
    rms = verdict[best][4]
    print(f'Elegida: {best}; acc_R = {CANDS[best].ravel().round(4).tolist()}')
    if not rms < 1.0:
        print(f'AVISO: RMS de r {rms:.2f} no << 1 m/s2: sospechar de la convencion roll/pitch de la odom')
    # sin salida cero adicional: exit 0


if __name__ == '__main__':
    main()
