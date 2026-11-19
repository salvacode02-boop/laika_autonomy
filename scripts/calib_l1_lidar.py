#!/usr/bin/env python3
"""Calibra rotaciones accel->lidar y gyro->lidar del L1 usando el piso visto por el lidar como
referencia de gravedad. Uso: calib_l1_lidar.py <bag_tilt> [<bag_giros_corr_z>]. Solo lee .db3.
Imprime imu_R / acc_R en la convencion de l1_to_body (-> body = cloud_R @ R_x->lidar). Sale 1 si falla el gate."""
import glob, os, sqlite3, sys
import numpy as np
from analizar_imu import leer, st
from analizar_acel import lp

CLOUD_R = np.array([[-0.9655, 0, 0.2605], [0, 1, 0], [-0.2605, 0, -0.9655]])  # l1_to_body.yaml
A_CUR = np.array([[0.9655, 0, 0.2605], [0, -1, 0], [0.2605, 0, -0.9655]])
c_, s_ = np.cos(np.radians(15.1)), np.sin(np.radians(15.1))
B_CUR = np.array([[c_, 0, -s_], [0, 1, 0], [s_, 0, c_]])
UP_NOM = CLOUD_R.T @ [0, 0, 1.0]  # "arriba" nominal en el marco del lidar (solo para elegir puntos y signo)


def kabsch(P, Q):  # R tal que R P ~ Q (filas); devuelve R y residuo rms (grados)
    U, _, Vt = np.linalg.svd(P.T @ Q)
    R = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
    e = np.degrees(np.arccos(np.clip(np.sum((P @ R.T) * Q, 1), -1, 1)))
    return R, np.sqrt(np.mean(e ** 2))


def ang(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def expm(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def logv(R):
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def cloud_xyz(m):
    d = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
    p = d[:, :12].copy().view('<f4').reshape(-1, 3).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def floor_normal(pts):
    """plano del piso: puntos a <1.5 m (horizontal nominal), los mas bajos; LSQ con rechazo."""
    h = pts - np.outer(pts @ UP_NOM, UP_NOM)
    z = pts @ UP_NOM
    for lo, hi in ((0.0, 1.5), (0.4, 1.2)):
        m = (np.linalg.norm(h, axis=1) < hi) & (np.linalg.norm(h, axis=1) > lo) & (z < np.percentile(z, 30) + 0.12)
        P = pts[m]
        keep = np.ones(len(P), bool)
        for _ in range(6):
            c = P[keep].mean(0)
            n = np.linalg.svd(P[keep] - c, full_matrices=False)[2][2]
            r = (P - c) @ n
            keep = np.abs(r) < max(0.01, 2.5 * r[keep].std())
        res = r[keep].std()
        if n @ UP_NOM < 0:
            n = -n
        if res <= 0.01:
            break
    return n, res, keep.sum()


def main():
    bag = sys.argv[1]
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
    imu = list(leer(db, '/utlidar/imu'))
    ti = np.array([st(m.header) for m in imu])
    w = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu])
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    fs = 1 / np.median(np.diff(ti))
    # 1. mesetas
    q = lp(np.linalg.norm(w, axis=1)[:, None], fs, 10.0)[:, 0] < 0.05
    runs, k = [], 0
    while k < len(q):
        if q[k]:
            j = k
            while j < len(q) and q[j]:
                j += 1
            if ti[j - 1] - ti[k] >= 1.0:
                runs.append((k, j))
            k = j
        else:
            k += 1
    # unir mesetas que son la misma pose (gravedad igual a <0.5 deg) separadas por <3 s sin giro real: se dejan separadas
    print(f'{len(runs)} mesetas >=1 s con |w|<0.05')
    ct = [(st(m.header), m) for m in leer(db, '/utlidar/cloud')]
    cts = np.array([t for t, _ in ct])
    pl = []
    for k, j in runs:
        k2, j2 = k + int(0.3 * fs), j - int(0.3 * fs)  # recorta bordes
        if j2 - k2 < 0.3 * fs:
            continue
        t0, t1 = ti[k2], ti[j2 - 1]
        idx = np.flatnonzero((cts >= t0) & (cts <= t1))[:25]
        if len(idx) < 3:
            continue
        pts = np.vstack([cloud_xyz(ct[i][1]) for i in idx])
        n, res, nn = floor_normal(pts)
        h = len(idx) // 2
        n1, n2 = floor_normal(np.vstack([cloud_xyz(ct[i][1]) for i in idx[:h]]))[0], floor_normal(np.vstack([cloud_xyz(ct[i][1]) for i in idx[h:]]))[0]
        am1, am2 = a[k2:(k2 + j2) // 2].mean(0), a[(k2 + j2) // 2:j2].mean(0)
        dn = np.degrees(np.arccos(np.clip(n1 @ n2, -1, 1)))
        da = np.degrees(np.arccos(np.clip(am1 @ am2 / np.linalg.norm(am1) / np.linalg.norm(am2), -1, 1)))
        am = a[k2:j2].mean(0)
        pl.append(dict(dn=dn, da=da, t0=t0, t1=t1, n=n, res=res, a=am / np.linalg.norm(am), wm=w[k2:j2].mean(0), k=k2, j=j2))
        print(f'  {t0-ti[0]:7.1f}-{t1-ti[0]:7.1f} s  nclouds {len(idx)}  res piso {res*100:.2f} cm  n {n.round(3)}  a/|a| {pl[-1]["a"].round(3)}')
    if len(pl) < 3:
        print('FALLA: <3 mesetas con piso'); sys.exit(1)
    bad = [p for p in pl if p['res'] > 0.015 or p['dn'] > 0.5 or p['da'] > 0.5]
    pl = [p for p in pl if p['res'] <= 0.015 and p['dn'] <= 0.5 and p['da'] <= 0.5]
    print('estables (res<1.5 cm, dn<0.5, da<0.5 deg):', [(round(p['t0']-ti[0]), round(p['dn'],2), round(p['da'],2)) for p in pl])
    if bad:
        print(f'descartadas {len(bad)} mesetas con residuo > 1.5 cm')
    N = np.array([p['n'] for p in pl]); Aa = np.array([p['a'] for p in pl])
    print(f'angulo max entre normales: {max(np.degrees(np.arccos(np.clip(N @ n, -1, 1))).max() for n in N):.1f} deg')
    # 3. accel -> lidar
    Ra, ra = kabsch(Aa, N)
    ga = np.degrees(np.arccos(np.clip(np.sum((Aa @ Ra.T) * N, 1), -1, 1)))
    print(f'\nacc->lidar: residuo {ra:.2f} deg (max {ga.max():.2f})')
    # 4. gyro -> lidar: entre mesetas consecutivas
    wm = np.array([p['wm'] for p in pl])
    U, V, rows = [], [], []
    print('maniobras:   t   ang_gyro  ang_normales  eje_gyro')
    for p, r in zip(pl[:-1], pl[1:]):
        s, e = p['j'], r['k']
        if ti[e] - ti[s] > 30 or e - s < 5:
            continue
        b = 0.5 * (p['wm'] + r['wm'])
        Rm = np.eye(3)
        for i in range(s, e):
            Rm = Rm @ expm((w[i] - b) * (ti[i + 1] - ti[i]))
        rv = logv(Rm)
        ag, an = np.degrees(np.linalg.norm(rv)), np.degrees(np.arccos(np.clip(p['n'] @ r['n'], -1, 1)))
        cr = np.cross(r['n'], p['n'])  # eje en el lidar (derivado: n_j = n_i x omega + n_i)
        rows.append((ti[s] - ti[0], ag, an, rv / max(np.linalg.norm(rv), 1e-9), cr, an))
        print(f'  {ti[s]-ti[0]:7.1f} {ag:8.2f} {an:10.2f}  {np.round(rv/max(np.linalg.norm(rv),1e-9),3)}')
    # pares de ejes: cabeceo/alabeo (an>2 deg) y guinada (an<1 y ag>20 deg -> eje = gravedad)
    pitch = [(r[3], r[4] / np.linalg.norm(r[4])) for r in rows if r[5] > 2.0]
    yaw = [r for r in rows if r[5] < 2.5 and r[1] > 20.0]
    print(f'pares de eje por inclinacion {len(pitch)}, por guinada {len(yaw)}')
    if len(pitch) < 2:
        print('FALLA: <2 maniobras de inclinacion'); sys.exit(1)
    Pg = np.array([x[0] for x in pitch]); Ql = np.array([x[1] for x in pitch])
    best = None
    for sgn in ([1, -1] if yaw else [0]):
        P2, Q2 = list(Pg), list(Ql)
        for r in yaw:
            P2.append(r[3]); Q2.append(sgn * pl[0]['n'])
        Rg, rg = kabsch(np.array(P2), np.array(Q2))
        print(f'  signo guinada {sgn:+d}: residuo {rg:.2f} deg')
        if best is None or rg < best[1]:
            best = (Rg, rg)
    Rg, rg = best
    # 5. gate
    gates = []
    gates.append(('acc residuo <=2', ra <= 2.0))
    gates.append(('gyro residuo <=2', rg <= 2.0))
    ratio = [r[1] / r[2] for r in rows if r[5] > 2.0]
    gates.append(('angulo gyro/normales ±5%', all(abs(x - 1) <= 0.05 for x in ratio)))
    print(f'razon angulo gyro/normales: {np.round(ratio, 3).tolist()}')
    # 6. matrices
    for nm, R, cur in (('acc_R', CLOUD_R @ Ra, B_CUR), ('imu_R', CLOUD_R @ Rg, A_CUR)):
        print(f'\n{nm} = {R.ravel().round(4).tolist()}  det {np.linalg.det(R):+.3f}')
        print(f'  angulo vs actual ({"B" if nm=="acc_R" else "A"}): {ang(R @ cur.T):.1f} deg;  vs Rz(pi)*actual: {ang(R @ (np.diag([-1,-1,1]) @ cur).T):.1f} deg')
    if len(sys.argv) > 2:
        import analizar_imu
        analizar_imu.CAND = {'NUEVA': CLOUD_R @ Rg}
        sys.argv = ['x', sys.argv[2]]
        print('\n--- corr_z en bag de giros (esperado ~0.99) ---')
        try:
            analizar_imu.main()
        except SystemExit as e:
            print('analizar_imu salio', e.code)
    print('\nGATE:', {n: ok for n, ok in gates})
    sys.exit(0 if all(ok for _, ok in gates) else 1)


if __name__ == '__main__':
    main()
