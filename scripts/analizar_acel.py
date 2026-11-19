#!/usr/bin/env python3
"""B3 (P5.2-A): rotacion del acelerometro del L1. Uso: analizar_acel.py <ruta_bag>
Solo lee el .db3. Candidatas: B y Rz(pi)*B. Sale con 1 si ninguna tiene corr > 0.5 en ambas."""
import glob, os, sqlite3, sys
import numpy as np
from analizar_imu import leer, st, CAND

CANDS = {'B': CAND['B'], 'RzB': np.diag([-1.0, -1.0, 1.0]) @ CAND['B']}


def lowpass(x, fs, fc=2.0):  # media movil ~ pasabajas fc (sin dependencias extra)
    k = max(1, int(fs / fc))
    return np.apply_along_axis(lambda c: np.convolve(c, np.ones(k) / k, mode='same'), 0, x)


def main():
    db = sqlite3.connect(glob.glob(os.path.join(sys.argv[1], '*.db3'))[0])
    imu = list(leer(db, '/utlidar/imu'))
    ti = np.array([st(m.header) for m in imu])
    w = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu])
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in imu])
    od = list(leer(db, '/utlidar/robot_odom'))
    to = np.array([st(m.header) for m in od])
    v = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in od])
    wo = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in od])
    quieto = (np.linalg.norm(v, axis=1) < 0.02) & (np.linalg.norm(wo, axis=1) < 0.02)
    mq = np.interp(ti, to, quieto.astype(float)) > 0.99
    mw = ~(np.interp(ti, to, (np.linalg.norm(v, axis=1) < 0.05).astype(float)) > 0.0)  # caminando: |v|>=0.05 siempre
    fs = 1 / np.median(np.diff(ti))
    print(f'IMU {len(ti)} msgs, fs {fs:.0f} Hz; muestras quietas {mq.sum()}, caminando {mw.sum()}; '
          f'max|v| odom {np.linalg.norm(v, axis=1).max():.2f} m/s')
    ax_ref = np.interp(ti, to, np.gradient(v[:, 0], to))
    ax_ref = lowpass(ax_ref[:, None], fs)[:, 0]
    wA = w @ CAND['A'].T
    res = {}
    for n, R in CANDS.items():
        ab = a @ R.T
        am = ab[mq].mean(axis=0)
        ang = np.degrees(np.arccos(am[2] / np.linalg.norm(am)))
        axl = lowpass(ab[:, :1], fs)[:, 0]
        c1 = np.corrcoef(axl[mw], ax_ref[mw])[0, 1]
        pitch = np.arctan2(-ab[:, 0], ab[:, 2])
        roll = np.arctan2(ab[:, 1], ab[:, 2])
        dp = lowpass(np.gradient(pitch, ti)[:, None], fs)[:, 0]
        dr = lowpass(np.gradient(roll, ti)[:, None], fs)[:, 0]
        gy, gx = lowpass(wA[:, 1:2], fs)[:, 0], lowpass(wA[:, :1], fs)[:, 0]
        c2, c3 = np.corrcoef(dp[mw], gy[mw])[0, 1], np.corrcoef(dr[mw], gx[mw])[0, 1]
        res[n] = (c1, c2)
        print(f'{n:4s} quieta {am.round(3)} norma {np.linalg.norm(am):.3f} angulo z {ang:.2f} deg | '
              f'corr ax-vs-dvx/dt {c1:.3f} | corr pitch_rate-vs-gyro_y {c2:.3f} (roll-vs-gyro_x {c3:.3f})')
    ok = [n for n, (c1, c2) in res.items() if c1 > 0.5 and c2 > 0.5]
    if not ok:
        print('INCONCLUSO: ninguna candidata con corr > 0.5 en ambas; no se elige matriz (hace falta prueba de inclinacion)')
        sys.exit(1)
    best = max(ok, key=lambda n: min(res[n]))
    print(f'Elegida: {best}; acc_R =', CANDS[best].ravel().round(4).tolist())


if __name__ == '__main__':
    main()
