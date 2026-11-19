#!/usr/bin/env python3
"""Evalúa FAST-LIO contra la odometría de patas (C2, P5.3). Nodo Foxy/Humble; corre en el robot durante un replay.

  ros2 launch laika_slam fast_lio.launch.py bag:=true      # en una terminal
  ~/reproducir_bag.sh <bag>                                # en otra
  python3 evaluar_lio.py [--guardar /tmp/x.npz]            # Ctrl-C al terminar el bag -> imprime el informe
  python3 evaluar_lio.py --npz /tmp/x.npz                  # re-analiza un registro guardado (sin ROS)

Suscribe: /lio/odometry, /bag/utlidar/robot_odom, /bag/utlidar/imu (giroscopio del L1), /laika/body_imu.
Informe: error de regreso 3D + yaw (LIO y patas), dirección de avance contra +x (tramos rectos >= 1 m), divergencia
(|v| > 2 m/s o salto en z > 0.3 m), Hz, CPU de fastlio_mapping y latencia del IMU del cuerpo contra el del L1
(solo si hay IMU de ambos: bag con movimiento rápido, p. ej. b2_estres).
Compuertas (plan lote 2, C2): regreso <= 8 cm / 1.5 deg; b2: sin divergencia, error LIO <= error de patas, latencia <= 10 ms.
"""
import argparse
import os
import signal
import sys
import time

import numpy as np


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def prep(a):
    """a: filas [t_llegada, t_stamp, x, y, z, qx, qy, qz, qw] -> (llegada, stamp, pos, yaw desenrollado)."""
    q = a[:, 5:9]
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    return a[:, 0], a[:, 1], a[:, 2:5], np.unwrap(yaw)


def regreso(res, name, ts, te):
    """Error de regreso en el marco de inicio. Ventana inicial: hasta 0.5 s antes de moverse (LIO: desde 3 s, init);
    ventana final: 1 s después de dejar de moverse."""
    arr, t, p, yaw = res[name]

    def win(a, b):
        i = (arr >= a) & (arr <= b)
        if not i.any():
            return None
        return np.median(p[i], axis=0), np.arctan2(np.sin(yaw[i]).mean(), np.cos(yaw[i]).mean())
    a0 = max(arr[0], 3.0) if name == 'lio' else arr[0]
    w0, w1 = win(a0, ts - 0.5), win(te + 1.0, arr[-1])
    if w0 is None or w1 is None:
        return None
    (p0, y0), (p1, y1) = w0, w1
    R = np.array([[np.cos(y0), np.sin(y0)], [-np.sin(y0), np.cos(y0)]])
    d = np.append(R @ (p1[:2] - p0[:2]), p1[2] - p0[2])
    return d, np.degrees(wrap(y1 - y0))


def rectos(res, name):
    """Ángulo (deg) entre la dirección de avance y el rumbo, por tramo recto (ventanas de 0.5 s, giro < 0.1 rad/s) >= 1 m."""
    arr, t, p, yaw = res[name]
    out = []
    k = np.minimum(np.searchsorted(t, t + 0.5), len(t) - 1)
    dt = np.maximum(t[k] - t, 1e-3)
    ok = (np.abs(wrap(yaw[k] - yaw)) / dt < 0.1) & (np.hypot(*(p[k, :2] - p[:, :2]).T) / dt > 0.1)
    i = 0
    while i < len(t):
        if not ok[i]:
            i += 1
            continue
        j = i
        while j < len(t) and ok[j]:
            j += 1
        disp = p[j - 1, :2] - p[i, :2]
        if np.hypot(*disp) >= 1.0:
            h = np.arctan2(np.sin(yaw[i:j]).mean(), np.cos(yaw[i:j]).mean())
            out.append(np.degrees(wrap(np.arctan2(disp[1], disp[0]) - h)))
        i = j
    return np.array(out)


def divergencia(res, name):
    arr, t, p, yaw = res[name]
    dt = np.maximum(np.diff(t), 1e-3)
    v = np.hypot(*np.diff(p[:, :2], axis=0).T) / dt
    dz = np.abs(np.diff(p[:, 2]))
    return int((v > 2.0).sum()), int((dz > 0.3).sum()), float(v.max()), float(dz.max())


def latencia_imu(l1, body, fs=200.0, lag_ms=50, w_min=0.8):
    """Retardo (ms) del giroscopio del cuerpo respecto al del L1: máximo de la correlación de |w| (invariante al marco)
    solo en los tramos rápidos (|w| del cuerpo > w_min rad/s, dilatado 0.25 s). + = el cuerpo llega después.
    Usa tiempo de llegada: en replay ambos salen del mismo reloj de bag, pero el relevo suma un poco de retardo propio."""
    t0, t1 = max(l1[0, 0], body[0, 0]), min(l1[-1, 0], body[-1, 0])
    g = np.arange(t0, t1, 1 / fs)
    a = np.interp(g, l1[:, 0], np.linalg.norm(l1[:, 1:4], axis=1))
    b = np.interp(g, body[:, 0], np.linalg.norm(body[:, 1:4], axis=1))
    fast = np.convolve(b > w_min, np.ones(int(0.25 * fs) * 2 + 1), 'same') > 0
    if fast.sum() < 2 * fs:
        return None, float(fast.sum() / fs)
    best = None
    for k in range(-int(lag_ms * fs / 1000), int(lag_ms * fs / 1000) + 1):
        bb = np.roll(b, -k)  # k>0: compara b(t+k) con a(t)
        m = fast & np.roll(fast, -k)
        c = np.corrcoef(a[m], bb[m])[0, 1]
        if best is None or c > best[0]:
            best = (c, k)
    return best[1] * 1000 / fs, float(fast.sum() / fs)


def informe(d, cpu=None):
    res = {n: prep(d[n]) for n in ('lio', 'leg') if n in d and len(d[n]) > 10}
    for n, (arr, t, p, yaw) in res.items():
        print('%s: n %d, %.2f Hz, hueco máx %.2f s' % (n, len(t), len(t) / max(arr[-1] - arr[0], 1e-3), np.diff(arr).max()))
    if cpu is not None:
        print('CPU fastlio_mapping: %.0f %% de un núcleo' % cpu)
    if 'leg' in res:
        arr, t, p, yaw = res['leg']
        v = np.hypot(*np.gradient(p[:, :2], t, axis=0).T)
        mov = np.where(np.convolve(v > 0.05, np.ones(5) / 5, 'same') > 0.5)[0]
        if len(mov):
            ts, te = arr[mov[0]], arr[mov[-1]]
            print('patas en movimiento (llegada): %.1f .. %.1f de %.1f s' % (ts, te, arr[-1]))
            for n in res:
                r = regreso(res, n, ts, te)
                if r:
                    e3 = np.linalg.norm(r[0])
                    print('%s regreso (marco inicial): x %.3f y %.3f z %.3f m | 3D %.1f cm | yaw %.2f deg'
                          % (n, r[0][0], r[0][1], r[0][2], 100 * e3, r[1]))
    for n in res:
        s = rectos(res, n)
        print('%s tramos rectos: %d%s' % (n, len(s), '' if not len(s) else ' | ángulos %s | mediana |a| %.1f deg, máx |a| %.1f deg'
              % (np.round(s, 1).tolist(), np.median(np.abs(s)), np.abs(s).max())))
        nv, nz, vm, zm = divergencia(res, n)
        print('%s divergencia: %d saltos |v|>2 m/s (máx %.2f), %d saltos z>0.3 m (máx %.2f) -> %s'
              % (n, nv, vm, nz, zm, 'DIVERGE' if nv or nz else 'ok'))
    if 'imu_l1' in d and 'imu_body' in d and len(d['imu_l1']) > 100 and len(d['imu_body']) > 100:
        lag, seg = latencia_imu(d['imu_l1'], d['imu_body'])
        print('latencia IMU cuerpo vs L1: %s (%.1f s de tramos rápidos)'
              % ('sin tramos rápidos suficientes' if lag is None else '%+.1f ms (compuerta <= 10 ms)' % lag, seg))


def _cpu_fastlio():
    for p in os.listdir('/proc'):
        try:
            if p.isdigit() and open('/proc/%s/comm' % p).read().strip() == 'fastlio_mappi':
                f = open('/proc/%s/stat' % p).read().rsplit(')', 1)[1].split()
                return int(p), (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')
        except OSError:
            pass
    return None, None


def grabar(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    signal.signal(signal.SIGINT, signal.default_int_handler)  # un proceso lanzado con & desde un script hereda SIGINT ignorado
    rclpy.init()
    n = Node('evaluar_lio')
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
    d = {'lio': [], 'leg': [], 'imu_l1': [], 'imu_body': []}
    t0 = []

    def now():  # s desde el primer mensaje recibido (marca común a todos los tópicos)
        t = n.get_clock().now().nanoseconds * 1e-9
        if not t0:
            t0.append(t)
        return t - t0[0]

    def odom(k):
        def cb(m):
            s, o = m.header.stamp, m.pose.pose
            d[k].append([now(), s.sec + s.nanosec * 1e-9, o.position.x, o.position.y, o.position.z,
                         o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w])
        return cb

    def imu(k):
        def cb(m):
            w = m.angular_velocity
            d[k].append([now(), w.x, w.y, w.z])
        return cb
    n.create_subscription(Odometry, args.lio, odom('lio'), qos)
    n.create_subscription(Odometry, args.leg, odom('leg'), qos)
    n.create_subscription(Imu, args.imu_l1, imu('imu_l1'), qos)
    n.create_subscription(Imu, args.imu_body, imu('imu_body'), qos)
    pid, c0 = _cpu_fastlio()
    w0 = time.time()
    cpu = None
    print('Grabando (Ctrl-C para el informe)...', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    if pid:
        p2, c1 = _cpu_fastlio()
        if p2 == pid:
            cpu = 100 * (c1 - c0) / (time.time() - w0)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    d = {k: np.array(v, dtype=np.float64) for k, v in d.items()}
    return d, cpu


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lio', default='/lio/odometry')
    ap.add_argument('--leg', default='/bag/utlidar/robot_odom')
    ap.add_argument('--imu-l1', default='/bag/utlidar/imu')
    ap.add_argument('--imu-body', default='/laika/body_imu')
    ap.add_argument('--guardar', help='guarda el registro en .npz')
    ap.add_argument('--npz', help='no usa ROS: analiza un registro guardado')
    args, _ = ap.parse_known_args()
    if args.npz:
        d, cpu = dict(np.load(args.npz)), None
        cpu = float(d.pop('cpu')) if 'cpu' in d else None
    else:
        d, cpu = grabar(args)
        if args.guardar:
            np.savez(args.guardar, cpu=np.nan if cpu is None else cpu, **d)
    informe(d, None if cpu is None or np.isnan(cpu) else cpu)


if __name__ == '__main__':
    sys.exit(main())
