#!/usr/bin/env python3
"""Paso 6 de la fase 4: movimiento mínimo sin Nav2 (requiere puente armado y aprobación del usuario).

Uso: source scripts/env_external.sh && python3 scripts/prueba_movimiento.py <prueba>
  giro           wz=0.2 durante 2 s
  avance         vx=0.1 durante 2 s
  giro2          wz=0.4 durante 2 s
  avance2        vx=0.2 durante 2 s
  corte_cmd      vx=0.2; a los 1.5 s deja de mandar /cmd_vel (deadman sigue)
  corte_deadman  vx=0.2; a los 1.5 s deja de mandar deadman (cmd sigue)
  paro_l2y       vx=0.2 durante 4 s; el usuario presiona L2+Y a la mitad
  larga          vx=0.2 durante 8 s (joystick o L2+Y del usuario)
Imprime cada 0.1 s: t, estado del puente y twist de /utlidar/robot_odom.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

PRUEBAS = {  # (vx, wz, duración, t_corte_cmd, t_corte_deadman)
    'giro': (0.0, 0.2, 2.0, None, None),
    'avance': (0.1, 0.0, 2.0, None, None),
    # 0.1 m/s y 0.2 rad/s solo inclinan el cuerpo sin dar pasos (zona muerta de la marcha del Go2)
    'giro2': (0.0, 0.4, 2.0, None, None),
    'avance2': (0.2, 0.0, 2.0, None, None),
    'corte_cmd': (0.2, 0.0, 3.0, 1.5, None),
    'corte_deadman': (0.2, 0.0, 3.0, None, 1.5),
    'paro_l2y': (0.2, 0.0, 4.0, None, None),
    'larga': (0.2, 0.0, 8.0, None, None),  # joystick o L2+Y a mitad de camino
    'larga03': (0.3, 0.0, 8.0, None, None),  # umbral de marcha
    'larga04': (0.4, 0.0, 8.0, None, None),
}
GOAL_KEY = '9.000,9.000,0.000,1.000'  # meta ficticia: el puente exige una aprobación que coincida


class Prueba(Node):
    def __init__(self, vx, wz, dur, t_cmd, t_dead):
        super().__init__('prueba_movimiento')
        self.p = (vx, wz, dur, t_cmd, t_dead)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dead = self.create_publisher(Empty, '/laika/deadman', 10)
        self.goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.appr = self.create_publisher(String, '/laika/plan_approved', 10)
        self.state, self.tw = '?', (0.0, 0.0, 0.0)
        self.create_subscription(String, '/laika/bridge_state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, 10)
        self.t0 = None

    def odom_cb(self, m):
        self.tw = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z)

    def step(self, t):
        vx, wz, dur, t_cmd, t_dead = self.p
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x = g.pose.position.y = 9.0
        g.pose.orientation.w = 1.0
        self.goal.publish(g)  # en cada ciclo: los primeros mensajes se pierden mientras DDS descubre
        self.appr.publish(String(data=GOAL_KEY))
        if t_dead is None or t < t_dead:
            self.dead.publish(Empty())
        if 0.5 <= t < 0.5 + dur and (t_cmd is None or t < 0.5 + t_cmd):
            m = Twist()
            m.linear.x, m.angular.z = vx, wz
            self.cmd.publish(m)
        print('%.2f t=%5.2f  puente=%-7s  odom vx=%+.3f vy=%+.3f wz=%+.3f' % (time.time(), t, self.state, *self.tw), flush=True)


def main():
    nombre = sys.argv[1] if len(sys.argv) > 1 else ''
    assert nombre in PRUEBAS, 'prueba: ' + ', '.join(PRUEBAS)
    rclpy.init()
    n = Prueba(*PRUEBAS[nombre])
    t0 = time.time()
    total = 0.5 + PRUEBAS[nombre][2] + 2.0  # 2 s finales observando la parada
    try:
        while time.time() - t0 < total:
            rclpy.spin_once(n, timeout_sec=0.0)
            n.step(time.time() - t0)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
