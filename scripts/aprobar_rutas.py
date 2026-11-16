#!/usr/bin/env python3
"""Aprobacion de rutas por Enter (temporal, fase 4). Uso: source scripts/env_external.sh && python3 scripts/aprobar_rutas.py

Hombre muerto: publica /laika/deadman a 5 Hz mientras corre (Ctrl-C = parada).
Meta nueva -> espera su primer /plan -> Enter publica /laika/plan_approved con la clave de la meta
(misma formula que laika_slam.sport_logic.goal_key). Los re-planeos de la misma meta no piden Enter.
"""
import math
import select
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker


def goal_key(x, y, qz, qw):
    return '%.3f,%.3f,%.3f,%.3f' % (x, y, qz, qw)


class Aprobar(Node):
    def __init__(self):
        super().__init__('aprobar_rutas')
        self.dead = self.create_publisher(Empty, '/laika/deadman', 10)
        self.appr = self.create_publisher(String, '/laika/plan_approved', 10)
        self.marker = self.create_publisher(Marker, '/laika/cmd_vel_marker', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/laika/bridge_state', lambda m: setattr(self, 'state', m.data), 10)
        self.last_plan = None
        self.key = None        # meta actual
        self.pending = False   # meta nueva sin plan mostrado
        self.waiting = False   # plan mostrado, esperando Enter
        self.approved = ''
        self.state, self.vel = '?', (0.0, 0.0)
        self.create_timer(0.2, self.tick)

    def goal_cb(self, m):
        p = m.pose
        self.key = goal_key(p.position.x, p.position.y, p.orientation.z, p.orientation.w)
        self.yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.gxy = (p.position.x, p.position.y)
        self.approved, self.pending, self.waiting = '', True, False
        self.appr.publish(String(data=''))  # invalida la aprobacion anterior
        if self.last_plan:
            self.plan_cb(self.last_plan)  # el plan pudo llegar antes que la meta

    def plan_cb(self, m):
        self.last_plan = m
        if not self.pending or len(m.poses) < 2:
            return
        e = m.poses[-1].pose.position  # ignora planes de la meta anterior
        if math.dist((e.x, e.y), self.gxy) > 0.5:
            return
        n = len(m.poses)
        L = sum(math.dist((a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y))
                for a, b in zip(m.poses, m.poses[1:]))
        print('\nRuta nueva planeada: %d puntos, longitud %.2f m, meta (%.2f, %.2f, %.0f deg)'
              % (n, L, self.gxy[0], self.gxy[1], math.degrees(self.yaw)))
        print('Enter para ejecutar (ver /plan en RViz)...')
        self.pending, self.waiting = False, True

    def cmd_cb(self, m):
        self.vel = (m.linear.x, m.angular.z)
        k = Marker()
        k.header.frame_id, k.ns, k.type = 'base_link', 'cmd_vel', Marker.ARROW
        k.scale.x, k.scale.y, k.scale.z = max(abs(m.linear.x) * 4.0, 0.02), 0.05, 0.05
        k.color.r, k.color.g, k.color.a = 1.0, 1.0, 1.0
        k.pose.orientation.z = math.sin(m.angular.z / 2)  # yaw = wz rad (visual)
        k.pose.orientation.w = math.cos(m.angular.z / 2)
        self.marker.publish(k)

    def tick(self):
        self.dead.publish(Empty())
        if self.waiting and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            self.approved, self.waiting = self.key, False
            print('Aprobada.')
        self.appr.publish(String(data=self.approved))
        print('\rvx=%.2f wz=%.2f  puente=%-7s ' % (*self.vel, self.state), end='', flush=True)


def main():
    rclpy.init()
    n = Aprobar()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    print('\nDeadman detenido.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
