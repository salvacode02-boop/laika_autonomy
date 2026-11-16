#!/usr/bin/env python3
"""Muestra a 2 Hz la pose del robot en el mapa (TF map->base_link), la odometría del robot y el estado del puente.
Uso: source scripts/env_external.sh && python3 scripts/ver_pose.py
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class VerPose(Node):
    def __init__(self):
        super().__init__('ver_pose')
        self.amcl = self.odom = None
        self.tf = Buffer()
        TransformListener(self.tf, self)  # AMCL solo publica /amcl_pose al moverse; el TF siempre está
        self.state = '-'
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: setattr(self, 'amcl', m), 10)
        self.create_subscription(Odometry, '/laika/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(String, '/laika/bridge_state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_timer(0.5, self.show)

    def show(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            a = 'mapa x=%+6.2f y=%+6.2f yaw=%+6.1f°' % (t.translation.x, t.translation.y, yaw(t.rotation))
            if self.amcl:
                c = self.amcl.pose.covariance
                a += ' (σxy %.2f m)' % math.sqrt(max(c[0], c[7]))
        except Exception:
            a = 'mapa: (esperando pose inicial)'
        o = 'odom: -'
        if self.odom:
            t = self.odom.twist.twist
            o = 'odom vx=%+.2f vy=%+.2f wz=%+.2f' % (t.linear.x, t.linear.y, t.angular.z)
        print('\r%s | %s | puente=%-7s ' % (a, o, self.state), end='', flush=True)


def main():
    rclpy.init()
    n = VerPose()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    print()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
