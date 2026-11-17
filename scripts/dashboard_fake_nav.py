#!/usr/bin/env python3
"""Simulador local para probar el dashboard sin robot (usar SOLO en un dominio aislado:
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1, sin env_external.sh).
Publica /map, TF, odometria, costmaps, /cmd_vel, /laika/bridge_state y, al recibir /goal_pose,
un /plan recto (1 Hz) hacia la meta. No publica nada hacia el robot real."""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def grid(w, h, res, ox, oy, frame, fn):
    g = OccupancyGrid()
    g.header.frame_id = frame
    g.info.width, g.info.height, g.info.resolution = w, h, res
    g.info.origin.position.x, g.info.origin.position.y = ox, oy
    g.data = [fn(i % w, i // w) for i in range(w * h)]
    return g


class Fake(Node):
    def __init__(self, state):
        super().__init__('fake_nav')
        self.state = state
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.gcm = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 1)
        self.lcm = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', 1)
        self.odom = self.create_publisher(Odometry, '/utlidar/robot_odom', 10)
        self.plan = self.create_publisher(Path, '/plan', 10)
        self.lplan = self.create_publisher(Path, '/local_plan', 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.st = self.create_publisher(String, '/laika/bridge_state', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.goal = None
        wall = lambda x, y: 100 if (x in (0, 199) or y in (0, 159)) else 0
        self.map_msg = grid(200, 160, 0.05, -2.0, -2.0, 'map', wall)
        # costo: letal en el borde, inflacion de 6 celdas hacia dentro
        infl = lambda x, y: 100 if min(x, y, 199 - x, 159 - y) == 0 else max(0, 99 - 15 * min(x, y, 199 - x, 159 - y)) if min(x, y, 199 - x, 159 - y) < 7 else 0
        self.gcm_msg = grid(200, 160, 0.05, -2.0, -2.0, 'map', infl)
        self.lcm_msg = grid(60, 60, 0.05, -1.5, -1.5, 'odom', lambda x, y: 100 if (28 <= x <= 32 and 40 <= y <= 44) else (60 if (25 <= x <= 35 and 37 <= y <= 47) else 0))
        self.create_timer(0.1, self.fast)
        self.create_timer(1.0, self.slow)

    def goal_cb(self, m):
        self.goal = m

    def fast(self):
        now = self.get_clock().now().to_msg()
        for p, c, ch, x, y, yaw in (('map', 'odom', 0, 0.2, -0.1, 0.1), ('odom', 'base_link', 0, 0.3, 0.2, 0.4)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, p, c
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.twist.twist.linear.x, o.twist.twist.angular.z = 0.28, 0.1
        self.odom.publish(o)
        c = Twist()
        c.linear.x, c.angular.z = 0.30, 0.25
        self.cmd.publish(c)
        self.st.publish(String(data=self.state))

    def slow(self):
        self.map_pub.publish(self.map_msg)
        self.gcm.publish(self.gcm_msg)
        self.lcm.publish(self.lcm_msg)
        # robot en map: odom(0.3,0.2,0.4) compuesto con map<-odom(0.2,-0.1,0.1)
        c, s = math.cos(0.1), math.sin(0.1)
        rx, ry = 0.2 + c * 0.3 - s * 0.2, -0.1 + s * 0.3 + c * 0.2
        lp = Path()
        lp.header.frame_id = 'odom'
        for i in range(15):
            q = PoseStamped()
            q.pose.position.x, q.pose.position.y = 0.3 + 0.04 * i * math.cos(0.4 + 0.02 * i), 0.2 + 0.04 * i * math.sin(0.4 + 0.02 * i)
            lp.poses.append(q)
        self.lplan.publish(lp)
        if self.goal:
            gx, gy = self.goal.pose.position.x, self.goal.pose.position.y
            p = Path()
            p.header.frame_id = 'map'
            for i in range(41):
                q = PoseStamped()
                q.pose.position.x, q.pose.position.y = rx + (gx - rx) * i / 40, ry + (gy - ry) * i / 40
                p.poses.append(q)
            self.plan.publish(p)


def main():
    rclpy.init()
    rclpy.spin(Fake(sys.argv[1] if len(sys.argv) > 1 else 'ACTIVE'))


if __name__ == '__main__':
    main()
