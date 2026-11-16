import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
from unitree_api.msg import Request
from unitree_go.msg import LowState

from laika_slam import sport_logic as L

API_MOVE, API_STOP = 1008, 1003  # unicos IDs que este nodo usa


class CmdVelToSport(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_sport')
        self.declare_parameter('dry_run_force_active', False)  # solo vale si la salida no es /api/sport/request
        self.out_topic = self.resolve_topic_name('sport_request_out')
        self.force = (self.get_parameter('dry_run_force_active').value
                      and self.out_topic != '/api/sport/request')
        self.pub = self.create_publisher(Request, 'sport_request_out', 10)
        self.state_pub = self.create_publisher(String, '/laika/bridge_state', 10)
        self.create_subscription(LowState, '/lowstate', self.lowstate_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Empty, '/laika/deadman', self.dead_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(String, '/laika/plan_approved', self.appr_cb, 10)
        self.active = self.force
        self.prev_keys = 0
        self.cmd = self.t_cmd = self.t_dead = None
        self.goal = self.approved_key = None
        self.moving = False
        self.stop_until = 0.0
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Salida: %s (armed=%s)%s' % (
            self.out_topic, self.out_topic == '/api/sport/request',
            ' [dry_run_force_active]' if self.force else ''))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send(self, api_id, param=''):
        m = Request()
        m.header.identity.id = self.get_clock().now().nanoseconds
        m.header.identity.api_id = api_id
        m.parameter = param
        self.pub.publish(m)

    def start_stop(self):
        self.stop_until = self.now() + L.STOP_BURST

    def lowstate_cb(self, msg):
        keys = L.remote_keys(msg.wireless_remote)
        ev = L.remote_event(keys, self.prev_keys)
        self.prev_keys = keys
        if ev == 'stop':
            self.active = False
            self.moving = False
            self.start_stop()
            self.get_logger().warn('L2+Y: StopMove 1 s y vuelve a IDLE')
        elif ev == 'activate' and not self.active:
            self.active = True
            self.get_logger().info('R2+X: ACTIVE')

    def cmd_cb(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.t_cmd = self.now()

    def dead_cb(self, _):
        self.t_dead = self.now()

    def goal_cb(self, m):
        p = m.pose
        self.goal = L.goal_key(p.position.x, p.position.y, p.orientation.z, p.orientation.w)

    def appr_cb(self, m):
        self.approved_key = m.data

    def tick(self):
        now = self.now()
        if now < self.stop_until:
            self.send(API_STOP)
        elif self.active:
            c = L.move_command(now, self.cmd, self.t_cmd, self.t_dead,
                               self.goal is not None and self.approved_key == self.goal)
            if c:
                self.moving = True
                self.send(API_MOVE, '{"x": %.3f, "y": %.3f, "z": %.3f}' % c)
            elif self.moving:  # paso de mover a no poder: 1 s de StopMove y silencio
                self.moving = False
                self.start_stop()
        state = 'STOP' if now < self.stop_until else (
            ('MOVING' if self.moving else 'ACTIVE') if self.active else 'IDLE')
        self.state_pub.publish(String(data=state))

    def destroy_node(self):
        if self.active or self.moving:  # StopMove al cerrar; no mandar nada si siempre estuvo IDLE
            for _ in range(3):
                self.send(API_STOP)
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
