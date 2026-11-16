import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, Int32, String
from unitree_api.msg import Request
from unitree_go.msg import LowState

from laika_slam import sport_logic as L

API_MOVE, API_STOP = 1008, 1003  # unicos IDs que este nodo usa


class CmdVelToSport(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_sport')
        self.declare_parameter('dry_run_force_active', False)  # solo vale si la salida no es /api/sport/request
        self.declare_parameter('sin_cable', False)  # modo sin cable: R2+X aprobado reemplaza al deadman
        self.declare_parameter('dry_run_keys_topic', '')  # prueba: Int32 con teclas simuladas (solo salida != /api/sport/request)
        self.pub = self.create_publisher(Request, 'sport_request_out', 10)
        self.out_topic = self.pub.topic_name  # ya con el remapping aplicado
        self.force = (self.get_parameter('dry_run_force_active').value
                      and self.out_topic != '/api/sport/request')
        self.sc = L.SinCable() if self.get_parameter('sin_cable').value else None
        self.state_pub = self.create_publisher(String, '/laika/bridge_state', 10)
        # /lf/lowstate (20 Hz) en vez de /lowstate (500 Hz): mismo control, ~70 % menos CPU
        self.create_subscription(LowState, '/lf/lowstate', self.lowstate_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Empty, '/laika/deadman', self.dead_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(String, '/laika/plan_approved', self.appr_cb, 10)
        self.active = self.force and self.sc is None
        if self.sc:
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.goal_msg = None
            # status de acciones: transient_local, para ver los goals que ya existian al llegar
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_cb, qos)
            keys_topic = self.get_parameter('dry_run_keys_topic').value
            if keys_topic and self.out_topic != '/api/sport/request':
                self.create_subscription(Int32, keys_topic, lambda m: self.on_event(L.remote_event(m.data, 0)), 10)
        self.prev_keys = 0
        self.cmd = self.t_cmd = self.t_dead = None
        self.t_low = None
        self.goal = self.approved_key = None
        self.moving = False
        self.stop_until = 0.0
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Salida: %s (armed=%s)%s sin_cable=%s' % (
            self.out_topic, self.out_topic == '/api/sport/request',
            ' [dry_run_force_active]' if self.force else '', self.sc is not None))

    def now(self):
        return time.monotonic()  # plazos inmunes a saltos del reloj

    def send(self, api_id, param=''):
        m = Request()
        m.header.identity.id = self.get_clock().now().nanoseconds
        m.header.identity.api_id = api_id
        m.parameter = param
        self.pub.publish(m)

    def start_stop(self):
        self.stop_until = self.now() + L.STOP_BURST

    def lowstate_cb(self, msg):
        self.t_low = self.now()
        keys = L.remote_keys(msg.wireless_remote)
        ev = L.remote_event(keys, self.prev_keys)
        self.prev_keys = keys
        self.on_event(ev)

    def on_event(self, ev):
        if self.sc:
            self.sc_result(self.sc.on_event(ev, self.now()), 'R2+X' if ev == 'activate' else 'L2+Y')
        elif ev == 'stop':
            self.active = False
            self.moving = False
            self.start_stop()
            self.get_logger().warn('L2+Y: StopMove 1 s y vuelve a IDLE')
        elif ev == 'activate' and not self.active:
            self.active = True
            self.get_logger().info('R2+X: ACTIVE')

    def sc_result(self, r, why):
        if r == 'run':
            self.get_logger().info('%s: RUN, republico la meta' % why)
            if self.goal_msg:
                self.goal_msg.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub.publish(self.goal_msg)
        elif r == 'stop':
            self.moving = False
            self.start_stop()
            self.get_logger().warn('%s: StopMove 1 s, IDLE, aprobación consumida' % why)

    def status_cb(self, m):
        ids = {bytes(s.goal_info.goal_id.uuid): s.status for s in m.status_list}
        self.sc_result(self.sc.on_status(ids), 'Nav2 terminó la meta')

    def cmd_cb(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.t_cmd = self.now()

    def dead_cb(self, _):
        self.t_dead = self.now()

    def goal_cb(self, m):
        p = m.pose
        self.goal = L.goal_key(p.position.x, p.position.y, p.orientation.z, p.orientation.w)
        if self.sc:
            if self.goal != self.sc.goal:  # el eco de la republicacion no cambia el mensaje guardado
                self.goal_msg = m
            self.sc_result(self.sc.on_goal(self.goal), 'Meta nueva')

    def appr_cb(self, m):
        self.approved_key = m.data
        if self.sc:
            self.sc.on_approved(m.data)

    def tick(self):
        now = self.now()
        if self.sc:
            self.sc_result(self.sc.tick(now), 'Tiempo de meta (%.0f s)' % L.META_TIMEOUT)
            self.active = self.sc.state == 'RUN'
        if self.active and not self.force and (self.t_low is None or now - self.t_low > L.LOWSTATE_TIMEOUT):
            self.active = self.moving = False  # sin control no hay botón de paro
            if self.sc:
                self.sc.end()
            self.start_stop()
            self.get_logger().warn('Sin /lf/lowstate > %.1f s: StopMove y IDLE' % L.LOWSTATE_TIMEOUT)
        if now < self.stop_until:
            self.send(API_STOP)
        elif self.active:
            c = (L.move_command(now, self.cmd, self.t_cmd, now, True) if self.sc  # RUN: sin deadman
                 else L.move_command(now, self.cmd, self.t_cmd, self.t_dead,
                                     self.goal is not None and self.approved_key == self.goal))
            if c:
                self.moving = True
                self.send(API_MOVE, '{"x": %.3f, "y": %.3f, "z": %.3f}' % c)
            elif self.moving:  # paso de mover a no poder: 1 s de StopMove y silencio
                self.moving = False
                self.start_stop()
        state = 'SC:' + self.sc.state if self.sc else 'STOP' if now < self.stop_until else (
            ('MOVING' if self.moving else 'ACTIVE') if self.active else 'IDLE')
        self.state_pub.publish(String(data=state))

    def destroy_node(self):
        if self.active or self.moving or (self.sc and self.sc.state == 'RUN'):  # StopMove al cerrar; no mandar nada si siempre estuvo IDLE
            for _ in range(3):
                self.send(API_STOP)
        super().destroy_node()


def _as_ctrl_c(*_):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = CmdVelToSport()
    # Cerrar tmux/ssh manda SIGHUP (o SIGTERM): tratarlo como Ctrl-C para que salga el StopMove.
    signal.signal(signal.SIGHUP, _as_ctrl_c)
    signal.signal(signal.SIGTERM, _as_ctrl_c)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
