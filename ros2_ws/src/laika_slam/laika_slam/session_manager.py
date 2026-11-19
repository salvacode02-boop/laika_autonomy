"""Reinicio de sesion de SLAM: dueno de slam_toolbox (subproceso) y servicio /laika/reset_session."""
import os
import signal
import subprocess
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

IDLE_STATES = ('IDLE', 'SC:IDLE')
# Un FAIL en estos pasos hace fallar el reinicio; en los demas solo se anota.
CRITICAL = {1, 4, 5, 7, 8}
STEPS = (  # (numero, metodo de `io`, texto)
    (1, 'check_idle', 'puente en IDLE'),
    (2, 'cancel_goals', 'cancelar metas de Nav2'),
    (3, 'save_map', 'autosave del mapa'),
    (4, 'stop_slam', 'detener slam_toolbox'),
    (5, 'reset_odom', 'reiniciar odometría'),
    (6, 'clear_state', 'limpiar costmaps, map3d y heightmap'),
    (7, 'bump_session', 'session_id += 1'),
    (8, 'start_slam', 'reiniciar slam y esperar /map + map->base_link'),
)


def run_reset(io, log=lambda s: None):
    """Ejecuta los 8 pasos en orden. `io.<metodo>()` -> (ok, detalle). Devuelve (success, lineas).

    Paso 1 en FAIL aborta sin tocar nada. Un FAIL critico tras detener slam salta al paso 8
    (no se deja al robot sin SLAM); el resultado es False.
    """
    lines, success, stopped = [], True, False
    todo = list(STEPS)
    while todo:
        n, fn, text = todo.pop(0)
        try:
            ok, detail = getattr(io, fn)()
        except Exception as e:  # noqa
            ok, detail = False, repr(e)
        line = '%s %d %s%s' % ('PASS' if ok else 'FAIL', n, text, ': ' + detail if detail else '')
        lines.append(line)
        log(line)
        stopped = stopped or (n == 4)
        if not ok and n in CRITICAL:
            success = False
            if n == 1:
                break
            if n < 8 and stopped:
                todo = [s for s in todo if s[0] == 8]  # salta al reinicio de slam
            elif n == 8:
                break
    return success, lines


class SessionManager(Node):
    def __init__(self):
        super().__init__('session_manager')
        share = get_package_share_directory('laika_slam')
        self.declare_parameter('slam_cmd', [
            'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
            '-r', '__node:=slam_toolbox', '--params-file', os.path.join(share, 'config', 'slam_async.yaml')])
        self.declare_parameter('autosave_dir', '~/laika_maps/autosave')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_slam', True)
        g = lambda n: self.get_parameter(n).value
        self.cmd = list(g('slam_cmd'))
        self.autosave = os.path.expanduser(g('autosave_dir'))
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')
        self.cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sid = 0
        self.sid_pub = self.create_publisher(Int32, '/laika/session_id', latched)
        self.sid_pub.publish(Int32(data=0))
        self.state, self.t_state = '', 0.0
        self.create_subscription(String, '/laika/bridge_state', self.state_cb, 10, callback_group=self.cb)
        self.map_seq = 0
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched, callback_group=self.cb)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.child = None
        self.busy = threading.Lock()
        self.create_service(Trigger, '/laika/reset_session', self.reset_cb, callback_group=self.cb)
        if g('start_slam'):
            self.launch_slam()

    # --- callbacks
    def state_cb(self, m):
        self.state, self.t_state = m.data, time.monotonic()

    def map_cb(self, _m):
        self.map_seq += 1

    def reset_cb(self, _req, res):
        if not self.busy.acquire(blocking=False):
            res.success, res.message = False, 'FAIL: ya hay un reinicio en curso'
            return res
        try:
            t0 = time.monotonic()
            res.success, lines = run_reset(self, lambda s: self.get_logger().info(s))
            res.message = '\n'.join(lines + ['%.1f s' % (time.monotonic() - t0)])
        finally:
            self.busy.release()
        return res

    # --- subproceso de slam
    def launch_slam(self):
        self.child = subprocess.Popen(self.cmd, start_new_session=True)

    def _wait_service(self, cli, t=1.0):
        return cli.wait_for_service(timeout_sec=t)

    def _call(self, cli, req, timeout):
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    # --- pasos (cada uno devuelve (ok, detalle))
    def check_idle(self):
        age = time.monotonic() - self.t_state
        if age > 1.0:
            return False, 'sin /laika/bridge_state reciente (%.1f s)' % age
        return (True, '') if self.state in IDLE_STATES else (False, 'estado %s: pulsa L2+Y' % self.state)

    def cancel_goals(self):
        cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=self.cb)
        if not self._wait_service(cli):
            return True, 'omitido (Nav2 no disponible)'
        r = self._call(cli, CancelGoal.Request(goal_info=GoalInfo()), 2.0)  # id y stamp en cero = todas
        return (r is not None), '' if r is not None else 'sin respuesta'

    def save_map(self):
        os.makedirs(self.autosave, exist_ok=True)
        path = os.path.join(self.autosave, time.strftime('%Y%m%d_%H%M%S') + '_failed')
        try:
            p = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
                               timeout=5, capture_output=True)
        except subprocess.TimeoutExpired:
            return False, 'map_saver_cli > 5 s'
        return p.returncode == 0, path

    def stop_slam(self):
        if self.child is None or self.child.poll() is not None:
            self.child = None
            return True, 'no estaba corriendo'
        os.killpg(self.child.pid, signal.SIGINT)
        try:
            self.child.wait(5.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.child.pid, signal.SIGKILL)
            self.child.wait()
        self.child = None
        return True, ''

    def reset_odom(self):
        cli = self.create_client(Trigger, '/laika/reset_odom', callback_group=self.cb)
        if not self._wait_service(cli, 2.0):
            return False, '/laika/reset_odom no disponible'
        r = self._call(cli, Trigger.Request(), 3.0)
        return (r.success, r.message) if r else (False, 'sin respuesta')

    def clear_state(self):
        notes = []
        for name, typ in (('/global_costmap/clear_entirely_global_costmap', ClearEntireCostmap),
                          ('/local_costmap/clear_entirely_local_costmap', ClearEntireCostmap),
                          ('/map3d_accumulator/reset', Trigger), ('/heightmap_node/reset', Trigger)):
            cli = self.create_client(typ, name, callback_group=self.cb)
            if not self._wait_service(cli, 0.5):
                notes.append('sin %s' % name.split('/')[1])
                continue
            if self._call(cli, typ.Request(), 2.0) is None:
                notes.append('timeout %s' % name.split('/')[1])
        return True, ', '.join(notes)

    def bump_session(self):
        self.sid += 1
        self.sid_pub.publish(Int32(data=self.sid))
        return True, str(self.sid)

    def start_slam(self):
        seq = self.map_seq
        self.launch_slam()
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            if self.child.poll() is not None:
                return False, 'slam terminó (código %s)' % self.child.returncode
            if self.map_seq > seq and self.tfbuf.can_transform(self.map_frame, self.base_frame, rclpy.time.Time()):
                return True, ''
            time.sleep(0.1)
        return False, 'sin /map o %s->%s en 8 s' % (self.map_frame, self.base_frame)

    def destroy_node(self):
        if self.child is not None and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGKILL)
        super().destroy_node()


def main():
    rclpy.init()
    node = SessionManager()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
