"""/lf/lowstate (unitree_go/LowState) -> /joint_states (sensor_msgs/JointState) a 20 Hz.

Orden de motor_state del Go2 (unitree_sdk2 / unitree_ros2): 0-2 FR (hip, thigh, calf), 3-5 FL, 6-8 RR, 9-11 RL.
Los nombres son los del URDF go2_description. Stamp = now() (el de la .161 va atrasado)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

NAMES = [f'{leg}_{j}_joint' for leg in ('FR', 'FL', 'RR', 'RL') for j in ('hip', 'thigh', 'calf')]


class JointStates(Node):
    def __init__(self):
        super().__init__('joint_states')
        self.declare_parameter('rate', 20.0)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(LowState, '/lf/lowstate', self.cb, qos_profile_sensor_data)
        self.last = None
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def cb(self, msg):
        self.last = ([m.q for m in msg.motor_state[:12]], [m.dq for m in msg.motor_state[:12]])

    def tick(self):
        if self.last is None:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = NAMES
        out.position = [float(q) for q in self.last[0]]
        out.velocity = [float(d) for d in self.last[1]]
        self.last = None  # sin datos nuevos no se republica (el modelo no finge estar vivo)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
