import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

_NP = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2',
       PointField.UINT16: 'u2', PointField.INT32: 'i4', PointField.UINT32: 'u4',
       PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


class L1ToBody(Node):
    def __init__(self):
        super().__init__('l1_to_body')
        for n, d in (('cloud_topic', '/utlidar/cloud'), ('imu_topic', '/utlidar/imu'),
                     ('out_frame', 'body'), ('cloud_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                     ('cloud_z_offset', 0.0), ('cloud_t', [float('nan')] * 3), ('self_box', [0.0] * 6),
                     ('imu_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                     ('gyro_bias', [0.0] * 3), ('gyro_scale', 1.0), ('ang_z2x_proj', 0.0),
                     ('ang_z2y_proj', 0.0), ('zero_acc', True)):
            self.declare_parameter(n, d)
        g = lambda n: self.get_parameter(n).value
        self.frame = g('out_frame')
        self.cR = np.array(g('cloud_R'), dtype=np.float64).reshape(3, 3)
        self.cz = g('cloud_z_offset')
        ct = np.array(g('cloud_t'), dtype=np.float64)  # nan = sin definir -> [0, 0, cloud_z_offset] (comportamiento anterior)
        self.ct = np.array([0.0, 0.0, self.cz]) if np.isnan(ct).any() else ct
        self.box = np.array(g('self_box'), dtype=np.float64)
        self.iR = np.array(g('imu_R'), dtype=np.float64).reshape(3, 3)
        self.bias = np.array(g('gyro_bias'), dtype=np.float64)
        self.scale, self.z2x, self.z2y = g('gyro_scale'), g('ang_z2x_proj'), g('ang_z2y_proj')
        self.zero_acc = g('zero_acc')
        self.offset = None  # ns; now() - stamp (los stamps de la .161 van atrasados)
        self.cloud_pub = self.create_publisher(PointCloud2, '/utlidar/transformed_cloud', 100)  # reliable: Point-LIO se suscribe con QoS por defecto
        self.imu_pub = self.create_publisher(Imu, '/utlidar/transformed_imu', 100)  # reliable: Point-LIO se suscribe con QoS por defecto
        self.create_subscription(PointCloud2, g('cloud_topic'), self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic'), self.imu_cb, qos_profile_sensor_data)

    def shift(self, header):
        raw = header.stamp.sec * 1000000000 + header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if self.offset is None or abs(now - (raw + self.offset)) > 500000000:
            if self.offset is not None:
                self.get_logger().warn('Offset de reloj desfasado >0.5 s; recalculando',
                                       throttle_duration_sec=5.0)
            self.offset = now - raw
        header.stamp.sec, header.stamp.nanosec = divmod(raw + self.offset, 1000000000)
        header.frame_id = self.frame

    def cloud_cb(self, msg):
        self.shift(msg.header)
        dt = np.dtype({'names': [f.name for f in msg.fields],
                       'formats': [_NP[f.datatype] for f in msg.fields],
                       'offsets': [f.offset for f in msg.fields],
                       'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height).copy()
        p = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64) @ self.cR.T
        p += self.ct
        b = self.box
        inside = ((p[:, 0] > b[0]) & (p[:, 0] < b[1]) & (p[:, 1] > b[2]) & (p[:, 1] < b[3])
                  & (p[:, 2] > b[4]) & (p[:, 2] < b[5]))
        keep = np.isfinite(p).all(axis=1) & ~inside
        a['x'], a['y'], a['z'] = p[:, 0], p[:, 1], p[:, 2]
        a = a[keep]
        msg.data = a.tobytes()
        msg.height, msg.width = 1, len(a)
        msg.row_step, msg.is_dense = msg.point_step * len(a), True
        self.cloud_pub.publish(msg)

    def imu_cb(self, msg):
        self.shift(msg.header)
        w = self.iR @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        w = (w - self.bias) * self.scale
        w[0] += self.z2x * w[2]
        w[1] += self.z2y * w[2]
        out = Imu()
        out.header = msg.header
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, w)
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        if not self.zero_acc:
            a = self.iR @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                                    msg.linear_acceleration.z])
            out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, a)
        self.imu_pub.publish(out)


def main():
    rclpy.init()
    node = L1ToBody()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
