#!/usr/bin/env python3
"""
Publica un archivo .ply (x, y, z) como PointCloud2 para verlo en RViz.

Uso:
  ros2 run autonomy_dashboard_pkg ply_publisher.py --ros-args \
      -p file:=/ruta/al/mapa.ply -p frame_id:=map -p topic:=/ply_cloud
"""

import sys
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def load_ply_xyz(path):
    """Lee x, y, z de un PLY ASCII o binario little-endian."""
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError('No es un archivo PLY')
        fmt, n_vertex, props, in_vertex = None, 0, [], False
        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('format'):
                fmt = line.split()[1]
            elif line.startswith('element'):
                in_vertex = line.split()[1] == 'vertex'
                if in_vertex:
                    n_vertex = int(line.split()[2])
            elif line.startswith('property') and in_vertex:
                _, ptype, name = line.split()
                props.append((name, ptype))
            elif line == 'end_header':
                break

        names = [p[0] for p in props]
        cols = [names.index(c) for c in ('x', 'y', 'z')]

        if fmt == 'ascii':
            data = np.loadtxt(f, max_rows=n_vertex, ndmin=2)
            return data[:, cols].astype(np.float32)
        if fmt == 'binary_little_endian':
            types = {'float': 'f4', 'float32': 'f4', 'double': 'f8',
                     'uchar': 'u1', 'uint8': 'u1', 'char': 'i1', 'int8': 'i1',
                     'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
                     'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4'}
            dt = np.dtype([(n, '<' + types[t]) for n, t in props])
            data = np.fromfile(f, dtype=dt, count=n_vertex)
            return np.column_stack((data['x'], data['y'], data['z'])).astype(np.float32)
        raise ValueError(f'Formato PLY no soportado: {fmt}')


class PlyPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        path = self.declare_parameter('file', '').value
        frame = self.declare_parameter('frame_id', 'map').value
        topic = self.declare_parameter('topic', '/ply_cloud').value
        rate = self.declare_parameter('rate', 1.0).value

        if not path:
            self.get_logger().error('Falta el parámetro "file" (ruta al .ply)')
            sys.exit(1)

        points = load_ply_xyz(path)
        self.get_logger().info(f'{len(points)} puntos cargados de {path}')

        # Transient local: RViz recibe la nube aunque se abra después
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.msg = pc2.create_cloud_xyz32(Header(frame_id=frame), points)
        self.publish()
        if rate > 0:
            self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f'Publicando en {topic} (frame: {frame})')

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = PlyPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
