import os
import sys

import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from laika_slam.l1_to_body import L1ToBody


def test_cloud_t_default_equals_old_offset():
    rclpy.init()
    try:
        n = L1ToBody()
        assert np.allclose(n.ct, [0.0, 0.0, n.cz])
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_cloud_t_override():
    rclpy.init(args=['--ros-args', '-p', 'cloud_t:=[0.1,0.2,0.3]', '-p', 'cloud_z_offset:=-0.5'])
    try:
        n = L1ToBody()
        assert np.allclose(n.ct, [0.1, 0.2, 0.3])
        n.destroy_node()
    finally:
        rclpy.shutdown()
