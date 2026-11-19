from glob import glob
from setuptools import setup

package_name = 'laika_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laika',
    maintainer_email='laika@example.com',
    description='TF y SLAM 2D para Laika',
    license='MIT',
    entry_points={'console_scripts': [
        'odom_to_tf = laika_slam.odom_to_tf:main',
        'cmd_vel_to_sport = laika_slam.cmd_vel_to_sport:main',
        'joint_states = laika_slam.joint_states:main',
        'l1_to_body = laika_slam.l1_to_body:main',
        'map3d_accumulator = laika_slam.map3d_accumulator:main',
        'heightmap_node = laika_slam.heightmap_node:main',
    ]},
)
