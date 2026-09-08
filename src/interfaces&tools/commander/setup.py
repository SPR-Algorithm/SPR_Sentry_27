from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'UI_classes'), glob(os.path.join('UI_classes', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lmh',
    maintainer_email='lmh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'cmd_vel_subscriber     = commander.cmd_vel_sub:main',
          'navigation_client    = commander.navigation_client:main',
          'navigation_through_poses_client    = commander.navigation_through_poses_client:main',
          'commander    = commander.commander:main',
          'commander_UI    = commander.commander_UI:main',
          'time_perception    = commander.time_perception:main',
          'time_sub    = commander.time_sub:main',
          'event_perception    = commander.event_perception:main',
          'navigation_client_cancel    = commander.navigation_client_cancel:cancel_navigation_goal',
          'odom_subscriber    = commander.odom_subscriber:main',
          'init_pose    = commander.init_pose:main',
          'sub_tf   = commander.sub_tf:main',
          'dynamic_pose    = commander.dynamic_pose:main',
          'lidar    = commander.lidar:main',
          'icp_localization    = commander.icp_localization:main',
          'cancel_demo    = commander.cancel_demo:main',
          'controller_serial    = commander.controller_serial:main',
          'controller_serial_1    = commander.controller_serial_1:main',
          'controller_serial_r    = commander.controller_serial_r:main',
          'controller_serial_b    = commander.controller_serial_b:main',
          'imu_to_odom    = commander.imu_to_odom:main',
          'pass_hole    = commander.pass_hole:main',
          'map_edit    = commander.map_edit:main',
          'enermy_position    = commander.enermy_position:main',
          'odom_aim_tf    = commander.odom_aim_tf:main',
        ],
    },
)
