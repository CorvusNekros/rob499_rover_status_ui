from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'rob499_rover_status_ui'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.py'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.xml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Wyatt Boer',
    maintainer_email='wyattboer@gmail.com',
    description='ROS2 implementation for improved rover status ui',
    license='BSD-3-Clause',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #test listening node for topics/nodes, needs to be confirmed working
            'node_topic_detector = rob499_rover_status_ui.node_topic_detector:main',
            'latency = rob499_rover_status_ui.latency:main',
            'node_info = rob499_rover_status_ui.node_info:main',
			'node_log = rob499_rover_status_ui.node_logs:main',
            'integrator = rob499_rover_status_ui.integrator:main',

            'moveit_logger = rob499_rover_status_ui.moveit_logger:main',
            
			'drivetrain_telemetry = rob499_rover_status_ui.odrive_telemetry:drivetrain_telem',
			'arm_telemetry = rob499_rover_status_ui.odrive_telemetry:arm_telem',
			'drive_slip = rob499_rover_status_ui.drivetrain_slip_detection:main',

			'demo_oscope = rob499_rover_status_ui.DEMO_oscope:normal_wave',
			'demo_limiter = rob499_rover_status_ui.DEMO_limiter:limit',
	   ],
    },
)
