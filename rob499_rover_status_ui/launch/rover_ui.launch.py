#!/usr/bin/env python3 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='rob499_rover_status_ui',
            executable='node_info',
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='node_log',
        ),
		Node(
			package='rob499_rover_status_ui',
			executable='node_topic_detector'
            ),
		Node(
			package='rob499_rover_status_ui',
            executable='moveit_logger',
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='drivetrain_telemetry',
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='drive_slip',
        ),
		
    ])
