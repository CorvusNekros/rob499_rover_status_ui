#!/usr/bin/env python3 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
return LaunchDescription([
		Node(
			package='rob499_rover_status_ui',
            executable='node_info',
            output='screen',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='node_logs',
            output='screen',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='node_topic_detector',
            output='screen',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='integrator',
            output='screen',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),

    ])
