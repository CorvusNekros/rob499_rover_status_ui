#!/usr/bin/env python3 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='rob499_rover_status_ui',
            executable='node_info',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='node_log',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='node_topic_detector',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
		Node(
			package='rob499_rover_status_ui',
            executable='moveit_logger',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
        Node(
            package ='rob499_rover_status_ui',
			executable='demo_oscope',
			arguments=[
                "--ros-args",
				"--disable-stdout-logs"]
		),
        Node(
            package ='rob499_rover_status_ui',
			executable='demo_limiter',
			arguments=[
                "--ros-args",
				"--disable-stdout-logs"]
        ),
    ])
