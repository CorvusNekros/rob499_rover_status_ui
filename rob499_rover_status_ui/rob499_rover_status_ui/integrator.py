#!/usr/bin/env python3


# integrates all data from nodes and sends it to UI
#
# integrator.py
#
# Wyatt Boer


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node




# We're subscribed to a NodesTopics custom message
from rob499_rover_status_ui_interfaces.msg import NodesTopics

# Our custom services
from rob499_rover_status_ui_interfaces.srv import NodeInfo
from rob499_rover_status_ui_interfaces.srv import LatencySize



class Integrator(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('integrator')

		# Create the subscriber.
		self.node_topic_status = self.create_subscription(NodesTopics, 'nodetopiclisten', self.node_topic_callback, 10)

		#Subscribe to both /nodetopiclisten and /log_filt

	# This callback will be called every time that the nodetopiclisten topic is published to.
	# Just updates the class lists:
	def node_topic_callback(self, msg):
		
		#data we get from our callback
		msg.topic_name 
		msg.topic_status 
		msg.node_name 
		msg.node_status  

		return 

	#This callback is called every time a new node_log message is published. 
	def node_log_callback

# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	integrate = Integrator()

	#TODO: While loop with table stuff here:

		#Spin once each loop iteratoion

	#Construct/Update tables:

	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()
