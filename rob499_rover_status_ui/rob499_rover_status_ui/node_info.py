#!/usr/bin/env python3


# service that returns info about a node
#
# node_info.py
#
# Wyatt Boer



import rclpy
from rclpy.node import Node

# Service call type
from rob499_rover_status_ui_interfaces.srv import NodeInfo

class NodeInfoService(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('node_info')

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(NodeInfo, 'node_info', self.callback)

	# This callback will be called every time that the service is called.
	def callback(self, request, response):
		
		self.nodename = request.node



		response.pub_topics = self.get_pub_topics()
		response.sub_topics = self.get_sub_topics()
		response.parameters = "ERROR! test value, this should not appear if the code works"
		response.services = "ERROR! test value, this should not appear if the code works"

		self.get_logger().info(f'responded with: {response}')

		return response

	# I dont know if this works when multiple topics are published from one node, also i dont think the data type is correct for the response
	#not even sure if this is working correctly at all
	def get_pub_topics(self):
		output_list = []
		pub_topic_list = self.get_publisher_names_and_types_by_node(self.nodename,'/')
		for topics in pub_topic_list:
			if topics[0] == '/'+str(self.nodename):
				output_list.append(topics[0])
		return output_list

		

	def get_sub_topics(self):
		output_list = []
		sub_topic_list = self.get_subscriber_names_and_types_by_node(self.nodename,'/')
		for topics in sub_topic_list:
			if topics[0] == '/'+str(self.nodename):
				output_list.append(topics[0])
		return output_list

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make a node class.
	info = NodeInfoService()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(info)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
