#!/usr/bin/env python3




import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names


# We're going to publish a Float32.
from std_msgs.msg import Float32

class ListenTest(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('listener')

		# Create a publisher, and assign it to a member variable.T
		# The call takes a type, topic name, and queue size.
		self.pub = self.create_publisher(Float32, 'nodetopiclisten', 10)

		self.publishing_period = 1 
		self.timer = self.create_timer(self.publishing_period, self.callback)
		self.seen_topic_list = []
		self.seen_node_list = []

	# This callback will be called	 every time the timer fires.
	def callback(self):
		
		#checks what current topic list is
		self.current_topic_list = []
		topic_list = get_topic_list()
		for info in topic_list:
			self.current_topic_list.append(info[0])
		
		#updates list of previously seen topics
		for n in self.current_topic_list:
			if self.current_topic_list in self.seen_topic_list:
				pass
			else:
				self.seen_topic_list.append(self.current_topic_list[n])

		#creates list of topic status for output - 2nd list instead of dictionary because of ROS msg typing
		topic_status = []
		#compares current topics with all previously seen topics
		for n in self.seen_topic_list:
			if self.seen_topic_list[n] in self.current_topic_list: #true if previously seen topic is currently seen
				topic_status.append('alive')
			else:
				topic_status.append('dead')




		#checks what current node list is
		self.current_node_list = []
		node_list = get_node_list()
		for info in node_list:
			self.current_node_list.append(info[0])
		
		#updates list of previously seen nodes
		for n in self.current_node_list:
			if self.current_node_list in self.seen_node_list:
				pass
			else:
				self.seen_node_list.append(self.current_node_list[n])

		#creates list of node status for output - 2nd list instead of dictionary because of ROS msg typing
		node_status = []
		#compares current nodes with all previously seen nodes
		for n in self.seen_node_list:
			if self.seen_node_list[n] in self.current_node_list: #true if previously seen node is currently seen
				node_status.append('alive')
			else:
				node_status.append('dead')



		#TODO: create .msg & publish here, need to import it and change type of publsiher 
		'''
		string[] topic_names
		string[] topic_status
		string[] node_names
		string[] node_status
		
		
		'''



		self.get_logger().info(f'current topics: {self.current_topic_list}')
		self.get_logger().info(f'current nodes: {self.current_node_list}')


# thank you: https://robotics.stackexchange.com/questions/97965/how-to-show-topics-using-python-in-ros2
def get_topic_list():
	node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
	topic_list = node_dummy.get_topic_names_and_types()
	node_dummy.destroy_node()
	return topic_list

def get_node_list():
	node_dummy = Node("_ros2cli_dummy_to_show_node_list")
	node_list = node_dummy.get_node_names()
	node_dummy.destroy_node()
	return node_list

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make a node class.
	listen = ListenTest()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(listen)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
