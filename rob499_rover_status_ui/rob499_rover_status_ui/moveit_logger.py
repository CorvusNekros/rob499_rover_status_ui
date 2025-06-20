#!/usr/bin/env python3


# Subscribes to MoveIt stack topics and republishes relevant data for use in Rover UI
#
# moveit_logger.py
#
# Wyatt Boer
#
# This node subscribes to a topic with Int64s.  It multiplies the numbers by a constant,
# and then republishes them on another topic.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to subscribe to a topic with type JointState
from sensor_msgs.msg import JointState

from std_msgs.msg import Int8

from std_msgs.msg import Header

# type of what we are publishing
from rob499_rover_status_ui_interfaces.msg import MoveItLogs

class MoveItLogger(Node):
	
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('moveit_logger')

		# Create the publisher first, to make sure it's available before we start the subscriber.
		self.pub = self.create_publisher(MoveItLogs, 'moveit_logs', 10)

		# Create the joint subscriber.
		self.jointsub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

		# the rate that this receives data from topic is publishing rate, because the status is the main thing we care about
		# Create the status subscriber.
		self.statussub = self.create_subscription(Int8, 'servo_node/status', self.status_callback, 10)

		self.header = Header()

		#initialize status so that we dont publish a lot of msgs/sec
		self.published_status = []

		self.name = []
		self.position = []
		self.velocity = []
		self.effort	= []

		#initialized as 0
		self.prev_pub_sec = 0
		self.joint_time = 0
	# This callback will be called whenever we receive a new message on the topic.
	def joint_state_callback(self, msg):
		# puts data inside of self. variables
		self.header = msg.header
		self.joint_time = msg.header.stamp.sec
		self.name = msg.name
		self.position = msg.position
		self.velocity = msg.velocity
		self.effort = msg.effort

	def status_callback(self,msg):
	
		# Make an MoveItLogs custom message
		new_msg = MoveItLogs()

		# stores relevant status string in new msg

		new_msg.status = msg.data
		
		# fills in info from states
		new_msg.header = self.header
		new_msg.name = self.name
		new_msg.position = self.position
		new_msg.velocity = self.velocity
		new_msg.effort = self.effort
		#only publish approx every second or if status changes
		time = self.joint_time


		if ((self.prev_pub_sec+1) < (time)) or (self.published_status != new_msg.status):
			
			self.published_status = new_msg.status
			self.prev_pub_sec = time
			#publish our message
			self.pub.publish(new_msg)

			# Log that we published something.  In ROS2, loggers are associated with nodes, and
			# the idiom is to use the get_logger() call to get the logger.  This has functions
			# for each of the logging levels.
			self.get_logger().info(f'Published {new_msg}')

# This is a entry point.	
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	logger = MoveItLogger()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(logger)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
