#!/usr/bin/env python3


# integrates all data from nodes and sends it to UI
#
# integrator.py
#
# Wyatt Boer & Osian Leahy


# Every Python node in ROS2 should include these lines. rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node. uwu
import rclpy
from rclpy.node import Node

# We're subscribed to NodesTopics and FilteredLog custom messages
from rob499_rover_status_ui_interfaces.msg import NodesTopics
from rob499_rover_status_ui_interfaces.msg import FilteredLog

# Our custom services
from rob499_rover_status_ui_interfaces.srv import NodeInfo
from rob499_rover_status_ui_interfaces.srv import LatencySize
from rob499_rover_status_ui_interfaces.srv import NodeLog

#We're using parameters:
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

#Import rich libraries for table generation:
from rich.live import Live
from rich.table import Table

class Integrator(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('integrator')

		#whether or not to use table display:
		self.table_display = True
	
		self.node_select_needs_updating = False

		# Create the subscribers
		#Subscribe to both /nodetopiclisten and /log_filt
		self.node_topic_status = self.create_subscription(NodesTopics, 'nodetopiclisten', self.node_topic_callback, 10)
		self.node_logs = self.create_subscription(FilteredLog, 'log_filt', self.node_log_callback, 10)

		#We have three main data sets to store/display from the subscribed messages:
		#First is the Node and Topic lists:
		self.topic_names = []
		self.topic_statuses = []
		self.node_names = []
		self.node_statuses = []
		#Second is the Node Logs:
		self.log_stamp = self.get_clock().now().to_msg()
		self.log_name = ""
		self.log_level = 0
		self.log_msg = ""
		
		#Third is our Node Topics/Services/Params:
		self.node_pubs = []
		self.node_subs = []
		self.node_params = []
		self.node_services = []

		#Setup the service clients
		#We can select a node for log info, and get a list of a nodes pub/subbed topics, paramters, services 
		self.info_cli = self.create_client(NodeInfo, 'node_info')
		self.log_select_cli = self.create_client(NodeLog, 'node_log')

		#These service calls will be fed by node name/namespace parameters
		# The controlling node (unity in the future) can update this parameter to change the node info/log data sent.
		self.declare_parameter('node_select', '_NULL')
		self.declare_parameter('namespace_select', '/')

		#We're gonna use a parameter callback to handle downstream updates:
		self.param_handler = ParameterEventHandler(self)
		self.callback_handle = self.param_handler.add_parameter_callback(
			parameter_name = 'node_select',
			node_name = 'integrator',
			callback = self.param_callback
		) #Should probably do this for the namespace too?

		#Before exiting the init step, we want to know that the info/log select services are available, otherwise chaos:
		while not (self.info_cli.wait_for_service(timeout_sec=1) or self.log_select_cli.wait_for_service(timeout_sec=1)):
			self.get_logger().info('Waiting for a service to start')

	### Define our class Functions ###

	# This callback will be called every time that the nodetopiclisten topic is published to.
	# Just updates the class lists:
	def node_topic_callback(self, msg):
		#Quick logging message:
		self.get_logger().info('Recieved node/topic status list message')

		#Save the most recent data we get from our callback
		self.topic_names =  msg.topic_name 
		self.topic_statuses = msg.topic_status 
		self.node_names = msg.node_name 
		self.node_statuses = msg.node_status

	#This callback is called every time a new node_log message is published. 
	def node_log_callback(self, msg):
		# Sets current time to use to calculate node latency using log stamp later
		current_time = self.get_clock().now().nanoseconds
		
		#Quick Logging message:
		self.get_logger().info('Recieved Node log message')

		#Save the most recent log we got from our callback
		self.log_stamp = msg.stamp
		self.log_name = msg.name
		self.log_level = msg.level
		self.log_msg = msg.msg

		#TODO probably nicer update this to average the latency across last 5-10 values or smth instead of raw
		# Calculating latency
		self.node_latency = current_time - msg.stamp.nanosec

	#If we need to update the selected node to introspect, do two service calls:
	#TODO: Is there a clean non-blocking way to do this better, or should we not care...
	def update_node_selection(self):
		
		#First build the requests
		self.info_request = NodeInfo.Request()
		self.info_request.node = self.get_parameter('node_select').get_parameter_value().string_value
		
		self.log_select = NodeLog.Request()
		self.log_select.enable = True
		self.log_select.hz = 1.0
		self.log_select.node = self.get_parameter('node_select').get_parameter_value().string_value

		#Then make the async calls and wait for them to return
		self.info_future = self.info_cli.call_async(self.info_request) #potential for race condition shennanigans here?
		self.log_future = self.log_select_cli.call_async(self.log_select)
		rclpy.spin_until_future_complete(self, self.info_future)
		rclpy.spin_until_future_complete(self, self.log_future) #probably don't need this

		#Unpack the returned info (nothing to unpack in the log response):
		info_response = self.info_future.result()
		self.node_pubs = info_response.pub_topics
		self.node_subs = info_response.sub_topics
		#self.node_params = info_response.parameters
		self.node_services = info_response.services

	#When the parameter changes, we should update the node selection downstream:
	def param_callback(self, parameter):
		#Quick logging message:
		self.get_logger().info(f'Parameter Changed: {parameter.name} = {parameter_value_to_python(parameter.value)}')

		#Prepare to call update_node_selection() since our node of interest changed:
		self.node_select_needs_updating = True

	#Creates a table of nodes and their dead/alive statuses
	def generate_node_status_table(self):
		#Generate the table headers:
		table = Table(title="Node Status")
		table.add_column("Node")
		table.add_column("Status")

		#fill in the table data:
		for node,status in zip(self.node_names,self.node_statuses):
			self.get_logger().info(f'row attempt: {str(node)}, {str(status)}')
			table.add_row(str(node),str(status))
		
		return table

	#Creates a table of node info: pubs/subs/params/services:
	def generate_node_info_table(self):
		#Layout each method as a row, unsure what this will do since they're diff. lengths
		table = Table(title="Node Communications")
		table.add_row("Subscribed Topics", *self.node_subs)
		table.add_row("Publishing Topics", *self.node_pubs)
		table.add_row("Services", *self.node_services)

		table.show_header = False
		table.show_lines = True

		return table

	#Creates a table of simplified node log info:
	def generate_node_log_table(self):
		
		#Dictionary of message levels:
		levels = {10:"DEBUG",
				  20:"INFO",
				  30:"WARN",
				  40:"ERROR",
				  50:"FATAL"}

		table = Table(title=f"{self.log_name} Log Info")
		table.add_row('time', f'{self.log_stamp.sec}.{self.log_stamp.nanosec}')
		table.add_row("level",levels.get(self.log_level))
		table.add_row('msg',self.log_msg)
		table.add_row('latency',self.node_latency)
		
		table.show_header = False
		table.show_lines = True

		return table

	def generate_tables(self):
		status_table = self.generate_node_status_table()
		info_table = self.generate_node_info_table()
		log_table = self.generate_node_log_table()

		grid = Table(box=None)
		grid.add_row(status_table,info_table,log_table)

		grid.show_header = False
		grid.show_lines = False
		grid.show_edge = False
	

		return grid

# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	integrate = Integrator()
	
	with Live(integrate.generate_tables(), refresh_per_second=15) as live:

		while rclpy.ok():
			#Check if we need to update the node to introspect
			if integrate.node_select_needs_updating:
				integrate.node_select_needs_updating = False
				integrate.update_node_selection()
		
			live.update(integrate.generate_tables())
			#Spin once each loop iteration
			rclpy.spin_once(integrate)

	#TODO: While loop with table stuff here:
		# We want 3 tables:
		# - The first table displays a list of node statuses
		# - The second table displays the selected node and it's topics
		# - The Third table displays the most recent log info of a selected node

	#Construct/Update tables:

	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()
