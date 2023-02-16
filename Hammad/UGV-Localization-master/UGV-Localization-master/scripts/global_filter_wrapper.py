#!/usr/bin/env python

import sys
import rospy
import subprocess

import numpy as np

import message_filters

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf import transformations


class GlobalFilterWrapper(object):

	def __init__(self, topic_wheel_odom, topic_gps_odom, topic_gps_heading, config_file, filter_node_name, topic_out):
		# configure subscriber and publisher
		self.sub_wheel_odom = message_filters.Subscriber(topic_wheel_odom, Odometry)
		self.sub_gps_odom = message_filters.Subscriber(topic_gps_odom, Odometry)
		self.sub_gps_heading = message_filters.Subscriber(topic_gps_heading, Imu)
		self.subs = [self.sub_wheel_odom, self.sub_gps_odom, self.sub_gps_heading]
		
		# time synchronizer
		self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, 10, 0.2)
		self.ts.registerCallback(self.callback)

		# initial state
		self.init_state = [0.0 for _ in range(15)]
		self.ready = False # flag indicating whether initial state is set

		# config file
		self.config_file = config_file

		# topic name for output
		self.topic_out = topic_out

		# filter node name
		self.filter_node_name = filter_node_name	


	def callback(self, msg_wheel, msg_gps_odom, msg_gps_heading):
		print('Initializing states ...')
		if self.ready:
			for sub in self.subs:
				sub.unregister()
			self.run_filter()
		else:
			self.set_init_state(msg_wheel, msg_gps_odom, msg_gps_heading)


	def set_init_state(self, msg_wheel, msg_gps_odom, msg_gps_heading):
		# position
		self.init_state[0] = msg_gps_odom.pose.pose.position.x
		self.init_state[1] = msg_gps_odom.pose.pose.position.y
		# orientation
		q = msg_gps_heading.orientation
		euler = transformations.euler_from_quaternion(quaternion=(q.x, q.y, q.z, q.w))
		self.init_state[5] = euler[2]
		# linear velocity
		self.init_state[6] = msg_wheel.twist.twist.linear.x
		# angular velocity
		self.init_state[11] = msg_wheel.twist.twist.angular.z

		self.ready = True


	def run_filter(self):
		cmd_param = "rosparam load %s %s" % (self.config_file, self.filter_node_name)
		cmd_set = "rosparam set /%s/initial_state '%s'" % (self.filter_node_name, self.init_state)
		cmd_run = "rosrun robot_localization ekf_localization_node __name:=%s odometry/filtered:=%s" % (self.filter_node_name, self.topic_out)
		cmd_list = [cmd_param, cmd_set, cmd_run]
		print('initial state: ', self.init_state)
		for cmd in cmd_list:
			subprocess.call(cmd, shell=True)


if __name__ == "__main__":

	if len(sys.argv) < 4:
		print 'Please specify three topics to subscribe (wheel_odom, gps_odom, gps_heading)'
		sys.exit()

	topic_wheel_odom, topic_gps_odom, topic_gps_heading = sys.argv[1], sys.argv[2], sys.argv[3]

	rospy.init_node('global_filter_wrapper', anonymous=True)

	node_name = rospy.get_name()
	filter_node_name = rospy.get_param(node_name + '/filter_node_name')
	config_file = rospy.get_param(node_name + '/config_file')

	try:
		topic_out = rospy.get_param(node_name + '/topic_out')
	except:
		topic_out = 'odometry/filtered'

	global_filter_wrapper = GlobalFilterWrapper(topic_wheel_odom, topic_gps_odom, topic_gps_heading, config_file, filter_node_name, topic_out)
	rospy.spin()
		
		
