#!/usr/bin/env python

import sys
import rospy

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class GPSCovHandler(object):
	# re-scale or crop the GPS measurement covariance. 

	def __init__(self, topic_in, topic_out, scale=1, cov_threshold=1, msr='position'):
		"""
		msr: 'position' (message type is Odometry) or 'heading' (message type is Imu)
		"""
		# configure subscriber and publisher
		if msr == 'position':
			self.sub = rospy.Subscriber(topic_in, Odometry, self.callback)
			self.pub = rospy.Publisher(topic_out, Odometry, queue_size=10)
		elif msr == 'heading':
			self.sub = rospy.Subscriber(topic_in, Imu, self.callback)
			self.pub = rospy.Publisher(topic_out, Imu, queue_size=10)
		else:
			raise ValueError('msr attribute has to be "position" or "heading"!')
		
		self.scale = scale
		self.cov_threshold = cov_threshold
		self.msr = msr


	def callback(self, msg_in):
		if self.msr == 'position':
			if (msg_in.position_covariance[0] > self.cov_threshold) and (msg_in.position_covariance[4] > self.cov_threshold):
				return
			msg_out = Odometry()
			msg_out.header = msg_in.header
			msg_out.pose.pose = msg_in.pose.pose
			for i in range(len(msg_in.pose.covariance)):
				msg_out.pose.covariance[i] = self.scale * msg_in.pose.covariance[i]

		elif self.msr == 'heading':
			if (msg_in.orientation_covariance[8] > self.cov_threshold):
				return
			msg_out = Imu()
			msg_out.header = msg_in.header
			msg_out.orientation = msg_in.orientation
			for i in range(len(msg_in.orientation_covariance)):
				msg_out.orientation_covariance[i] = self.scale * msg_in.orientation_covariance[i]
			msg_out.angular_velocity_covariance[0] = -1
			msg_out.linear_acceleration_covariance[0] = -1

		self.pub.publish(msg_out)


if __name__ == "__main__":

	rospy.init_node('gps_cov_handler', anonymous=True)

	node_name = rospy.get_name()
	msr = rospy.get_param(node_name + '/msr', 'heading')
	topic_in = rospy.get_param(node_name + '/topic_in', '/gps/navheading')
	topic_out = rospy.get_param(node_name + '/topic_out', '/gps/navheading/scaled')
	scale = rospy.get_param(node_name + '/scale', 1000)
	cov_threshold = rospy.get_param(node_name + '/cov_thresh', 1000)

	gps_cov_handler = GPSCovHandler(topic_in, topic_out, scale, cov_threshold, msr)
	rospy.spin()

	
