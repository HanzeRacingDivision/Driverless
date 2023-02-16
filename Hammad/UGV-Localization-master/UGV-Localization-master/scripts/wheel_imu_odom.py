#!/usr/bin/env python

import sys
import rospy

import numpy as np

from rampage_msgs.msg import WheelOdometry
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf import transformations


class WheelImuOdom(object):
	# use wheel odometry for linear velocity and imu for angular velocity or yaw.

	def __init__(self, topic_wheel, topic_imu, topic_odom, mode='local', offset=None):
		"""
		mode: 'global': use imu absolute yaw, get global odometry; 'local': simply use imu continuous measurment (vel, accel), get local odometry.
		offset: x-y position offset. has to specify if mode is 'global'
		"""
		# configure subscriber and publisher
		self.sub_wheel = rospy.Subscriber(topic_wheel, WheelOdometry, self.wheel_callback)
		self.sub_imu = rospy.Subscriber(topic_imu, Imu, self.imu_callback)
		self.pub_odom = rospy.Publisher(topic_odom, Odometry, queue_size=10)

		self.mode = mode
		self.offset = offset
		if self.mode == 'global' and self.offset is None:
			raise ValueError("Have to specify offset for 'global' mode!") 

		# define motion params (linear velocity, angular velocity, x, y, orientation)
		self.vel_lin = 0
		self.vel_ang = 0
		if self.mode == 'global':
			self.x, self.y = self.offset[0], self.offset[1]
		elif self.mode == 'local':
			self.x, self.y = 0, 0
		self.ori = 0 # 2D
		self.time = -1
		self.seq_pub = 0


	def set_init_state(self, x_init, y_init, yaw_init):
		self.set_init_loc(x_init, y_init)
		self.set_init_ori(yaw_init)


	def set_init_loc(self, x_init, y_init):
		self.x, self.y = x_init, y_init


	def set_init_ori(self, yaw_init):
		self.ori = yaw_init


	def wheel_callback(self, msg_wheel):
		time_curr = msg_wheel.t_epoch.to_sec()
		dt_v = msg_wheel.delta_micros * 1e-6 # just used for linear vel
		dt = time_curr - self.time

		# update
		self.vel_lin = self.rpm_to_vel(msg_wheel.delta_ticks)
		dist = self.vel_lin * dt_v
		if self.mode == 'local':
			self.ori += self.vel_ang * dt
		self.x += dist * np.cos(self.ori)
		self.y += dist * np.sin(self.ori)
		self.time = time_curr

		# publish odom
		msg_odom = Odometry()
		self.seq_pub += 1
		msg_odom.child_frame_id = "base_link"
		msg_odom.header.frame_id = "odom" if self.mode == 'local' else "world"
		msg_odom.header.seq = self.seq_pub
		msg_odom.header.stamp = msg_wheel.t_epoch
		msg_odom.twist.twist.angular.x = 0
		msg_odom.twist.twist.angular.y = 0
		msg_odom.twist.twist.angular.z = self.vel_ang
		msg_odom.twist.twist.linear.x = self.vel_lin
		msg_odom.twist.twist.linear.y = 0
		msg_odom.twist.twist.linear.z = 0

		q = transformations.quaternion_from_euler(0, 0, self.ori)
		msg_odom.pose.pose.orientation.x = q[0]
		msg_odom.pose.pose.orientation.y = q[1]
		msg_odom.pose.pose.orientation.z = q[2]
		msg_odom.pose.pose.orientation.w = q[3]

		msg_odom.pose.pose.position.x = self.x
		msg_odom.pose.pose.position.y = self.y
		msg_odom.pose.pose.position.z = 0

		self.pub_odom.publish(msg_odom)


	def imu_callback(self, msg_imu):
		time_curr = msg_imu.header.stamp.to_sec()
		dt = time_curr - self.time

		# update
		self.vel_ang = msg_imu.angular_velocity.z
		
		if self.mode == 'global':
			q = msg_imu.orientation
			euler = transformations.euler_from_quaternion(quaternion=(q.x, q.y, q.z, q.w))
			self.ori = euler[2]
		elif self.mode == 'local':
			if self.time != -1:
				self.ori += self.vel_ang * dt
		
		self.time = time_curr


	def rpm_to_vel(self, rpm):
		return 0.0006046 * rpm


if __name__ == "__main__":

	if len(sys.argv) < 4:
		print 'Please specify two topics to subscribe (wheel, imu) and one topic to publish (odom)'
		sys.exit()

	topic_wheel, topic_imu, topic_odom = sys.argv[1], sys.argv[2], sys.argv[3]

	rospy.init_node('wheel_imu_odom', anonymous=True)
	wheel_imu_odom = WheelImuOdom(topic_wheel, topic_imu, topic_odom, mode='local')
	rospy.spin()

	
