#!/usr/bin/env python

import sys
import rospy

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

from tf import transformations


class TrajPlotter(object):

	def __init__(self, topic, sensor='odom', relative=True):
	#sensor: 'odom', 'gps', 'imu'.
		self.topic = topic
		self.sensor = sensor
		self.relative = relative
		self.fig = plt.figure()
		self.traj_waypts = np.empty(shape=(0, 2)) # x, y
		
		if self.sensor == 'odom':
			self.sub = rospy.Subscriber(self.topic, Odometry, self.odom_callback)
		elif self.sensor == 'gps':
			self.sub = rospy.Subscriber(self.topic, NavSatFix, self.gps_callback)
		elif self.sensor == 'imu':
			self.sub = rospy.Subscriber(self.topic, Imu, self.imu_callback)
		else:
			rospy.logerr('Sensor type invalid !')


	def plot_traj(self):
		if self.sensor == 'odom' or self.sensor == 'imu':
			plt.xlabel('x')
			plt.ylabel('y')
		elif self.sensor == 'gps':
			plt.xlabel('longitude')
			plt.ylabel('latitude')
		plt.title(self.sensor)
		plt.axis('equal')
		plt.show()
	

	def odom_callback(self, msg_odom):
		print 'current timestamp: %f' % msg_odom.header.stamp.to_sec()
		pos = msg_odom.pose.pose.position
    	# save trajectory waypoints
		waypt_2d = np.array([pos.x, pos.y])
		self.traj_waypts = np.vstack((self.traj_waypts, waypt_2d))
		if self.traj_waypts.shape[0] == 1:
			rot_q = msg_odom.pose.pose.orientation
			rot_euler = transformations.euler_from_quaternion((rot_q.x, rot_q.y, rot_q.z, rot_q.w))
			self.yaw_init = rot_euler[2]
			self.rot_mat = np.array([[np.cos(self.yaw_init), -np.sin(self.yaw_init)], [np.sin(self.yaw_init), np.cos(self.yaw_init)]]).T
		else:
			traj_last = self.traj_waypts[-2:, :]
			if self.relative:
				traj_last = traj_last - self.traj_waypts[0, :]
				traj_last = np.matmul(traj_last, self.rot_mat.T)
			plt.plot(traj_last[:, 0], traj_last[:, 1], color='blue')


	def gps_callback(self, msg_gps):
		print 'current timestamp: %f' % msg_gps.header.stamp.to_sec()
		waypt_2d = np.array([msg_gps.longitude, msg_gps.latitude])
		self.traj_waypts = np.vstack((self.traj_waypts, waypt_2d))
		if self.traj_waypts.shape[0] > 1:
			traj_last = self.traj_waypts[-2:, :]
			plt.plot(traj_last[:, 0], traj_last[:, 1], color='blue')


	def imu_callback(self, msg_imu):
		time_current = msg_imu.header.stamp.to_sec()
		print 'current timestamp: %f' % time_current
		if self.traj_waypts.shape[0] == 0:
			self.time_last = time_current
			self.vel_last, self.ori_last = 0, 0
			waypt_2d = np.zeros(shape=(1, 2))
			self.traj_waypts = np.vstack((self.traj_waypts, waypt_2d))
		else:
			dt = time_current - self.time_last
			accel_lin = msg_imu.linear_acceleration
			vel_angular = msg_imu.angular_velocity
			vel_average = self.vel_last + accel_lin.x * dt / 2
			dist = vel_average * dt
			self.time_last = time_current
			self.vel_last += accel_lin.x * dt
			self.ori_last -= vel_angular.z * dt
			waypt_2d = self.traj_waypts[-1, :] + dist * np.array([np.cos(self.ori_last), np.sin(self.ori_last)])
			self.traj_waypts = np.vstack((self.traj_waypts, waypt_2d))

			traj_last = traj_last = self.traj_waypts[-2:, :]			
			plt.plot(traj_last[:, 0], traj_last[:, 1], color='blue')


if __name__ == "__main__":

	if len(sys.argv) < 3:
		print 'Please specify topic and associated sensor type'
		sys.exit()

	topic = sys.argv[1]
	sensor = sys.argv[2]

	rospy.init_node('plot_traj', anonymous=True)
	traj_plotter = TrajPlotter(topic, sensor)
	rospy.spin()

	if rospy.is_shutdown():
		traj_plotter.plot_traj()
