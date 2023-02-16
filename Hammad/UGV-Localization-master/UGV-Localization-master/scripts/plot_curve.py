#!/usr/bin/env python

import os, sys
import rospy
import rospkg

import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from ublox_msgs.msg import NavRELPOSNED9

from tf import transformations
from tf2_msgs.msg import TFMessage


class CurvePlotter(object):

	def __init__(self, mode='global'):
	# hardcoded topics.
		self.mode = mode

		self.start_time = -1

		# records
		self.record_nofilter = np.empty(shape=(0, 4)) # t, x, y, yaw
		self.record_filter = np.empty(shape=(0, 7)) # t, x, y, yaw, x_cov, y_cov, yaw_cov
		self.slip_angle = [] # slip angle for filtered odometry

		self.gps_msr = np.empty(shape=(0, 5)) # t, lat, lat_cov, lon, lon_cov
		self.heading_msr = np.empty(shape=(0, 3)) # t, yaw, yaw_cov
		self.navrelposned = np.empty(shape=(0, 3)) # t, relPoseLength, accHeading

		self.imu_3dm = np.empty(shape=(0, 5)) # t, ang_vel.z, lin_accel.x, lin_accel.y, lin_accel.z
		self.t265_ang_vel = np.empty(shape=(0, 2)) # t, ang_vel.z
		self.t265_lin_accel = np.empty(shape=(0, 4)) # t, lin_accel.x, lin_accel.y, lin_accel.z

		self.gps_odom_record = np.empty((0, 3)) # t, x, x_cov
		
		# subscribers
		self.sub_nofilter = rospy.Subscriber('/odometry', Odometry, self.odom_callback)

		if self.mode == 'local':
			self.sub_tf = rospy.Subscriber('/tf_static', TFMessage, self.tf_callback)

		self.sub_filter = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

		self.sub_odom = rospy.Subscriber('/odometry/wheel_imu', Odometry, self.odom_callback)

		self.sub_gps = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
		self.sub_gps_odom = rospy.Subscriber('/odometry/gps', Odometry, self.gps_odom_callback)
		self.sub_heading = rospy.Subscriber('/gps/navheading/new', Imu, self.heading_callback)
		self.sub_navrelposned = rospy.Subscriber('/gps/navrelposned', NavRELPOSNED9, self.navrelposned_callback)

		self.sub_imu_3dm = rospy.Subscriber('/imu/refined', Imu, self.imu_callback)
		self.sub_t265_ang_vel = rospy.Subscriber('/rs_t265/gyro/sample', Imu, self.imu_callback)
		self.sub_t265_lin_accel = rospy.Subscriber('/rs_t265/accel/sample', Imu, self.imu_callback)

		self.tf_utm2map, self.tf_odom2utm, self.tf_map2odom = None, None, None


	def plot_curve_pose(self, path):
		fig, axs = plt.subplots(3)
		for i in range(3): # x, y, yaw
			axs[i].plot(self.record_nofilter[:, 0], self.record_nofilter[:, i+1], color='green', label='no filter')
			axs[i].plot(self.record_filter[:, 0], self.record_filter[:, i+1], color='red', label='filter')
		axs[0].set_ylabel('x')
		axs[1].set_ylabel('y')
		axs[2].set_ylabel('yaw')
		axs[2].set_xlabel('time')
		axs[0].legend()
		plt.savefig(path)
		plt.show()

	
	def plot_curve_slipangle(self, path):
		fig, axs = plt.subplots(2)

		# plot filtered yaw
		axs[0].plot(self.record_filter[:, 0], self.record_filter[:, 3])
		axs[0].set_ylabel('yaw')

		# plot slip angle
		axs[1].plot(self.record_filter[:, 0], self.slip_angle)
		axs[1].set_ylabel('slip angle')
		axs[1].set_xlabel('time')

		plt.savefig(path)
		plt.show()


	def plot_covariance_gps(self, path):
		fig, axs = plt.subplots(2, 2)

		# plot lat with covariance
		axs[0, 0].plot(self.gps_msr[:, 0], self.gps_msr[:, 1])
		axs[0, 0].set_ylabel('latitude')
		axs[1, 0].plot(self.gps_msr[:, 0], self.gps_msr[:, 2])
		axs[1, 0].set_ylabel('latitude covariance')
		axs[1, 0].set_xlabel('time')

		# plot lon with covariance
		axs[0, 1].plot(self.gps_msr[:, 0], self.gps_msr[:, 3])
		axs[0, 1].set_ylabel('longitude')
		axs[1, 1].plot(self.gps_msr[:, 0], self.gps_msr[:, 4])
		axs[1, 1].set_ylabel('longitude covariance')
		axs[1, 1].set_xlabel('time')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def plot_covariance_heading(self, path):
		fig, axs = plt.subplots(2)

		# plot heading measurements with covariance
		axs[0].plot(self.heading_msr[:, 0], self.heading_msr[:, 1])
		axs[0].set_ylabel('heading measurement')
		axs[1].plot(self.heading_msr[:, 0], self.heading_msr[:, 2])
		axs[1].set_ylabel('measurement covariance')
		axs[1].set_xlabel('time')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def plot_heading_clue(self, path):
		fig, axs = plt.subplots(4)

		# plot heading measurements with covariance
		axs[0].plot(self.heading_msr[:, 0], self.heading_msr[:, 1])
		axs[0].set_ylabel('GPS heading')
		axs[1].plot(self.heading_msr[:, 0], self.heading_msr[:, 2])
		axs[1].set_ylabel('msr cov')

		# plot navrelposned
		axs[2].plot(self.navrelposned[:, 0], self.navrelposned[:, 1])
		axs[2].set_ylabel('relPosLen')
		axs[3].plot(self.navrelposned[:, 0], self.navrelposned[:, 2])
		axs[3].set_ylabel('accHeading')
		axs[3].set_xlabel('time')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def plot_imu_continuous(self, path, mode='separate', t_min=0):
		imu_3dm = self.imu_3dm[self.imu_3dm[:, 0] >= t_min, :]
		t265_ang_vel = self.t265_ang_vel[self.t265_ang_vel[:, 0] >= t_min, :]
		t265_lin_accel = self.t265_lin_accel[self.t265_lin_accel[:, 0] >= t_min, :]

		if mode == 'separate':
			fig, axs = plt.subplots(4, 2)

			# plot angular velocity (Z-axis)
			axs[0, 0].plot(imu_3dm[:, 0], imu_3dm[:, 1], color='green', label='imu_3dm')
			axs[0, 1].plot(t265_ang_vel[:, 0], t265_ang_vel[:, 1], color='red', label='t265')
			axs[0, 0].set_ylabel('ang vel (yaw)')
			min_y = min(np.min(imu_3dm[:, 1]), np.min(t265_ang_vel[:, 1]))
			max_y = max(np.max(imu_3dm[:, 1]), np.max(t265_ang_vel[:, 1]))
			axs[0, 0].set_ylim(min_y, max_y)
			axs[0, 1].set_ylim(min_y, max_y)

			# plot linear acceleration (X-axis)
			axs[1, 0].plot(imu_3dm[:, 0], imu_3dm[:, 2], color='green', label='imu_3dm')		
			axs[1, 1].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 1], color='red', label='t265')
			axs[1, 0].set_ylabel('lin accel (X)')
			min_y = min(np.min(imu_3dm[:, 2]), np.min(t265_lin_accel[:, 1]))
			max_y = max(np.max(imu_3dm[:, 2]), np.max(t265_lin_accel[:, 1]))
			axs[1, 0].set_ylim(min_y, max_y)
			axs[1, 1].set_ylim(min_y, max_y)

			# plot linear acceleration (Y-axis)
			axs[2, 0].plot(imu_3dm[:, 0], imu_3dm[:, 3], color='green', label='imu_3dm')
			axs[2, 1].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 2], color='red', label='t265')
			axs[2, 0].set_ylabel('lin accel (Y)')
			min_y = min(np.min(imu_3dm[:, 3]), np.min(t265_lin_accel[:, 2]))
			max_y = max(np.max(imu_3dm[:, 3]), np.max(t265_lin_accel[:, 2]))
			axs[2, 0].set_ylim(min_y, max_y)
			axs[2, 1].set_ylim(min_y, max_y)

			# plot linear acceleration (Z-axis)
			axs[3, 0].plot(imu_3dm[:, 0], imu_3dm[:, 4], color='green', label='imu_3dm')
			axs[3, 1].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 3], color='red', label='t265')
			axs[3, 0].set_ylabel('lin accel (Z)')
			min_y = min(np.min(imu_3dm[:, 4]), np.min(t265_lin_accel[:, 3]))
			max_y = max(np.max(imu_3dm[:, 4]), np.max(t265_lin_accel[:, 3]))
			axs[3, 0].set_ylim(min_y, max_y)
			axs[3, 1].set_ylim(min_y, max_y)

			axs[3, 0].set_xlabel('time')
			axs[3, 1].set_xlabel('time')

			axs[0, 0].legend()
			axs[0, 1].legend()

		elif mode == 'single':
			fig, axs = plt.subplots(4)

			# plot angular velocity (Z-axis)
			axs[0].plot(imu_3dm[:, 0], imu_3dm[:, 1], color='green', label='imu_3dm')
			axs[0].plot(t265_ang_vel[:, 0], t265_ang_vel[:, 1], color='red', label='t265')
			axs[0].set_ylabel('ang vel (yaw)')

			# plot linear acceleration (X-axis)
			axs[1].plot(imu_3dm[:, 0], imu_3dm[:, 2], color='green', label='imu_3dm')		
			axs[1].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 1], color='red', label='t265')
			axs[1].set_ylabel('lin accel (X)')

			# plot linear acceleration (Y-axis)
			axs[2].plot(imu_3dm[:, 0], imu_3dm[:, 3], color='green', label='imu_3dm')
			axs[2].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 2], color='red', label='t265')
			axs[2].set_ylabel('lin accel (Y)')

			# plot linear acceleration (Z-axis)
			axs[3].plot(imu_3dm[:, 0], imu_3dm[:, 4], color='green', label='imu_3dm')
			axs[3].plot(t265_lin_accel[:, 0], t265_lin_accel[:, 3], color='red', label='t265')
			axs[3].set_ylabel('lin accel (Z)')

			axs[3].set_xlabel('time')

			axs[0].legend()

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def plot_covariance_odom(self, path):
		# plot pose covariance of filtered odometry
		fig, axs = plt.subplots(2, 3)
		axs[0, 0].plot(self.record_filter[:, 0], self.record_filter[:, 1])
		axs[0, 0].set_ylabel('x')
		axs[1, 0].plot(self.record_filter[:, 0], self.record_filter[:, 4])
		axs[1, 0].set_ylabel('x cov')

		axs[0, 1].plot(self.record_filter[:, 0], self.record_filter[:, 2])
		axs[0, 1].set_ylabel('y')
		axs[1, 1].plot(self.record_filter[:, 0], self.record_filter[:, 5])
		axs[1, 1].set_ylabel('y cov')

		axs[0, 2].plot(self.record_filter[:, 0], self.record_filter[:, 3])
		axs[0, 2].set_ylabel('yaw')
		axs[1, 2].plot(self.record_filter[:, 0], self.record_filter[:, 6])
		axs[1, 2].set_ylabel('yaw cov')

		plt.tight_layout()
		plt.savefig(path)
		plt.show()


	def odom_callback(self, msg_odom):
		time_curr = msg_odom.header.stamp.to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		print(rospy.Time.now().to_sec() - time_curr)
		# print 'current timestamp: %f' % time_curr
		dt = time_curr - self.start_time
		p = msg_odom.pose.pose.position # position
		q = msg_odom.pose.pose.orientation # orientation (quaternion)
		euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # orientation (euler)
		cov = msg_odom.pose.covariance # covariance

		if msg_odom.header.frame_id == '/world':
			yaw = euler[2] % (2 * np.pi) # restrict between [0, 2*PI]
			new_record = np.array([[dt, p.x, p.y, yaw]])
			self.record_nofilter = np.vstack((self.record_nofilter, new_record))

		elif msg_odom.header.frame_id == 'odom' or msg_odom.header.frame_id == 'map':
			if self.mode == 'local':
				trans_odom2baselink = transformations.translation_matrix([p.x, p.y, p.z])
				rot_odom2baselink = transformations.euler_matrix(euler[0], euler[1], euler[2])
				tf_odom2baselink = np.matmul(trans_odom2baselink, rot_odom2baselink)
				if self.tf_map2odom is not None:
					tf_map2baselink = np.matmul(self.tf_map2odom, tf_odom2baselink)
					x, y, _ = transformations.translation_from_matrix(tf_map2baselink)
					_, _, yaw = transformations.euler_from_matrix(tf_map2baselink)
					yaw = yaw % (2 * np.pi) # restrict between [0, 2*PI]
					new_record = np.array([[dt, x, y, yaw]])
					self.record_filter = np.vstack((self.record_filter, new_record))
			
			elif self.mode == 'global':
				yaw = euler[2] % (2 * np.pi)
				new_record = np.array([[dt, p.x, p.y, yaw, cov[0], cov[7], cov[35]]])
				self.record_filter = np.vstack((self.record_filter, new_record))

			# slip angle
			vel_lin = msg_odom.twist.twist.linear # linear velocity
			slip_angle = - np.arctan2(vel_lin.y, np.abs(vel_lin.x))
			self.slip_angle.append(slip_angle)
		

	def tf_callback(self, msg_tfs):
		msg_tf = msg_tfs.transforms[0]
		p = msg_tf.transform.translation
		q = msg_tf.transform.rotation
		euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		if (msg_tf.header.frame_id == 'utm') and (msg_tf.child_frame_id == 'map') and (self.tf_utm2map is None):
			trans_utm2map = transformations.translation_matrix([p.x, p.y, p.z])
			rot_utm2map = transformations.euler_matrix(euler[0], euler[1], euler[2])
			self.tf_utm2map = np.matmul(trans_utm2map, rot_utm2map)
		elif (msg_tf.header.frame_id == 'odom') and (msg_tf.child_frame_id == 'utm') and (self.tf_odom2utm is None):
			trans_odom2utm = transformations.translation_matrix([p.x, p.y, p.z])
			rot_odom2utm = transformations.euler_matrix(euler[0], euler[1], euler[2])
			self.tf_odom2utm = np.matmul(trans_odom2utm, rot_odom2utm)

		if (self.tf_utm2map is not None) and (self.tf_odom2utm is not None):
			tf_odom2map = np.matmul(self.tf_odom2utm, self.tf_utm2map)
			self.tf_map2odom = transformations.inverse_matrix(tf_odom2map)
			self.sub_tf.unregister()


	def gps_callback(self, msg_gps):
		time_curr = msg_gps.header.stamp.to_sec()
		#print(rospy.Time.now().to_sec() - time_curr)
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		#if self.gps_msr.shape[0] > 0:
		#	diff = np.abs(msg_gps.longitude - self.gps_msr[-1, 3])
		#	print(diff)
		#	print('cov: ',  msg_gps.position_covariance[4])
		new_record = np.array([[dt, msg_gps.latitude, msg_gps.position_covariance[0], msg_gps.longitude, msg_gps.position_covariance[4]]])
		self.gps_msr = np.vstack((self.gps_msr, new_record))


	def gps_odom_callback(self, msg_gps):
		time_curr = msg_gps.header.stamp.to_sec()
		#print(rospy.Time.now().to_sec() - time_curr)
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		#if self.gps_odom_record.shape[0] > 0:
		#	diff = np.abs(msg_gps.pose.pose.position.x - self.gps_odom_record[-1, 1])
		#	print(diff)
		#	print('cov: ',  msg_gps.pose.covariance[0])
		new_record = np.array([[dt, msg_gps.pose.pose.position.x, msg_gps.pose.covariance[0]]])
		self.gps_odom_record = np.vstack((self.gps_odom_record, new_record))


	def heading_callback(self, msg_heading):
		#if msg_heading.orientation_covariance[8] >= 1000:
		#	return
		
		time_curr = msg_heading.header.stamp.to_sec()
		# print(rospy.Time.now().to_sec() - time_curr)
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		
		q = msg_heading.orientation
		_, _, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
		yaw = yaw % (2 * np.pi)
		#if self.heading_msr.shape[0] > 0:
		#	diff = np.abs(yaw - self.heading_msr[-1, 1])
		#	print(diff)
		#	print('cov: ',  msg_heading.orientation_covariance[8])
		new_record = np.array([[dt, yaw, msg_heading.orientation_covariance[8]]])
		self.heading_msr = np.vstack((self.heading_msr, new_record))


	def navrelposned_callback(self, msg_navrelposned):
		time_curr = rospy.Time.now().to_sec()
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time
		rel_len = msg_navrelposned.relPosLength + 0.01 * msg_navrelposned.relPosHPLength
		new_record = np.array([[dt, rel_len, msg_navrelposned.accHeading]])
		self.navrelposned = np.vstack((self.navrelposned, new_record))


	def imu_callback(self, msg_imu):
		time_curr = rospy.Time.now().to_sec()
		# print(rospy.Time.now().to_sec() - time_curr)
		if self.start_time == -1:
			self.start_time = time_curr
		dt = time_curr - self.start_time

		if msg_imu.header.frame_id == 'base_link':
			new_record = np.array([[dt, msg_imu.angular_velocity.z, msg_imu.linear_acceleration.x, msg_imu.linear_acceleration.y, msg_imu.linear_acceleration.z]])
			self.imu_3dm = np.vstack((self.imu_3dm, new_record))

		# T265 IMU, X(left)-Y(up)-Z(forward) 
		elif msg_imu.header.frame_id == 'rs_t265_gyro_optical_frame':
			new_record = np.array([[dt, msg_imu.angular_velocity.y]])
			self.t265_ang_vel = np.vstack((self.t265_ang_vel, new_record))
		elif msg_imu.header.frame_id == 'rs_t265_accel_optical_frame':
			new_record = np.array([[dt, msg_imu.linear_acceleration.z, msg_imu.linear_acceleration.x, msg_imu.linear_acceleration.y]])
			self.t265_lin_accel = np.vstack((self.t265_lin_accel, new_record))

	def save_results(self):
		path_filter = '/home/charlierkj/asco/src/ugv_localization/records/test_03_filter_(no_gps).npy'
		path_heading = '/home/charlierkj/asco/src/ugv_localization/records/test_03_heading_msr_(no_gps).npy'
		np.save(path_filter, self.record_filter)
		np.save(path_heading, self.heading_msr)

		

if __name__ == "__main__":

	rospy.init_node('plot_curve', anonymous=True)

	node_name = rospy.get_name()
	mode = rospy.get_param(node_name + '/mode', 'global')
	data = rospy.get_param(node_name + '/name', 'test')

	curve_plotter = CurvePlotter(mode)
	rospy.spin()

	pkg_path = rospkg.RosPack().get_path("ugv_localization")
	if not os.path.exists(os.path.join(pkg_path, 'figs')):
		os.mkdir(os.path.join(pkg_path, 'figs'))

	path_pose = os.path.join(pkg_path, 'figs/%s_pose.png' % data)
	path_slipangle = os.path.join(pkg_path, 'figs/%s_slipangle.png' % data)
	path_gps = os.path.join(pkg_path, 'figs/%s_gps_msr.png' % data)
	path_heading = os.path.join(pkg_path, 'figs/%s_heading_msr.png' % data)
	path_imu_continuous = os.path.join(pkg_path, 'figs/%s_imu_vs_t265.png' % data)
	path_cov_odom = os.path.join(pkg_path, 'figs/%s_cov_odom.png' % data)
	path_heading_clue = os.path.join(pkg_path, 'figs/%s_heading_clue_2.png' % data)

	if rospy.is_shutdown():
		curve_plotter.plot_curve_pose(path_pose)
		curve_plotter.plot_curve_slipangle(path_slipangle)
		curve_plotter.plot_covariance_gps(path_gps)
		curve_plotter.plot_covariance_heading(path_heading)
		curve_plotter.plot_imu_continuous(path_imu_continuous, mode='separate', t_min=50)
		curve_plotter.plot_covariance_odom(path_cov_odom)
		curve_plotter.save_results()
		#curve_plotter.plot_heading_clue(path_heading_clue)
