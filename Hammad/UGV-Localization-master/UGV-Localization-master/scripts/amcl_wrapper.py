#!/usr/bin/env python

import os, sys
import rospy
import rospkg
import subprocess

import yaml
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf


class AmclWrapper(object):

	# description: an external manager to load specific lidar map 
	# and run amcl localization when GPS fails for a long time
	def __init__(self, frequency=50, time_thresh=3):
		# frequency: running frequency for the wrapper
		# time_thresh: when GPS data fail for a period larger than time_thresh, launch amcl

		self.frequency = frequency
		self.time_thresh = time_thresh

		self.rate = rospy.Rate(self.frequency)

		self.sub_gps = rospy.Subscriber("/odometry/gps", Odometry, self.gps_callback)
		self.sub_filter = rospy.Subscriber("/odometry/filtered", Odometry, self.filter_callback)
		self.pub = rospy.Publisher("/amcl_initialpose", PoseWithCovarianceStamped, queue_size=10)

		self.amcl_running = False
		self.map_loaded = False

	def run(self):
		while not rospy.is_shutdown():
			time_curr = rospy.Time.now().to_sec()
			#print(time_curr)
			if not (hasattr(self, "gps_last_time") and hasattr(self, "last_loc")):
				continue

			if (time_curr - self.gps_last_time > self.time_thresh) and (not self.amcl_running):
				print("start to add AMCL lidar-based localization")
				if self.load_lidarmap():
					self.run_amcl()
					self.a_time = rospy.Time().now().to_sec()
			elif (time_curr - self.gps_last_time <= self.time_thresh) and self.amcl_running:
				self.stop_amcl()

			if self.map_loaded and (time_curr - self.a_time < 3):
				self.init_amcl_pose()

	def gps_callback(self, msg_gps):
		self.gps_last_time = msg_gps.header.stamp.to_sec()

	def filter_callback(self, msg_filter):
		self.last_loc = np.array([msg_filter.pose.pose.position.x, msg_filter.pose.pose.position.y]) # 2d location
		t = msg_filter.pose.pose.position
		q = msg_filter.pose.pose.orientation
		cov = msg_filter.pose.covariance
		rot_w2b = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		self.tf_w2b = np.vstack((np.hstack((rot_w2b[0:3, 0:3], np.array([[t.x], [t.y], [t.z]]))), np.array([[0, 0, 0, 1]]))) # world / map to base_link
		self.cov_w2b = [cov[0], cov[7], cov[35]] # x, y, a

	def load_lidarmap(self):
		if np.linalg.norm(self.last_loc - np.array([25, 300])) < 150:
			self.load_named_map("gilman")
			self.map_loaded = True
		#elif np.linalg.norm(self.last_loc - np.array([55, 45])) < 80:
		#	self.load_named_map("hackerman")
		#	self.map_loaded = True
		elif np.linalg.norm(self.last_loc - np.array([70, 125])) < 100:
			self.load_named_map("latrobe")
			self.map_loaded = True
		else:
			print("no associated maps available.")
			self.map_loaded = False
		return self.map_loaded

	def load_named_map(self, map_name):
		print("loading %s map" % map_name)
		#cmd = "roslaunch ugv_localization rampage_map_n_static_tf.launch lidarmap:=%s" % map_name
		#subprocess.Popen(cmd, shell=True)
		pack_path = rospkg.RosPack().get_path("ugv_localization")
		with open(os.path.join(pack_path, "map", "w2o_%s.yaml" % map_name), 'r') as yaml_file:
			param_world2og_org = yaml.load(yaml_file)
		rospy.set_param("world2og_org", param_world2og_org["world2og_org"])
		cmds = []
		cmds.append("rosparam set /map_og_original_server/frame_id og_org")
		map_path = os.path.join(pack_path, "map", "map_%s.yaml" % map_name)
		cmds.append("rosrun map_server map_server %s __name:=map_og_original_server" % map_path)
		cmds.append("rosrun tf2_ros static_transform_publisher world2og_org __name:=world_to_og_org")
		cmds.append("rosrun tf2_ros static_transform_publisher 0.45 0 0 0 0 0 1 base_link laser __name:=base_link2laser")
		for cmd in cmds:
			subprocess.Popen(cmd, shell=True)

		# retrieve transformation from world / map to og_org
		tf_w2o_dict = rospy.get_param("world2og_org/transform")
		t = tf_w2o_dict['translation']
		q = tf_w2o_dict['rotation']
		rot_w2o = tf.transformations.quaternion_matrix([q['x'], q['y'], q['z'], q['w']])
		tf_w2o = np.vstack((np.hstack((rot_w2o[0:3, 0:3], np.array([[t['x']], [t['y']], [t['z']]]))), np.array([[0, 0, 0, 1]]))) # world / map to base_link
		self.tf_o2w = tf.transformations.inverse_matrix(tf_w2o)

	def run_amcl(self):
		print("running amcl localizer ...")
		#self.init_amcl_pose()
		#cmd = "roslaunch ugv_localization rampage_amcl_diff.launch"
		#subprocess.Popen(cmd, shell=True)
		pack_path = rospkg.RosPack().get_path("ugv_localization")
		amcl_params_path = os.path.join(pack_path, "params/rampage_amcl_diff.yaml")
		cmds = []
		cmds.append("rosparam load %s amcl" % amcl_params_path)
		cmds.append("rosrun amcl amcl scan:=scan initialpose:=amcl_initialpose __name:=amcl")
		for cmd in cmds:
			subprocess.Popen(cmd, shell=True)
		self.amcl_running = True

	def init_amcl_pose(self):
		#listener = tf.TransformListener()
		#listener.waitForTransform("world", "og_org", rospy.Time().now(), rospy.Duration(1))
		#(o2b_trans, o2b_rot) = listener.lookupTransform("og_org", "base_link", rospy.Time(0))
		#o2b_a = tf.transformations.euler_from_quaternion(o2b_rot)[2]
		if hasattr(self, "tf_o2w") and hasattr(self, "tf_w2b"):
			tf_o2b = np.matmul(self.tf_o2w, self.tf_w2b)
			trans_o2b = tf.transformations.translation_from_matrix(tf_o2b)
			rot_o2b = tf.transformations.euler_from_matrix(tf_o2b)
			q_o2b = tf.transformations.quaternion_from_matrix(tf_o2b)
		else:
			rospy.logerr("Required tfs cannot be retrieved for amcl pose initialization.")
		
		#rospy.set_param("amcl/initial_pose_x", float(trans_o2b[0]))
		#rospy.set_param("amcl/initial_pose_y", float(trans_o2b[1]))
		#rospy.set_param("amcl/initial_pose_a", float(rot_o2b[2]))

		msg = PoseWithCovarianceStamped()
		msg.header.stamp = rospy.Time().now()
		msg.header.frame_id = "og_org"
		msg.pose.pose.position.x = trans_o2b[0]
		msg.pose.pose.position.y = trans_o2b[1]
		msg.pose.pose.position.z = trans_o2b[2]
		msg.pose.pose.orientation.x = q_o2b[0]
		msg.pose.pose.orientation.y = q_o2b[1]
		msg.pose.pose.orientation.z = q_o2b[2]
		msg.pose.pose.orientation.w = q_o2b[3]
		msg.pose.covariance[0] = 1
		msg.pose.covariance[7] = 1
		msg.pose.covariance[35] = 0.03
		self.pub.publish(msg)
		
	def stop_amcl(self):
		cmds = []
		cmds.append("rosnode kill amcl")
		cmds.append("rosnode kill map_og_original_server")
		cmds.append("rosnode kill world_to_og_org")
		for cmd in cmds:
			subprocess.call(cmd, shell=True)
		self.amcl_running = False
		print("stop running amcl.")


if __name__ == "__main__":

	rospy.init_node('amcl_wrapper', anonymous=True)

	amcl_wrapper = AmclWrapper()
	amcl_wrapper.run()

	rospy.spin()



