#!/usr/bin/env python

import rospy
import numpy as np

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


class WaypointsMarkerManager(object):

	def __init__(self):
		self.server = InteractiveMarkerServer("waypoints_marker")
		self.waypoints = np.empty((0, 2))


	def reset(self):
		self.server.clear()
		self.server.applyChanges()
		self.waypoints = np.empty((0, 2))


	def add_waypoints(self, point):
		# point: numpy array of size (2,)
		self.waypoints = np.vstack((self.waypoints, point.reshape(1, 2)))

		# add new interactive marker
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "map"
		int_marker.name = "marker_%d" % self.waypoints.shape[0]
		int_marker.description = "marker with 2-DOF control (x-y plane)"
		int_marker.pose.position.x = point[0]
		int_marker.pose.position.y = point[1]
		int_marker.pose.position.z = 0

		box_marker = Marker()
		box_marker.type = Marker.CUBE
		box_marker.scale.x = 1
		box_marker.scale.y = 1
		box_marker.scale.z = 1
		box_marker.color.r = 0.0
		box_marker.color.g = 0.5
		box_marker.color.b = 0.5
		box_marker.color.a = 1.0

		box_control = InteractiveMarkerControl() # button control to start trajectory refinement
		box_control.name = "start_refine"
		box_control.interaction_mode = InteractiveMarkerControl.BUTTON
		box_control.always_visible = True
		box_control.markers.append(box_marker)
		int_marker.controls.append(box_control)

		movex_control = InteractiveMarkerControl()
		movex_control.name = "move_x"
		movex_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		int_marker.controls.append(movex_control)
		movey_control = InteractiveMarkerControl()
		movey_control.name = "move_y"
		movey_control.orientation.w = 1
		movey_control.orientation.x = 0
		movey_control.orientation.y = 0
		movey_control.orientation.z = 1
		movey_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		int_marker.controls.append(movey_control)

		self.server.insert(int_marker, self.update_waypoints)
		self.server.applyChanges()


	def update_waypoints(self, feedback):
		wp_idx = int(feedback.marker_name.split("_")[1]) - 1 # marker index for update
		self.waypoints[wp_idx, 0] = feedback.pose.position.x
		self.waypoints[wp_idx, 1] = feedback.pose.position.y


	def get_waypoints(self):
		return self.waypoints


