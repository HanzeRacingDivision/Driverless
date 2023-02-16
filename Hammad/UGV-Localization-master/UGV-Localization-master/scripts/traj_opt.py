#!/usr/bin/env python

import os, sys
import rospy
import rosbag
import rospkg

import numpy as np
import matplotlib
#matplotlib.use('TkAgg')
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import yaml

import scipy.optimize
from cvxopt import solvers
from cvxopt import matrix

from rampage_msgs.msg import RefTrajAxyt

MAGIC_FABIAN_CONST = 6.5


class TrajOptimizer(object):

	def __init__(self, topic_out, vel=1, accel=1, loop=False):
		self.polys_x = None
		self.polys_y = None
		self.time = None

		self.vel = vel
		self.accel = accel
		self.loop = loop

		self.topic_out = topic_out
		self.pub = rospy.Publisher(topic_out, RefTrajAxyt, queue_size=10)


	def set_wps(self, wps):
		self.waypoints = wps
		self.num_segments = self.waypoints.shape[0] - 1
		if self.loop:
			self.waypoints = np.vstack((self.waypoints, self.waypoints[0, :].reshape(1, 2)))
			self.num_segments += 1
		self.durations = np.zeros(shape=(self.num_segments,))
		self.time = np.sum(self.durations)


	def load_npy(self, path):
		wps = np.load(path)
		self.set_wps(wps)


	def estimate_segment_times(self):
		v_max = self.vel
		a_max = self.accel
		for seg_i in range(self.num_segments):
			start = self.waypoints[seg_i, :]
			end = self.waypoints[seg_i + 1, :]
			distance = np.linalg.norm(end - start)
			t = distance / v_max * 2 * (1.0 + MAGIC_FABIAN_CONST * v_max / a_max * np.exp(- distance / v_max * 2))
			self.durations[seg_i] = t
		self.time = np.sum(self.durations)
		print(self.durations)


	def refine_segment_times(self):
		v_appr = self.vel
		a_appr = self.accel
		for seg_i in range(self.num_segments):
			dist = 0
			for t in np.arange(0, self.durations[seg_i], 0.01):
				poly_x = self.polys_x[seg_i * 8 : (seg_i + 1) * 8]
				poly_y = self.polys_y[seg_i * 8 : (seg_i + 1) * 8]
				poly_velx = np.polyder(poly_x, 1)
				poly_vely = np.polyder(poly_y, 1)
				vel_x = np.polyval(poly_velx, t)
				vel_y = np.polyval(poly_vely, t)
				vel = np.linalg.norm([vel_x, vel_y])
				dist += vel * 0.01
			if (seg_i == 0) or (seg_i == (self.num_segments - 1)):
				self.durations[seg_i] = (dist + v_appr ** 2 / (2 * a_appr)) / v_appr
			else:
				self.durations[seg_i] = dist / v_appr
		self.time = np.sum(self.durations)
		print(self.durations)


	def objective_func(self, polys):
		# minimize integral of snap squared
		cost = 0
		for seg_i in range(self.num_segments):
			poly = polys[seg_i * 8 : (seg_i + 1) * 8]
			der = np.polyder(poly, 4)
			for t in np.arange(0, self.durations[seg_i], 0.0001):
				cost_i = np.polyval(der, t) ** 2 * 0.0001
				cost += cost_i
		return cost


	def constraint_position_value(self, polys, axis, seg_i, t, desired_value):
		poly = polys[seg_i * 8 : (seg_i + 1) * 8]
		value = np.polyval(poly, t)
		return value - desired_value


	def constraint_position_continuity(self, polys, seg_i, a):
		poly_last = polys[(seg_i - 1) * 8 : seg_i * 8]
		poly_this = polys[seg_i * 8 : (seg_i + 1) * 8]
		value_last = np.polyval(poly_last, self.durations[seg_i - 1])
		value_this = np.polyval(poly_this, 0)
		return value_this - value_last


	def constraint_derivative_value(self, polys, seg_i, t, desired_value):
		poly = polys[seg_i * 8 : (seg_i + 1) * 8]
		der = np.polyder(poly, 1)
		value = np.polyval(der, t)
		return value - desired_value


	def constraint_derivative_continuity(self, polys, seg_i):
		poly_last = polys[(seg_i - 1) * 8 : seg_i * 8]
		poly_this = polys[seg_i * 8 : (seg_i + 1) * 8]
		der_last = np.polyder(poly_last, 1)
		der_this = np.polyder(poly_this, 1)
		value_last = np.polyval(der_last, self.durations[seg_i - 1])
		value_this = np.polyval(der_this, 0)
		return value_this - value_last


	def set_Q(self):
		# cost matrix in QP objective, penalize on integral of k-th derivative
		self.Q = np.zeros(shape=(8 * self.num_segments, 8 * self.num_segments))
		for seg_i in range(self.num_segments):
			for t in np.arange(0, self.durations[seg_i], 0.1):
				q = np.array([42*t**5, 30*t**4, 20*t**3, 12*t**2, 6*t, 2, 0, 0]).reshape(8, 1) # accel
				# q = np.array([210*t**4, 120*t**3, 60*t**2, 24*t, 6, 0, 0, 0]).reshape(8, 1) # jerk
				# q = np.array([840*t**3, 360*t**2, 120*t, 24, 0, 0, 0, 0]).reshape(8, 1) # snap
				Q_i = np.dot(q, q.T) * 0.1
				self.Q[seg_i * 8 : (seg_i + 1) * 8, seg_i * 8 : (seg_i + 1) * 8] += Q_i


	def set_A(self):
		# mapping matrix in equality constraints
		self.A = np.zeros(shape=(3 * self.num_segments + 1, 8 * self.num_segments))

		# position constraint (pinned to waypoints)
		for seg_i in range(self.num_segments):
			self.A[seg_i * 2, seg_i * 8 + 7] = 1
			T = self.durations[seg_i]
			self.A[seg_i * 2 + 1, seg_i * 8 : (seg_i + 1) * 8] = np.array([T**7, T**6, T**5, T**4, T**3, T**2, T**1, 1])

		# start derivative zero constraint
		self.A[2 * self.num_segments, 6] = 1

		# derivative continuity constraint
		for seg_i in range(1, self.num_segments):
			self.A[2 * self.num_segments + seg_i, seg_i * 8 + 6] = 1
			T_last = self.durations[seg_i - 1]
			self.A[2 * self.num_segments + seg_i, (seg_i - 1) * 8 : seg_i * 8] = - np.array([7*T_last**6, 6*T_last**5, 5*T_last**4, 4*T_last**3, 3*T_last**2, 2*T_last, 1, 0])

		# end derivative zero constraint
		T = self.durations[-1]
		self.A[3 * self.num_segments, (self.num_segments - 1) * 8 : self.num_segments * 8] = np.array([7*T**6, 6*T**5, 5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0])


	def set_d(self):
		# right-hand-side vector in equality constraints, columns correspond to x, y
		self.d = np.zeros(shape=(3 * self.num_segments + 1, 2))

		for axis in [0, 1]:
			# position constraint
			for seg_i in range(self.num_segments):
				self.d[seg_i * 2, axis] = self.waypoints[seg_i, axis]
				self.d[seg_i * 2 + 1, axis] = self.waypoints[seg_i + 1, axis]

			# start derivative zero constraint
			self.d[2 * self.num_segments, axis] = 0

			# derivative continuity constraint
			for seg_i in range(1, self.num_segments):
				self.d[2 * self.num_segments + seg_i, axis] = 0

			# end derivative zero constraint
			self.d[3 * self.num_segments, axis] = 0


	def set_G(self):
		# inequality constraints on velocity
		self.G = np.zeros(shape=(self.num_segments - 1, 8 * self.num_segments))
		for seg_i in range(1, self.num_segments):
			self.G[seg_i - 1, seg_i * 8 + 6] = -1


	def set_h(self):
		# inequality constraints on velocity
		self.h = - 0.5 * np.ones(shape=(self.num_segments - 1, 1))


	def solve_polys_scipy(self, axis):
		polys_init = np.zeros(self.num_segments * 8)
		constraints = []
		for seg_i in range(self.num_segments):
			constraints.append({'type': 'eq', 'fun': self.constraint_position_value, 'args': (axis, seg_i, 0, self.waypoints[seg_i, axis])})
			constraints.append({'type': 'eq', 'fun': self.constraint_position_value, 'args': (axis, seg_i, self.durations[seg_i], self.waypoints[seg_i + 1, axis])})

			if seg_i != 0:
				constraints.append({'type': 'eq', 'fun': self.constraint_derivative_continuity, 'args': (seg_i,)})


		constraints.append({'type': 'eq', 'fun': self.constraint_derivative_value, 'args': (0, 0, 0)})
		constraints.append({'type': 'eq', 'fun': self.constraint_derivative_value, 'args': (self.num_segments - 1, self.durations[-1], 0)})

		sol = scipy.optimize.minimize(self.objective_func, polys_init, method="SLSQP", options={"maxiter":1000}, constraints=constraints)
		print(sol.success)
		return sol.x


	def solve_polys_cvxopt(self, axis):
		# axis: 0 (x-axis), 1 (y-axis)
		self.set_Q()
		print("cost matrix Q has been set up, size of which is ", self.Q.shape)
		self.set_G()
		self.set_h()
		self.set_A()
		print("constraint mapping matrix A has been set up, size of which is ", self.A.shape)
		self.set_d()
		print("constraint vector d has been set up, size of which is ", self.d[:, axis].shape)
		q = matrix(np.zeros(shape=(8 * self.num_segments, 1)))
		sol = solvers.qp(P=matrix(self.Q), q=q, G=None, h=None, A=matrix(self.A), b=matrix(self.d)[:, axis], kktsolver='ldl')
		# sol = solvers.qp(P=matrix(self.Q), q=q, G=matrix(self.G), h=matrix(self.h), A=matrix(self.A), b=matrix(self.d)[:, axis], kktsolver='ldl')
		polys = np.array(sol['x']).flatten()
		return polys


	def generate_traj(self, solver='cvxopt', iterations=1):
		# iterations: num of iters for refinement
		if solver == 'scipy':
			self.polys_x = self.solve_polys_scipy(0)
			self.polys_y = self.solve_polys_scipy(1)
		elif solver == 'cvxopt':
			for i in range(iterations + 1):
				if i != 0:
					self.refine_segment_times()

				self.polys_x = self.solve_polys_cvxopt(0)
				self.polys_y = self.solve_polys_cvxopt(1)
		#print(self.polys_x)
		#print(self.polys_y)


	def plot_traj(self, dt):
		#plt.close()
		gs = gridspec.GridSpec(7, 1)
		fig = plt.figure()

		num_samples = 0
		times = []
		xs = []
		ys = []
		headings = []
		vels = []
		# vels_f = [] # forward velocity
		# vels_l = [] # lateral velocity
		accels = []
		jerks = []
		for seg_i in range(self.num_segments):
			for t in np.arange(0, self.durations[seg_i], dt):
				poly_x = self.polys_x[seg_i * 8 : (seg_i + 1) * 8]
				poly_y = self.polys_y[seg_i * 8 : (seg_i + 1) * 8]
				x = np.polyval(poly_x, t)
				y = np.polyval(poly_y, t)
				poly_velx = np.polyder(poly_x, 1)
				poly_vely = np.polyder(poly_y, 1)
				poly_accelx = np.polyder(poly_x, 2)
				poly_accely = np.polyder(poly_y, 2)
				poly_jerkx = np.polyder(poly_x, 3)
				poly_jerky = np.polyder(poly_y, 3)
				vel_x = np.polyval(poly_velx, t)
				vel_y = np.polyval(poly_vely, t)
				accel_x = np.polyval(poly_accelx, t)
				accel_y = np.polyval(poly_accely, t)
				jerk_x = np.polyval(poly_jerkx, t)
				jerk_y = np.polyval(poly_jerky, t)
				heading = np.arctan2(vel_y, vel_x)
				vel = np.linalg.norm([vel_x, vel_y])
				# vel_f = vel_x * np.cos(heading) + vel_y * np.sin(heading)
				# vel_l = - vel_x * np.sin(heading) + vel_y * np.cos(heading)
				accel = np.linalg.norm([accel_x, accel_y])
				jerk = np.linalg.norm([jerk_x, jerk_y])
				#if t == 0:
				#	print(x, y)
				xs.append(x)
				ys.append(y)
				headings.append(heading)
				vels.append(vel)
				# vels_f.append(vel_f)
				# vels_l.append(vel_l)
				accels.append(accel)
				jerks.append(jerk)
				times.append(np.sum(self.durations[0:seg_i]) + t)
				num_samples += 1

		# plot traj
		ax = plt.subplot(gs[0:3, 0])
		ax.scatter(self.waypoints[:, 0], self.waypoints[:, 1], s=5, color="red")
		ax.plot(xs, ys)
		ax.axis('equal')
		ax.set_xlabel('x [m]')
		ax.set_ylabel('y [m]')
		ax.grid()

		# plot heading
		ax = plt.subplot(gs[3, 0])
		ax.plot(times, headings)
		ax.set_xlabel('time')
		ax.set_ylabel('heading [rad]')

		# plot velocity
		ax = plt.subplot(gs[4, 0])
		ax.plot(times, vels, color="blue", label="norm")
		# ax.plot(times, vels_f, color="red", label="forward")
		# ax.plot(times, vels_l, color="green", label="lateral")
		# ax.legend()
		ax.set_xlabel('time')
		ax.set_ylabel('vel [m/s]')

		# plot acceleration
		ax = plt.subplot(gs[5, 0])
		ax.plot(times, accels)
		ax.set_xlabel('time')
		ax.set_ylabel('accel [m/s^2]')

		# plot jerk
		ax = plt.subplot(gs[6, 0])
		ax.plot(times, jerks)
		ax.set_xlabel('time')
		ax.set_ylabel('jerk [m/s^3]')

		#plt.show()
		return fig


	def publish_reftraj(self, dt = 1):
		msg = RefTrajAxyt()
		axyt = []
		seg_i = 0
		time_offset = 0
		for t in np.arange(0, self.time, dt):
			t_seg = t - time_offset
			if t_seg > self.durations[seg_i]:
				time_offset += self.durations[seg_i]
				seg_i += 1
				t_seg = t - time_offset
			poly_x = self.polys_x[seg_i * 8 : (seg_i + 1) * 8]
			poly_y = self.polys_y[seg_i * 8 : (seg_i + 1) * 8]
			x = np.polyval(poly_x, t_seg)
			y = np.polyval(poly_y, t_seg)
			poly_velx = np.polyder(poly_x, 1)
			poly_vely = np.polyder(poly_y, 1)
			vel_x = np.polyval(poly_velx, t_seg)
			vel_y = np.polyval(poly_vely, t_seg)
			heading = np.arctan2(vel_y, vel_x)
			axyt.append(heading)
			axyt.append(x)
			axyt.append(y)
			axyt.append(t)
		# row major format
		axyt_np = np.array(axyt).reshape(-1, 4)
		axyt_np = axyt_np.T
		# print(axyt_np.shape)
		msg.axyt = list(axyt_np.flatten())
		msg.modifier = [0, 0, 0, 1]
		msg.looping = self.loop
		self.pub.publish(msg)
		return msg


	def save_reftraj(self, msg, save_file):
		msg_dict = {}
		msg_dict["axyt"] = [float(c) for c in msg.axyt]
		msg_dict["modifier"] = msg.modifier
		msg_dict["looping"] = msg.looping
		folder_path = os.path.join(rospkg.RosPack().get_path("ugv_localization"), "traj")
		if not os.path.exists(folder_path):
			os.mkdir(folder_path)
		file_path = os.path.join(folder_path, "%s.yaml" % save_file)
		with open(file_path, 'w') as f:
			data = yaml.dump(msg_dict, f)
		print("generate trajectory saved to %s" % file_path)


	def save_plot(self, fig, save_file):
		folder_path = os.path.join(rospkg.RosPack().get_path("ugv_localization"), "traj")
		if not os.path.exists(folder_path):
			os.mkdir(folder_path)
		file_path = os.path.join(folder_path, "%s.png" % save_file)
		fig.savefig(file_path)
		print("trajectory plot saved to %s" % file_path)
		


if __name__ == "__main__":

	rospy.init_node('traj_opt', anonymous=True)

	traj_opt = TrajOptimizer(topic_out="/reftraj", loop=False)

	node_name = rospy.get_name()
	if rospy.has_param(node_name + '/start_time') and rospy.has_param(node_name + '/end_time') and rospy.has_param(node_name + "/bagfile") and rospy.has_param(node_name + "/topic"):
		start_time = rospy.get_param(node_name + '/start_time')
		end_time = rospy.get_param(node_name + '/end_time')
		bagfile = rospy.get_param(node_name + "/bagfile")
		topic = rospy.get_param(node_name + "/topic")

		if rospy.has_param(node_name + '/wps_every_secs'):
			wps_every_secs = rospy.get_param(node_name + '/wps_every_secs')
		else:
			wps_every_secs = 1

		last_time = -1

		wps_np = np.empty((0, 2))
		bag_path = os.path.join(rospkg.RosPack().get_path("ugv_localization"), "bag", "%s.bag" % bagfile)
		bag = rosbag.Bag(bag_path)
		for topic, msg, t in bag.read_messages(topics=[topic]):
			if t.secs < start_time or t.secs > end_time:
				continue

			if t.secs - last_time >= wps_every_secs:
				new_wps = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]])
				wps_np = np.vstack((wps_np, new_wps))
				last_time = t.secs
				
		bag.close()
		traj_opt.set_wps(wps_np)

	else:
		npy_path = os.path.join(rospkg.RosPack().get_path("ugv_localization"), "records/wps.npy")
		traj_opt.load_npy(npy_path)

	vel = float(rospy.get_param(node_name + '/vel')) if rospy.has_param(node_name + '/vel') else 1.0
	accel = float(rospy.get_param(node_name + '/accel')) if rospy.has_param(node_name + '/accel') else 1.0
	
	traj_opt.estimate_segment_times()
	traj_opt.generate_traj(iterations=1)
	fig = traj_opt.plot_traj(0.1)
	msg = traj_opt.publish_reftraj()
	
	if rospy.has_param(node_name + '/save_traj'):
		save_traj = rospy.get_param(node_name + '/save_traj')
		traj_opt.save_reftraj(msg, save_traj)
		tra_opt.save_plot(fig, save_traj)

	rospy.spin()



