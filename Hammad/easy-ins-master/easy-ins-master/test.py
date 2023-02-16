#!/usr/bin/python

import marshal
import sys

class INS:
	def __init__(self, antenna_to_imu_transform):
		self.antenna_to_imu_transform = antenna_to_imu_transform

		self.position = None
		self.velocity = None
		
	def handle_position(self, x, y, height):
		self.position = (x, y, height)

	def handle_velocity(self, north, east, down):
		self.velocity = (north, east, down)

	def handle_imu(
		self,
		angular_velocity_x, angular_velocity_y, angular_velocity_z, angular_velocity_covariance, 
		linear_acceleration_x, linear_acceleration_y, linear_acceleration_z, linear_acceleration_covariance,
		orientation_w, orientation_x, orientation_y, orientation_z, orientation_covariance, 
		heading, pitch, roll,
	):
		if self.position is None:
			return

		return({
			'position_x': self.position[0],
			'position_y': self.position[1],
			'position_z': self.position[2],
			'orientation_x': orientation_x,
			'orientation_y': orientation_y,
			'orientation_z': orientation_z,
			'orientation_w': orientation_w,
		})


ins = INS(
	# The IMU is approximately one meter behind the antenna (over the rear axle) and down 0.40m.
	antenna_to_imu_transform = [-1.0, 0, 0.40],
)

log_file_name = sys.argv[1]
f = open(log_file_name, 'r')

while True:
	try:
		sensor_data = marshal.load(f)
	except:
		break


	data_type = sensor_data['type']

	if data_type == 'position':
		ins.handle_position(
			sensor_data['x'],
			sensor_data['y'],
			sensor_data['height'],
		)
	elif data_type == 'velocity':
		ins.handle_velocity(
			sensor_data['north'],
			sensor_data['east'],
			sensor_data['down'],
		)
	elif data_type == 'imu':
		update = ins.handle_imu(
			angular_velocity_x = sensor_data['angular_velocity.x'],
			angular_velocity_y = sensor_data['angular_velocity.y'],
			angular_velocity_z = sensor_data['angular_velocity.z'],
			angular_velocity_covariance = sensor_data['angular_velocity_covariance'],
			linear_acceleration_x = sensor_data['linear_acceleration.x'],
			linear_acceleration_y = sensor_data['linear_acceleration.y'],
			linear_acceleration_z = sensor_data['linear_acceleration.z'],
			linear_acceleration_covariance = sensor_data['linear_acceleration_covariance'],
			orientation_w = sensor_data['orientation.w'],
			orientation_x = sensor_data['orientation.x'],
			orientation_y = sensor_data['orientation.y'],
			orientation_z = sensor_data['orientation.z'],
			orientation_covariance = sensor_data['orientation_covariance'],
			heading = sensor_data['heading'],	
			pitch = sensor_data['pitch'],	
			roll = sensor_data['roll'],	
		)	
		print(update)
