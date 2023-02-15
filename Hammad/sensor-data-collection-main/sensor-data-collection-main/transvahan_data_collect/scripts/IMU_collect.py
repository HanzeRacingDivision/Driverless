#!/usr/bin/env python3

import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from witmotion import IMU
from datetime import date
#EACH READING AT interval 0.1s

def lp_filter(arr, alpha = 0.02):
    fil_arr = [arr[0]]
    for a in arr[1:]:
        fil_arr.append(alpha*a+(1-alpha)*fil_arr[-1])
    return fil_arr

def hp_filter(arr, alpha = 0.02):
    fil_arr = [arr[0]]
    for i in range(arr.shape[0]):
        if i==0:
            continue
        fil_arr.append(alpha*fil_arr[-1]+(1-alpha)*(arr[i]-arr[i-1]))
    return fil_arr


def main():

	imu = IMU()
	time.sleep(1)
	acc_data = []
	yaw_rate = []

	try:
		while True:
			temp = imu.get_acceleration()
			temp = list(temp)
			temp.append(time.time())
			temp = tuple(temp)
			acc_data.append(temp)
			temp_yaw = imu.get_angular_velocity()
			yaw_rate.append([temp_yaw[2],time.time()])
			time.sleep(0.1)

	except KeyboardInterrupt:

		df_acc = pd.DataFrame(acc_data)
		df_yaw = pd.DataFrame(yaw_rate)
		df_acc.to_csv('./../Data/' + str(date.today()) + '_' + sys.argv[1] + '_acc.csv')
		df_yaw.to_csv('./../Data/' + str(date.today()) + '_' + sys.argv[1] + '_yaw.csv')

		acc_data = np.array(acc_data)
		yaw_rate = np.array(yaw_rate)

		fig, axes = plt.subplots(3, 2, figsize=(15,10))
		axes[0][0].plot(acc_data[:,0], '.')
		axes[0][0].grid()
		axes[0][0].set_ylabel('acc_x')

		axes[0][1].plot(lp_filter(acc_data[:,0]), '.')
		axes[0][1].grid()
		axes[0][1].set_ylabel('acc_x_filter')

		axes[1][0].plot(acc_data[:,1], '.')
		axes[1][0].grid()
		axes[1][0].set_ylabel('acc_y')

		axes[1][1].plot(lp_filter(acc_data[:,1]), '.')
		axes[1][1].grid()
		axes[1][1].set_ylabel('acc_y_filter')

		axes[2][0].plot(yaw_rate[:,1], '.')
		axes[2][0].grid()
		axes[2][0].set_ylabel('yaw_rate')

		axes[2][1].plot(hp_filter(yaw_rate[:,1]), '.')
		axes[2][1].grid()
		axes[2][1].set_ylabel('yaw_rate_filter')

		plt.show()

if __name__ == '__main__':
	main()
