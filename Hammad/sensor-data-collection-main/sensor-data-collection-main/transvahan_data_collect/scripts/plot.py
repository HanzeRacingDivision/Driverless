#!/usr/bin/env python3

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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

	temp = np.genfromtxt(sys.argv[1], delimiter=',')
	index= int(sys.argv[2])
	data = temp[:,index]

	data = lp_filter(data[1:])

	if int(sys.argv[3]):
		data_y = temp[:, index + 1]
		data_y = data_y[1:]

	if not int(sys.argv[3]):
		plt.figure(figsize=(15,15))
		plt.plot(data[:],'.')
		plt.show()
	else:
		plt.figure(figsize=(15,15))
		plt.plot(data[:],data_y[:],'.')
		plt.show()

if __name__ == '__main__':
	main()
