import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import json


def get_data(filename):
    lines = []
    json_file = open(filename)
    for line in json_file:
        line_dict = json.loads(line.strip())
        attitude = json.loads(line_dict['atitude'])
        lines.append(attitude)
    return lines

def data_gen(num):
    attitude = data[num]
    vx = attitude['b']
    vy = attitude['c']
    vz = attitude['d']
    mag = max((vx**2 + vy**2 + vz**2)**.5,0.00001)
    ax.cla()
    ax.quiver(0, 0, 0, vx, vy, vz, pivot="tail", color="black")
    ax.quiver(0, 0, 0, vx/mag, vy/mag, vz/mag, pivot="tail", color="red")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.view_init(elev=30, azim=60)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file', help='log file to process')
    args = parser.parse_args()
    data = get_data(args.log_file)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    n = len(data)
    ani = animation.FuncAnimation(fig, data_gen, range(0,n,int(n/100)), blit=False)
    plt.show()
        
