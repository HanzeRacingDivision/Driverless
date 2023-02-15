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
        attitude = json.loads(line_dict['position'])
        lines.append(attitude)
    return lines
def plot(data):
    m = len(data)
    n = int(m/100)
    for x in range(0,m,n):
        point = data[x]
        xs = point['b']
        ys = point['c']
        zs = point['d']
        ax.scatter(xs, ys, zs)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file', help='log file to process')
    args = parser.parse_args()
    data = get_data(args.log_file)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot(data)
    plt.show()
        
