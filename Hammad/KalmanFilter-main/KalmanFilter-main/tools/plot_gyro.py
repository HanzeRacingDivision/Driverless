import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import json


def get_data(filename):
    x = []
    start = None
    y = []
    z = []
    t = []
    json_file = open(filename)
    for line in json_file:
        line_dict = json.loads(line.strip())
        attitude = json.loads(line_dict['gyro'])
        if not start:
            start = line_dict['time_stamp']
        t.append(line_dict['time_stamp']-start)
        x.append(attitude['b'])
        y.append(attitude['c'])
        z.append(attitude['d'])
    lines = (x,y,z)
    return (lines,t)
    
def plot(data):
    for point in data:
        plt.plot(t,point)

plt.show()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file', help='log file to process')
    args = parser.parse_args()
    data,t = get_data(args.log_file)
    fig = plt.figure()
    plot(data)
    plt.legend(['x','y','z'])
    plt.show()
        
