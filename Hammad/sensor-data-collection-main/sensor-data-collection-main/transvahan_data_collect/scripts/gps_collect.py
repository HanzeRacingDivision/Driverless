#!/usr/bin/env python3

import numpy as np
import serial
import math
import os
import pandas as pd
import sys
import matplotlib.pyplot as plt
import time
from datetime import date

#EACH READING AT INTERVAL 1s

class Readgps():
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        #Constants used for conversion
        self.a = 6378137.0 #Radius of the earth
        self.odom_origin = np.array([0 , 0])
        #Counter for initializing GPS
        self.gps_counter = 0
        #Angular offset between odom and map frame
        #Multiplying rotation matrix to pose will rotate to axes anti-clockwise
        #We have to align x with East so rotate accordingly
        self.angle = np.deg2rad(180)
        self.rotation_matrix = np.array([[np.cos(self.angle), np.sin(self.angle)],
                                         [-np.sin(self.angle), np.cos(self.angle)]])
    
    def read_gps(self):
        '''
        Read Lat, Long and altitude values
        '''
        ser = serial.Serial('/dev/ttyACM0', 4800, timeout=5)
        while True:
            line = ser.readline()
            line = line.decode('utf-8')
            splitline = line.split(',')
            if splitline[0] == '$GPGGA':

                if splitline[6] == '0':
                    print("Current GPS data invalid")
                    self.gps_counter = 0
                
                else:
                    lat = splitline[2]
                    lat_deg = lat[:2] 
                    lat_min = lat[2:] 
                    latdirection = splitline[3]

                    lon = splitline[4]
                    lon_deg = lon[:3].lstrip("0")
                    lon_min = lon[3:]
                    londirection = splitline[5]

                    num_satellites = splitline[7]
                    
                    self.latitude = int(lat_deg) + float(lat_min)/60
                    self.longitude = int(lon_deg) + float(lon_min)/60
                    self.altitude = float(splitline[9])
                    #Convert to xyz
                    self.conv_relative()
                    if self.gps_counter == 0:
                        self.odom_origin[0] = self.x
                        self.odom_origin[1] = self.y
                        self.x = 0
                        self.y = 0
                    else:
                        path.append(np.array([self.x, self.y, time.time()]))
                    self.gps_counter += 1

    def conv_relative(self):
        '''
        Convert to relative coordinates using mercartor scale
        '''
        s = np.cos(self.latitude * np.pi/180)
        self.x = s * self.a * (np.pi*self.longitude/180)
        self.x = self.x - self.odom_origin[0]
        self.y = s * self.a * np.log(np.tan(np.pi*(90 + self.latitude)/360))
        self.y = self.y - self.odom_origin[1]
        temp = np.array([self.x, self.y])
        temp = np.dot(temp, self.rotation_matrix)
        
        if not self.gps_counter == 0:
            self.x = temp[0]
            self.y = temp[1]
        self.z = self.altitude

def main():

    global path
    path = []

    Readgpsobj = Readgps()
    
    try:
        Readgpsobj.read_gps()

    except KeyboardInterrupt:

        df_path = pd.DataFrame(path)
        df_path.to_csv('./../Data/' + str(date.today()) + '_' + sys.argv[1] + '_gps.csv')

        path = np.array(path)

        if path.shape[0] == 0:
            print("No data recorded")
            exit()

        plt.figure(figsize=(15,10))
        plt.xlim(-1,200)
        plt.ylim(-1,200)
        plt.plot(path[:,0],path[:,1],'.')
        plt.show()

if __name__ == '__main__':
	main()
