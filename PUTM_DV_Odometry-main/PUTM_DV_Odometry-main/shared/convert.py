#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

IMU_OUT = '/imu'
ODOM_OUT = '/odometry'
GPS_OUT = '/gps'

IMU_IN = '/fsds/imu'
ODOM_IN = '/fsds/testing_only/odom'
GPS_IN = '/fsds/gps'


class Converter:
    def __init__(self):
        self.publisher_imu = rospy.Publisher(IMU_OUT, Imu, queue_size=10)
        self.publisher_odom = rospy.Publisher(ODOM_OUT, Odometry, queue_size=10)
        self.publisher_gps = rospy.Publisher(GPS_OUT, NavSatFix, queue_size=10)
        rospy.init_node('converter', anonymous=True)
        self.rate = rospy.Rate(100)
        rospy.Subscriber(IMU_IN, Imu, self.imu_callback)
        rospy.Subscriber(ODOM_IN, Odometry, self.odom_callback)
        rospy.Subscriber(GPS_IN, NavSatFix, self.gps_callback)
        rospy.spin()


    def odom_callback(self, data):
        data.header.frame_id = 'odom'
        data.child_frame_id = 'base_link'
        self.send_odom(data)
    

    def send_odom(self,data):
        self.publisher_odom.publish(data)
        self.rate.sleep()


    def imu_callback(self, data):
        data.header.frame_id = 'base_link'
        self.send_imu(data)
    

    def send_imu(self, data):
        self.publisher_imu.publish(data)
        self.rate.sleep()
    

    def gps_callback(self, data):
        data.header.frame_id = 'base_link'
        self.send_gps(data)
    
    
    def send_gps(self, data):
        self.publisher_gps.publish(data)
        self.rate.sleep()


if __name__ == '__main__':
    con = Converter()
