#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import parse_data
import matplotlib.pyplot as plt
from math import cos, sin, atan2, sqrt, pi, atan

rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)

cones = parse_data.extract_cones("CompetitionMap1.txt")

class MySubscriber(object):
    def __init__(self):
        self.position = [0, 0]
        self.start_position = None
        self.orientation = 0
        rospy.Subscriber("/fsds/testing_only/odom", Odometry, self.callback)

    def callback(self, data):
        self.position = [data.pose.pose.position.x, data.pose.pose.position.y]
        orientation = data.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # plotting position
        plt.axis([-100, 100, -100, 100])
        plt.scatter(self.position[0], self.position[1])
        plt.pause(0.0000000001)
        # plt.clf()

        close_cones = parse_data.pick_close_cones(cones, self.position)

        # plotting cones
        # plt.axis([-10, 10, 0, 10])
        # for cone in close_cones:
        #     y, x = mult(-self.orientation, cone[0] - self.position[0], cone[1] - self.position[1])
        #     if y > 0:
        #         plt.scatter(-x, y)
        # plt.pause(0.0000000001)
        # plt.clf()

if __name__ == '__main__':
    my_subs = MySubscriber()
    rospy.spin()
