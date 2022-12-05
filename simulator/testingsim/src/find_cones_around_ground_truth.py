#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import parse_data
import matplotlib.pyplot as plt

rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)

def yaw_from_quaternion(x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

cones = parse_data.extract_cones("TrainingMap.txt")

# plt.ion()

class MySubscriber(object):
    def __init__(self):
        self.position = [0, 0]
        self.start_position = None
        self.orientation = 0
        rospy.Subscriber("/fsds/testing_only/odom", Odometry, self.callback)

    def callback(self, data):
        self.position = [data.pose.pose.position.y, data.pose.pose.position.x]
        orientation = data.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # plitting
        # plt.axis([-250, 250, -250, 250])
        # plt.scatter(self.position[0], self.position[1])
        # plt.pause(0.00001)
        # plt.clf()

        self.orientation = yaw_from_quaternion(x, y, z, w)
        
        close_cones = parse_data.pick_close_cones_in_view(cones, self.position, self.orientation)
        # print(close_cones)
        relative_cones = parse_data.make_cones_relative(close_cones, self.position, self.orientation)
        print(relative_cones)

if __name__ == '__main__':
    my_subs = MySubscriber()
    rospy.spin()
