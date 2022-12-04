#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import parse_data
import matplotlib.pyplot as plt

plt.ion()

rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)

def yaw_from_quaternion(x, y, z, w):
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll_x = math.atan2(t0, t1)
        # t2 = +2.0 * (w * y - z * x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

cones = parse_data.extract_cones("CompetitionMap1.txt")
for cone in cones:
        plt.scatter(cone[0], cone[1])
plt.show()
close_cones = []

class MySubscriber(object):
    def __init__(self):
        self.position = [0, 0]
        self.start_position = None
        self.orientation = 0
        rospy.Subscriber("/fsds/imu", Imu, self.callback_imu)
        rospy.Subscriber("/fsds/gps", NavSatFix, self.callback_gps)

    def callback_imu(self, data):
        orientation = data.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        self.rotation = yaw_from_quaternion(x, y, z, w)
        rospy.loginfo(self.rotation)

    def callback_gps(self, data):
        self.position = [data.latitude, data.longitude]
        if self.start_position == None:
            self.start_position = self.position
        dif1 = (self.position[1]-self.start_position[1])*6378.137
        dif0 = (self.position[0]-self.start_position[0])*6378.137
        print(dif0, dif1)
        close_cones = parse_data.pick_close_cones_in_view(cones, self.position, self.rotation)
        #print(close_cones)
        #print(self.position, self.rotation)




if __name__ == '__main__':
    my_subs = MySubscriber()
    rospy.spin()