#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu  # see http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html

def callback(data):
    orientation = data.orientation #(x, y, z)
    angular_velocity = data.angular_velocity #(x, y, z)
    linear_acceleration = data.linear_acceleration #(x, y, z)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fsds/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()