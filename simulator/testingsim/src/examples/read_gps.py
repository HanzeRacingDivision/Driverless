#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix  # see https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

def callback(data):
    latitude = data.latitude
    altitude = data.altitude
    longitude = data.longitude
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fsds/gps", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()