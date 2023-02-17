#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped # see https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

def callback(data):
    x = data.twist.linear.x
    y = data.twist.linear.y
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fsds/gss", TwistStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()