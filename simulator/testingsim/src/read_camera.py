#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image  # see https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html

def callback(data):
    rospy.loginfo(data.header)
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/fsds/camera/Camera1", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()