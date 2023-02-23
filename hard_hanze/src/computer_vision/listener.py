#!/usr/bin/env python
import rospy
from hard_hanze.msg import CVdata

def callback(message):
    print("X = ", message.x)
    print("Y = ", message.y)
    print("Z = ", message.z)
    print("Label = ", message.label)

def listener():
    rospy.init_node('eyes', anonymous=True)
    rospy.Subscriber("coords", CVdata, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()