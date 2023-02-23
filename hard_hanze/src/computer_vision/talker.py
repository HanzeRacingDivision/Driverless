#!/usr/bin/env python

#camera, raspberry pie, 2 esp 32 encoders, gps, steering motor, throttle

import rospy
from hard_hanze.msg import CVdata

def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub = rospy.Publisher('coords', CVdata, queue_size=10)
    rospy.init_node('mouth', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(10) # 1hz

    X = -101
    Y = 201
    Z = 595
    Label = "Yellow"
    coordinates = CVdata()
    coordinates.x = X
    coordinates.y = Y
    coordinates.z = Z
    coordinates.label = Label

    while not rospy.is_shutdown():
        pub.publish(coordinates)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass