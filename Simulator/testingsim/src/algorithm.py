#!/usr/bin/env python

# libraries
import rospy
from math import cos, sin, atan2, sqrt, pi, atan, pow
# messages
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from fs_msgs.msg import ControlCommand
from fs_msgs.msg import FinishedSignal
# custom libraries
from control.control import control
from parse_data import extract_cones, pick_close_cones
from utils import yaw_from_quaternion, rotate, calculate_throttle
from constants import *

# Extracting cones from saved map
cones = extract_cones("CompetitionMap1.txt")

def position_callback(data):
    """
    Callback for the subcriber of the ROS topic with position of the car.
    It retrieves cones which are around and generates the steering command using
    the custom control library.
    """
    # update position of the model
    global model
    model.position = [data.pose.pose.position.x, data.pose.pose.position.y]
    orientation = data.pose.pose.orientation
    model.orientation = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    
    # pick steering cones
    close_cones = pick_close_cones(cones, model.position)
    steering_cones = []
    for cone in close_cones:
            y, x = rotate(-model.orientation, cone[0] - model.position[0], cone[1] - model.position[1])
            if y > IGNORED_DISTANCE and cone[3] != 2:
                new_cone = {"Label": "Yellow" if cone[3] == 1 else "Blue", "Zpos": None, "Ypos": y, "Xpos": -x, "Time": None}
                steering_cones.append(new_cone)
    
    # generating angle using the control
    angle = control(steering_cones) 
    # adding the controls to the model and publishing the command
    model.cc.header.stamp = rospy.Time.now()
    model.cc.steering = angle
    model.cc.brake = 0
    controllpublisher.publish(model.cc)

def speed_callback(data):
    """
    Callback function for the update of the velocity of the model.
    """
    global model
    x = data.twist.linear.x
    y = data.twist.linear.y
    model.cc.throttle = calculate_throttle(x, y)

class Model:
    """
    Class to represent the car with all the necessary information about it.
    """
    def __init__(self):
        self.position = [0, 0]
        self.start_position = None
        self.orientation = 0
        self.cc = ControlCommand()

if __name__ == '__main__':
    # Car model with all the information
    model = Model()
    # Nodes of ROS
    rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)
    odometry_subscriber = rospy.Subscriber("/fsds/testing_only/odom", Odometry, position_callback)
    gss_subscriber = rospy.Subscriber("/fsds/gss", TwistStamped, speed_callback)
    controllpublisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
    finishedpublisher = rospy.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)
    rospy.spin()
