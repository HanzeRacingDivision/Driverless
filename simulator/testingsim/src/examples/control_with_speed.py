#!/usr/bin/env python
import rospy, math
from fs_msgs.msg import ControlCommand
from fs_msgs.msg import FinishedSignal
from geometry_msgs.msg import TwistStamped

# the number of seconds to do a full steering cycle (neutral -> full right -> full left -> neutroal)
STEERING_PERIOD = 5
# hz at what the car setpoints are published
SETPOINT_FREQUENCY = 5.0
# The throtle
THROTLE = 0
# Autonomous system constatns
max_throttle = 0.2 # m/s^2
target_speed = 4 # m/s

def mult(t, x1, x2):
    return cos(t)*x1 - sin(t)*x2, sin(t)*x1 + cos(t)*x2

def calculate_throttle(x, y):
    # Calculate the velocity in the vehicle's frame
    velocity = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    # the lower the velocity, the more throttle, up to max_throttle
    return max_throttle * max(1 - velocity / target_speed, 0)

time = 0

def callback(data):
    global cc
    x = data.twist.linear.x
    y = data.twist.linear.y
    cc.throttle = calculate_throttle(x, y)

def finish(x):
    finishedpublisher.publish(FinishedSignal())

if __name__ == '__main__':
    controllpublisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=10)
    finishedpublisher = rospy.Publisher('/fsds/signal/finished', FinishedSignal, queue_size=1)
    rospy.init_node('listener', anonymous=True)
    gss_subscriber = rospy.Subscriber("/fsds/gss", TwistStamped, callback)

    cc = ControlCommand() # importing the structure from fsds messages
    cc.header.stamp = rospy.Time.now() # current time stamp
    cc.throttle = THROTLE
    cc.steering = 0 #math.sin(time * (math.pi / STEERING_PERIOD)) # steering angle
    cc.brake = 0 # breaking
    controllpublisher.publish(cc)
    time += 1.0 / SETPOINT_FREQUENCY

    rospy.spin()
    
