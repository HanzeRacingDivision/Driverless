from math import cos, sin, atan2, sqrt, pi, atan, pow
from constants import *

def yaw_from_quaternion(x, y, z, w):
    """
    Converts a quaternion into yaw (rotation in z).
    :param x, y, z, w: quaternion in the standard format
    :return: yaw in z
    """
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return atan2(t3, t4)

def rotate(theta, x1, x2):
    """
    Rotates points x1 and x2 around zero point in the counterclockwise direction
    :param theta: rotation angle
    :param x1, x2: points to rotate
    :return: rotated x1, x2
    """
    return cos(theta)*x1 - sin(theta)*x2, sin(theta)*x1 + cos(theta)*x2

def calculate_throttle(x_speed, y_speed):
    """
    Returns throttle based on current speed and target speed.
    """
    # Calculate the velocity in the vehicle's frame
    velocity = sqrt(pow(x_speed, 2) + pow(y_speed, 2))
    # The lower the velocity, the more throttle, up to max_throttle
    return max_throttle * max(1 - velocity / target_speed, 0)