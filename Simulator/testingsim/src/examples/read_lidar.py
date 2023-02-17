#!/usr/bin/env python
import rospy
import numpy
import math
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2  # see https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html

cones_range_cutoff = 7 # meters

def pointgroup_to_cone(group):
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))


def callback(data):
    point_cloud = []
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
     #print(" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
     point_cloud.append(p[0])
     point_cloud.append(p[1])
     point_cloud.append(p[2])
    
    # no points
    if len(point_cloud) < 3:
        rospy.loginfo("")

    # Convert the list of floats into a list of xyz coordinates
    points = numpy.array(point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))
    

    # Go through all the points and find nearby groups of points that are close together as those will probably be cones.

    current_group = []
    cones = []
    for i in range(1, len(points)):

        # Get the distance from current to previous point
        distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

        if distance_to_last_point < 0.1:
            # Points closer together then 10 cm are part of the same group
            current_group.append({'x': points[i][0], 'y': points[i][1]})
        else:
            # points further away indiate a split between groups
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                # calculate distance between lidar and cone
                if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff:
                    cones.append(cone)
                current_group = []
    
    rospy.loginfo(cones)

    
def listener():

    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("/fsds/lidar/Lidar", PointCloud, callback)
    rospy.Subscriber("/fsds/lidar/Lidar", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()