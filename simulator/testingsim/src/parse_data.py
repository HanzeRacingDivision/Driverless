from math import cos, sin, atan2, sqrt, pi, atan
from constants import *

def extract_cones(filename):
    """
    Extract cones from file to an array.
    """
    f = open(filename, 'r')
    lines = f.readlines()
    counter = 0
    current_index = -1
    cones = []
    for line in lines:
        line = line.strip()
        if line == "location:":
            counter = 1
            cones.append([0, 0, 0, 0])
            current_index +=1
            continue

        if counter == 1:
            x = float(line.split(" ")[1])
            cones[current_index][0] = x
            counter = 2
            continue

        if counter == 2:
            x = float(line.split(" ")[1])
            cones[current_index][1] = x
            counter = 3
            continue

        if counter == 3:
            x = float(line.split(" ")[1])
            cones[current_index][2] = x
            counter = 4
            continue

        if counter == 4:
            x = float(line.split(" ")[1])
            cones[current_index][3] = x
            counter =-1
    return cones

def cone_car_distance(cone_position, car_position):
    return sqrt((cone_position[0]-car_position[0])**2 + (cone_position[1]-car_position[1])**2)

def pick_close_cones(cone_list, car_position):
    """
    Filters cones to the within vision range of the car using Eucledian distance.
    """
    close_cones = []
    for cone in cone_list:
        if cone_car_distance(cone, car_position) < DISTANCE:
            close_cones.append(cone)
    return close_cones

