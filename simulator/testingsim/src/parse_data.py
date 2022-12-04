from math import cos, sin, atan2, sqrt, pi, atan

distance = 10

field_of_view = pi/2

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

def cone_car_angle(cone_position, car_angle, car_position):
    """
    Verifies whether the cone is within the angle which the car sees.
    """
    cone_vec_x = cone_position[0] - car_position[0]
    cone_vec_y = cone_position[1] - car_position[1]
    angle = atan(cone_vec_y/cone_vec_x)
    print(car_angle + field_of_view, angle, car_angle - field_of_view)
    if car_angle + field_of_view > angle and angle > car_angle - field_of_view:
        return True
    return False

def pick_close_cones_in_view(cone_list, car_position, car_angle):
    """
    Filters cones to the within vision range of the car using Eucledian distance
    and angle of the view of the car.
    """
    close_cones = []
    for cone in cone_list:
        if cone_car_distance(cone, car_position) < distance and cone_car_angle(cone, car_angle, car_position):
            close_cones.append(cone)
    return close_cones

def mult(t, x1, x2):
    return cos(t)*x1 - sin(t)*x2, sin(t)*x1 + cos(t)*x2

def make_cones_relative(cones, car_position, car_angle):
    """
    Transforms the coordinates of the cone such that it now lies relative to the car.
    """
    # rotation matrix = (cos t -sin t)
    #                   (sin t  cos t)
    # vector x = (x1, x2)
    # R dot x = (cos t x1 - sin t x2, sin t x1 + cos t x2)
    for cone in cones:
        angle_diff = cone_car_angle(cone, car_angle, car_position)
        x, y = mult(angle_diff, cone[0], cone[1])
        cone[0] = x
        cone[1] = y

    return cones
