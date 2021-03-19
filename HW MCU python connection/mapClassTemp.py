import numpy as np


class Map:
    """ A Parent Map Class that SLAM, PathPlanning and other simulations inherit from """

    def __init__(self, carStartPos=[5,11]):  # variables here that define the scenario/map
        self.car = self.Car(carStartPos[0], carStartPos[1])
        self.left_cone_list = []
        self.right_cone_list = []
        self.finish_line_cones = [] #holds 2 cones, 1 left and 1 right

    class Car:
        def __init__(self, x, y, angle=0, max_steering=80, max_acceleration=4.0):
            self.position = [x, y]
            self.velocity = 0.0
            self.angle = angle
            self.length = 2
            self.width = 1
            self.max_acceleration = max_acceleration
            self.max_steering = max_steering
            self.max_velocity = 5

            self.acceleration = 0.0
            self.steering = 0.0
            self.fov_range = 60  #(thijs) this is the actual (camera) field of view variable
            self.auto = False

        def update(self, dt):
            self.velocity += self.acceleration * dt
            self.velocity = max(-self.max_velocity, min(self.velocity, self.max_velocity))

            if self.steering:
                turning_radius = self.length / np.sin(np.radians(self.steering))
                angular_velocity = self.velocity / turning_radius
            else:
                angular_velocity = 0

            self.position[0] += dt * self.velocity * np.cos(self.angle)
            self.position[1] += dt * self.velocity * np.sin(self.angle)
            self.angle += np.degrees(angular_velocity) * dt

    class Cone:
        """ Class for storing the coordinates and visibility of each cone """
        def __init__(self, x, y, leftOrRight):
            self.position = [x, y]
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is
            self.connecData = []

    class Target:
        """ 'Target' child class only used in PathPlanning """
        def __init__(self, x, y):
            self.position = [x, y]
            self.passed = 0 #counts the number of times it's been passed
            self.connecData = []


def get_angle_between(obj_1, obj_2, obj_2_angle):
    return(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle)

