from pygame.math import Vector2
import numpy as np
from enum import Enum
from scipy.interpolate import splprep, splev
from constants import *

class Side(Enum):
    LEFT = 1
    RIGHT = 2


class Cones:
    """ Holds all lists of Cone classes which are used to perform computation """
    def __init__(self):
        self.list = {Side.LEFT: [], Side.RIGHT: []}  # all cones loaded from the map
        self.polar = []
        self.visible = {Side.LEFT: [], Side.RIGHT: []}  # seen by the car
        self.in_fov = {Side.LEFT: [], Side.RIGHT: []}  # field of view of the car
        self.image = {Side.LEFT: None, Side.RIGHT: None}
        self.new_visible_cone_flag = {Side.LEFT: False, Side.RIGHT: False}
        self.first_cone_found = {Side.LEFT: False, Side.RIGHT: False}
        self.first_visible_cone = {Side.LEFT: 0, Side.RIGHT: 0}

        self.boundary_sample = {Side.LEFT: [[-100 for _ in range(5)], [-100 for _ in range(5)]],
                                Side.RIGHT: [[-100 for _ in range(5)],
                                             [-100 for _ in range(5)]]}  # [[x_pos_list], [y_pos_list]]
        self.polar_boundary_sample = {Side.LEFT: [], Side.RIGHT: []}  # [[r1, theta1], [r2, theta2], ...]

    def update_cone_list(self, pp):
        self.polar = []
        for category in Side:
            initial_length = len(self.visible[category])
            self.visible[category] = []
            self.in_fov[category] = []

            for cone in self.list[category]:
                self.polar.append([cone.alpha, cone.dist_car, cone.category])
                if cone.visible:
                    self.visible[category].append(cone)

                if cone.in_fov:
                    self.in_fov[category].append(cone)

            if initial_length != len(self.visible[category]):
                self.new_visible_cone_flag[category] = True
            else:
                self.new_visible_cone_flag[category] = False

        # update boundary sample and polar boundary sample
        if pp.car.auto:
            num_samples = 5
            for category in Side:
                cone_list_x_pos = []
                cone_list_y_pos = []
                if self.new_visible_cone_flag[Side.LEFT] or self.new_visible_cone_flag[Side.RIGHT]:
                    for cone in self.in_fov[category]:
                        cone_list_x_pos.append(cone.position.x)
                        cone_list_y_pos.append(cone.position.y)

                    cone_pos = [cone_list_x_pos, cone_list_y_pos]

                    if len(cone_pos[0]) <= 1:
                        spline_points = [[0 for _ in range(5)], [0 for _ in range(5)]]
                        self.boundary_sample[category] = spline_points

                    elif len(cone_pos[0]) == 2:
                        tck, u = splprep(cone_pos, s=0.25, k=1)

                        start = 0
                        stop = 1.01

                        unew = np.arange(start, stop, 1 / (num_samples - 1))
                        spline_points = splev(unew, tck)
                        self.boundary_sample[category] = spline_points

                    elif len(cone_pos[0]) > 2:
                        tck, u = splprep(cone_pos, s=0.25, k=2)

                        start = 0
                        stop = 1.01

                        unew = np.arange(start, stop, 1 / (num_samples - 1))
                        spline_points = splev(unew, tck)
                        self.boundary_sample[category] = spline_points

                self.polar_boundary_sample[category] = []

                # updating polar_boundary_sample
                for i in range(num_samples):
                    dist = np.linalg.norm(Vector2(self.boundary_sample[category][0][i],
                                                  self.boundary_sample[category][1][i]) - pp.car.position)
                    # print('dist: ', dist)
                    # calculating angle between car angle and sample point (alpha)
                    a_b = Vector2(self.boundary_sample[category][0][i],
                                  self.boundary_sample[category][1][i]) - pp.car.position
                    a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))
                    rotate = np.matrix(
                        [[np.cos(-pp.car.angle * np.pi / 180), -1 * np.sin(-pp.car.angle * np.pi / 180)],
                         [np.sin(-pp.car.angle * np.pi / 180), np.cos(-pp.car.angle * np.pi / 180)]])

                    a_b = rotate * a_b
                    a = a_b[0]
                    b = a_b[1]
                    beta = np.arctan(b / a) * (180 / np.pi)
                    alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
                    angle = alpha[0, 0]

                    self.polar_boundary_sample[category].append([dist, angle])


class Cone:
    def __init__(self, x, y, category, id_number):
        self.true_position = Vector2(x, y)
        self.position = Vector2(x, y)
        self.image = {Side.LEFT: None, Side.RIGHT: None}
        self.visible = False
        self.in_fov = False
        self.category = category
        self.true_dist_car = 10 ** 10
        self.dist_car = 10 ** 10
        self.alpha = 0
        self.id = id_number
        self.cov = Vector2(x, y)

    def update(self, pp):
        # distance to car
        self.true_dist_car = np.linalg.norm(self.true_position - pp.car.true_position)
        self.dist_car = np.linalg.norm(self.position - pp.car.position)

        # calculating angle between car angle and cone (alpha)
        a_b = self.true_position - pp.car.true_position
        a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))
        rotate = np.matrix([[np.cos(-pp.car.true_angle * np.pi / 180), -1 * np.sin(-pp.car.true_angle * np.pi / 180)],
                            [np.sin(-pp.car.true_angle * np.pi / 180), np.cos(-pp.car.true_angle * np.pi / 180)]])
        a_b = rotate * a_b
        a = a_b[0]
        b = a_b[1]
        beta = np.arctan(b / a) * (180 / np.pi)
        alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
        self.alpha = alpha[0, 0]

        # if cone within car fov, set to visible
        if self.dist_car < CAR_FIELD_OF_VIEW / pp.ppu and np.abs(self.alpha) < CAR_FOV_RANGE:
            self.visible = True
            self.in_fov = True
        else:
            self.in_fov = False
