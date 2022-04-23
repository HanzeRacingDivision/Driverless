from math import sin, radians, degrees
from pygame.math import Vector2
import numpy as np

from cone import Side
from pp_functions.utils import bound_angle_180

NOISE = 1e-3  # noise constant for SLAM variables


class Car:
    def __init__(self, x, y, angle=0, length=2, max_steering=80, max_acceleration=4.0):
        self.true_position = Vector2(x, y)  # ground truth
        self.position = Vector2(x, y)  # perceived position with errors
        self.velocity = Vector2(0.0, 0.0)
        self.angular_velocity = 0
        self.true_angle = angle
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 0.5  # 5
        self.brake_deceleration = 4
        self.free_deceleration = 1
        self.car_image = None
        self.crashed = False

        self.acceleration = 0.0
        self.steering_angle = 0.0
        self.fov = 225  # 150
        self.turning_sharpness = 1.8
        self.breaks = True
        self.fov_range = 60
        self.auto = True
        self.headlights = False

    def config_angle(self):
        self.true_angle = bound_angle_180(self.true_angle)

    # def steering(self, pp):
    # if (len(pp.target.visible_targets) > 0
    # and np.linalg.norm(pp.target.closest_target.position-self.position) < self.fov/pp.ppu
    # and np.linalg.norm(pp.target.closest_target.position-self.position) > 20/pp.ppu
    # and self.auto == True
    # and pp.target.closest_target.passed == False):

    # dist = pp.target.closest_target.dist_car
    # alpha = pp.target.closest_target.alpha
    # self.steering_angle = (self.max_steering*2/np.pi)*np.arctan(alpha/dist**self.turning_sharpness)
    # self.velocity.x = pp.cruising_speed

    # self.acceleration = max(-self.max_acceleration, min(self.acceleration, self.max_acceleration))
    # self.steering_angle = max(-self.max_steering, min(self.steering_angle, self.max_steering))

    # Car crash mechanic
    def car_crash_mechanic(self, cone_obj, path_obj):
        if len(cone_obj.cone_list[Side.LEFT]) > 0 or len(cone_obj.cone_list[Side.RIGHT]) > 0:
            self.crashed = False

            for category in Side:
                for i in range(len(cone_obj.cone_list[category])):
                    if np.linalg.norm(tuple(x - y for x, y in zip([self.true_position.x, self.true_position.y],
                                                                  [cone_obj.cone_list[category][i].true_position.x,
                                                                   cone_obj.cone_list[category][
                                                                       i].true_position.y]))) < 0.4:
                        self.crashed = True
                        break

                if self.crashed:
                    break

        # checking splines for crash
        for category in Side:
            if not self.crashed and path_obj.splines[category] != 0:
                for i in range(len(path_obj.splines[category][0])):
                    if np.linalg.norm(tuple(x - y for x, y in zip([self.true_position.x, self.true_position.y],
                                                                  [path_obj.splines[category][0][i],
                                                                   path_obj.splines[category][1][i]]))) < 0.25:
                        self.crashed = True
                        break

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(float(-self.max_velocity), min(self.velocity.x, self.max_velocity))

        if self.steering_angle:
            turning_radius = self.length / sin(radians(self.steering_angle))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        # SLAM variables
        self.angle += degrees(self.angular_velocity) * dt
        self.position += self.velocity.rotate(-self.angle) * dt
        # adding noise
        self.position += Vector2(np.random.normal(loc=0, scale=NOISE), np.random.normal(loc=0, scale=NOISE))
        self.angle += np.random.normal(loc=0, scale=NOISE)

        # ground truth
        self.true_angle += degrees(self.angular_velocity) * dt
        self.true_position += self.velocity.rotate(-self.true_angle) * dt
