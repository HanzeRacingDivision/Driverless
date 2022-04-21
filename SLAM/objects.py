from math import sin, radians, degrees
from pygame.math import Vector2
import numpy as np
np.seterr(divide='ignore', invalid='ignore')  # clears division by 0 warnings


class Car:
    def __init__(self, x, y, angle=0, length=2, max_steering=80, max_acceleration=4.0):
        self.position = Vector2(x, y)
        self.true_position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.true_angle = angle

        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 5
        self.brake_deceleration = 4
        self.free_deceleration = 1
        self.angular_velocity = 0

        self.acceleration = 0.0
        self.steering = 0.0
        self.fov = 175  # 150
        self.turning_sharpness = 1.8
        self.breaks = True
        self.fov_range = 60
        self.auto = False
        self.slam = True
        self.headlights = False

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(float(-self.max_velocity), min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        # updating true position and angle
        self.true_position += self.velocity.rotate(-self.true_angle) * dt
        self.true_angle += degrees(self.angular_velocity) * dt

        # updating estimated position and angle
        if self.slam:
            self.position += self.velocity.rotate(-self.angle) * dt
            self.angle += degrees(self.angular_velocity) * dt

            # adding some noise/sensor error to the estimated position and angle
        self.position += (np.random.normal(loc=0, scale=1e-10), np.random.normal(loc=0, scale=1e-10))
        self.angle += np.random.normal(loc=0, scale=1e-10)


class Target:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.passed = False
        self.dist_car = 10 ** 10
        self.alpha = 0
        self.visible = False

    def update(self, car, ppu, car_angle):

        # distance to car
        self.dist_car = np.linalg.norm(self.position - car.position)

        # if within 20 pixels of car, target has been 'passed' by the car
        if not self.passed and np.linalg.norm(self.position - car.position) <= 20 / ppu:
            self.passed = True
            self.visible = False

        # calculating angle between car angle and target
        if np.linalg.norm(self.position - car.position) < car.fov / ppu:

            a_b = self.position - car.position
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))

            rotate = np.matrix([[np.cos(-car_angle * np.pi / 180), -1 * np.sin(-car_angle * np.pi / 180)],
                                [np.sin(-car_angle * np.pi / 180), np.cos(-car_angle * np.pi / 180)]])

            a_b = rotate * a_b

            a = a_b[0]
            b = a_b[1]

            beta = np.arctan(b / a) * (180 / np.pi)
            alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
            self.alpha = alpha[0, 0]

            # if the target is outside the car fov, it is no longer visible
            if np.abs(self.alpha) < car.fov_range and not self.passed:
                self.visible = True
            else:
                self.visible = False
        else:
            self.visible = False


class Cone:
    def __init__(self, x, y, category, left_cones, right_cones):
        self.position = Vector2(x + np.random.normal(loc=0, scale=1e-10), y + np.random.normal(loc=0, scale=1e-10))
        self.true_position = Vector2(x, y)
        self.visible = False
        self.in_fov = False
        self.category = category
        self.dist_car = 10 ** 10
        self.id = len(left_cones) + len(right_cones) + 2

    def update(self, car, time_running, ppu, car_angle):

        # distance to car
        self.dist_car = np.linalg.norm(self.position - car.position)
        self.true_dist_car = np.linalg.norm(self.true_position - car.true_position)

        # calculating angle between car angle and cone
        if np.linalg.norm(self.true_position - car.true_position) < car.fov / ppu:

            a_b = self.true_position - car.true_position
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))

            rotate = np.matrix([[np.cos(-car_angle * np.pi / 180), -1 * np.sin(-car_angle * np.pi / 180)],
                                [np.sin(-car_angle * np.pi / 180), np.cos(-car_angle * np.pi / 180)]])

            a_b = rotate * a_b

            a = a_b[0]
            b = a_b[1]

            beta = np.arctan(b / a) * (180 / np.pi)
            alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
            self.alpha = alpha[0, 0]

            # if cone within car fov, set to visible
            if np.abs(self.alpha) < car.fov_range:
                self.visible = True
                self.in_fov = True
            else:
                pass
                self.visible = False  # commenting this line allows cones to be remembered
                self.in_fov = False

        else:
            pass
            self.visible = False  # commenting this line allows cones to be remembered
            self.in_fov = False
