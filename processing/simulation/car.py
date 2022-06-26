from math import radians, degrees
from pygame.math import Vector2
import numpy as np
from constants import *
from cone import Side
from pp_functions.utils import bound_angle_180


class Car:
    def __init__(self):
        self.true_position = Vector2(CAR_X_START_POSITION, CAR_Y_START_POSITION)  # ground truth position
        self.position = Vector2(CAR_X_START_POSITION, CAR_Y_START_POSITION)  # perceived position with errors
        self.velocity = Vector2(0.0, 0.0)
        self.angular_velocity = 0
        self.true_angle = 0
        self.angle = 0
        self.acceleration = 0.0
        self.steering_angle = 0.0

        self.car_image = None
        self.crashed = False
        self.breaks = True
        self.auto = True
        self.headlights = False

        self.steering_simulated = 0.0
        self.steering_velocity = 0.0
        self.steering_accel_max = 1850  # (constant) steering acceleration at full motor power (deg/sec^2)
        self.steering_friction = 7.5  # (constant) as rotation speed increases, so does friction
        self.steering_target_margin = 0.1  # (constant) acceptable margin of error for steering control loop
        self.steerSimSubdt = 0.01  # (constant) delta time of steering sub-simulation loop

    def steering(self, pp):
        self.true_angle = bound_angle_180(self.true_angle)  # redefining the car angle so that it is in (-180,180)

        if (len(pp.targets.visible_targets) > 0
                and np.linalg.norm(
                    pp.targets.closest_target.position - self.position) < CAR_FIELD_OF_VIEW / PIXELS_PER_UNIT
                and np.linalg.norm(pp.targets.closest_target.position - self.position) > 20 / PIXELS_PER_UNIT
                and self.auto and not pp.targets.closest_target.passed):
            dist = pp.targets.closest_target.dist_car
            alpha = pp.targets.closest_target.alpha
            self.steering_angle = (MAX_STEERING * 2 / np.pi) * np.arctan(
                alpha / dist ** TURNING_SHARPNESS)
            self.velocity.x = pp.cruising_speed

        self.acceleration = max(-MAX_ACCELERATION, min(self.acceleration, MAX_ACCELERATION))
        self.steering_angle = max(-MAX_STEERING, min(self.steering_angle, MAX_STEERING))

    # Car crash mechanic
    def car_crash_mechanic(self, cone_obj, path_obj, slam_active):
        if len(cone_obj.list[Side.LEFT]) > 0 or len(cone_obj.list[Side.RIGHT]) > 0:
            self.crashed = False

            for category in Side:
                for i in range(len(cone_obj.list[category])):
                    if np.linalg.norm(tuple(x - y for x, y in zip([self.true_position.x, self.true_position.y],
                                                                  [cone_obj.list[category][i].true_position.x,
                                                                   cone_obj.list[category][
                                                                       i].true_position.y]))) < 0.4:
                        self.crashed = True
                        break

                if self.crashed:
                    break

        # checking splines for crash
        if not slam_active:
            for category in Side:
                if not self.crashed and path_obj.splines[category] != 0:
                    for i in range(len(path_obj.splines[category][0])):
                        if np.linalg.norm(tuple(x - y for x, y in zip([self.true_position.x, self.true_position.y],
                                                                      [path_obj.splines[category][0][i],
                                                                       path_obj.splines[category][1][i]]))) < 0.25:
                            self.crashed = True
                            break

    def update(self, dt):
        # doesn't seem like the cleanest use of numpy array addition, also: why is this an array addition and not just
        # 'velocity.x += acc*dt' ?
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(float(-MAX_VELOCITY), min(self.velocity.x, MAX_VELOCITY))

        # simulate steering motor:
        # run a whole-ass sub-simulation just for the steering motor
        for _ in range(int(round(dt / self.steerSimSubdt, 0))):
            steer_abs_diff = abs(self.steering_simulated - self.steering_angle)  # absolute angle difference

            # calculate ratio between time_remaining_if_you_start_braking_now and current_velocity using the area under
            # the curve, 2*dist (2x because I'm thinking square, not triangle). note: doesn't use steering_friction!

            # (see comment above for math) avoid divide by 0 and substitute some high number in case it is 0
            steer_required_decel = ((self.steering_velocity ** 2) / (2 * steer_abs_diff) if (steer_abs_diff > 0.000001)
                                    else self.steering_accel_max)

            if (steer_abs_diff > self.steering_target_margin) or (steer_required_decel > (self.steering_accel_max*.25)):
                steer_accel = (self.steering_accel_max if (self.steering_angle > self.steering_simulated)
                               else -self.steering_accel_max)
                # simulate the microcontroller's decision to start decelerating
                if steer_required_decel > (self.steering_accel_max * 0.75):
                    steer_accel = (-self.steering_accel_max if (self.steering_velocity > 0.0)
                                   else self.steering_accel_max)  # brake steering motor

                # higher steering velocity means more friction
                steer_accel -= self.steering_friction * self.steering_velocity
                # update steering velocity
                self.steering_velocity += self.steerSimSubdt * steer_accel
                # SUVAT applied to rotational distance
                self.steering_simulated += self.steerSimSubdt * self.steering_velocity + ((self.steerSimSubdt ** 2)
                                                                                          / 2.0) * steer_accel
            else:
                break
                # if the steering system has reached its target (and there is no chance of overshoot),
                # stop iterating (saves a little time)
        # TODO simplify formulas for steering motor to approximate it more efficiently...

        # if self.steering_angle:
        # turning_radius = self.length / sin(radians(self.steering_angle))
        if abs(self.steering_simulated) > 0.01:  # fixed bad non-zero check! >:(
            # thijs: i'm pretty sure tan() makes more sense than sin() here, but since the car position is at the center
            # it'll never be perfect
            turning_radius = CAR_LENGTH / np.tan(radians(self.steering_simulated))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0
        # SLAM variables with added noise
        self.angle += degrees(self.angular_velocity) * dt
        self.position += self.velocity.rotate(-self.angle) * dt
        self.position += Vector2(np.random.normal(loc=0, scale=STEERING_NOISE), np.random.normal(loc=0, scale=STEERING_NOISE))
        self.angle += np.random.normal(loc=0, scale=STEERING_NOISE)

        # ground truth
        self.true_angle += degrees(self.angular_velocity) * dt
        self.true_position += self.velocity.rotate(-self.true_angle) * dt
