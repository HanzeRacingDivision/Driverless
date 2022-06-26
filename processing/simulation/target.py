from pygame.math import Vector2
import numpy as np

from constants import *


class Targets:
    def __init__(self):
        self.targets = []
        self.target_locations = []
        self.non_passed_targets = []
        self.visible_targets = []
        self.visible_dists = []
        self.closest_target = None
        self.dists = []
        self.non_passed_dists = []

    # initially in pp_functions.utils
    def update_target_lists(self):
        # make list of visible targets and list of passed targets
        self.dists = []
        self.non_passed_dists = []
        self.visible_targets = []
        self.visible_dists = []

        for target in self.targets:
            self.dists.append(target.dist_car)
            self.non_passed_dists.append(target.dist_car)

            if target.passed:
                self.non_passed_dists.remove(target.dist_car)

            if target.visible:
                self.visible_targets.append(target)
                self.visible_dists.append(target.dist_car)

        for target in self.non_passed_targets:
            if target.passed:
                self.non_passed_targets.remove(target)

        if len(self.targets) > 15:
            self.target_locations = self.target_locations[-15:]
            self.targets = self.targets[-15:]

    def reset_targets(self):
        self.non_passed_targets = self.targets.copy()
        for target in self.targets:
            target.passed = False

    def update_closest_target(self):
        # define closest target
        if len(self.visible_targets) > 0:
            if len(self.visible_dists) == 0:
                self.visible_targets = []
            else:
                self.closest_target = self.visible_targets[np.array(self.visible_dists).argmin()]


class Target:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.image = None
        self.passed = False
        self.dist_car = 10 ** 10
        self.alpha = 0
        self.visible = False
        self.time_since_passed = 0

    def update(self, pp):

        # distance to car
        self.dist_car = np.linalg.norm(self.position - pp.car.position)

        # if within 20 pixels of car, target has been 'passed' by the car
        if not self.passed and np.linalg.norm(self.position - pp.car.position) <= 20 / pp.ppu:
            self.passed = True
            self.visible = False

        # calculating angle between car angle and target
        if np.linalg.norm(self.position - pp.car.position) < CAR_FIELD_OF_VIEW / pp.ppu:

            a_b = self.position - pp.car.position  # position difference between car and target
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))  # make vertical matrix and reverse y axis (thijs: idfk why you would want to reverse the Y axis other than a bad fix for pygame's coordinate system)

            rotate = np.matrix(
                [[np.cos(-pp.car.angle * np.pi / 180), -1 * np.sin(-pp.car.angle * np.pi / 180)],
                 [np.sin(-pp.car.angle * np.pi / 180), np.cos(-pp.car.angle * np.pi / 180)]])  # make rotation matrix (google it if you're not faniliar)

            a_b = rotate * a_b  # after this operation, a is the distance to the target and b is the perpendicular distance (shortest distance from the target to a line alligned with the car's orientation)

            a = float(a_b[0][0])  # for reasons beyond my (thijs) comprehension, when requesting data from an np.matrix, you ALWAYS get a matrix back (even if it's a single entry, it returns a 1x1 matrix)
            b = float(a_b[1][0])  # casting to float just makes this a number again

            beta = np.arctan(b / a) * (180 / np.pi)  # calculate angle between car and target
            self.alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)  # finally, calculate the angle between the car and the target, relative to the car orientation
            
            ## thijs note: all the math above is just for calculating alpha (which is the angle between the car and target, relative to the car orientation, with rollover),
            ## but you could also just do that in 4 lines:
            # a_b = self.position - pp.car.position
            # self.alpha = (np.arctan2(pp.car.position[1]-self.position[1], self.position[0]-pp.car.position[0]) * 180 / np.pi) - pp.car.angle  # calculate angle between car and target (weird y axis reversal included) and subtract car angle
            # # self.alpha = thijsSim.GF.degRoll(self.alpha) # a simple universal function for handling rollover to (-180,180) would really be usefull... if only someone made that several years ago... ;)
            # self.alpha = ((self.alpha % -180) if (self.alpha > 180) else ((self.alpha % 180) if (self.alpha < -180) else self.alpha)) # first half of rollover handling
            # self.alpha = ((self.alpha % -360) if (self.alpha < 0) else (self.alpha % 360)) # second half of rollover handling

            # if the target is outside the car fov, it is no longer visible
            if np.abs(self.alpha) < CAR_FOV_RANGE and not self.passed:
                self.visible = True
            else:
                self.visible = False
        else:
            self.visible = False
