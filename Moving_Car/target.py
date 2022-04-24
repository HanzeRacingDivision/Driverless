from pygame.math import Vector2
import numpy as np


class Target:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.image = None
        self.passed = False
        self.dist_car = 10 ** 10
        self.alpha = 0
        self.visible = False

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
        # print(len(self.non_passed_targets))

    def update(self, pp):

        # distance to car
        self.dist_car = np.linalg.norm(self.position - pp.car.position)

        # if within 20 pixels of car, target has been 'passed' by the car
        if not self.passed and np.linalg.norm(self.position - pp.car.position) <= 20 / pp.ppu:
            self.passed = True
            self.visible = False

        # calculating angle between car angle and target
        if np.linalg.norm(self.position - pp.car.position) < pp.car.fov / pp.ppu:

            a_b = self.position - pp.car.position
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))

            rotate = np.matrix(
                [[np.cos(-pp.car.true_angle * np.pi / 180), -1 * np.sin(-pp.car.true_angle * np.pi / 180)],
                 [np.sin(-pp.car.true_angle * np.pi / 180), np.cos(-pp.car.true_angle * np.pi / 180)]])

            a_b = rotate * a_b

            a = a_b[0]
            b = a_b[1]

            beta = np.arctan(b / a) * (180 / np.pi)
            alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
            self.alpha = alpha[0, 0]

            # if the target is outside the car fov, it is no longer visible
            if np.abs(self.alpha) < pp.car.fov_range and not self.passed:
                self.visible = True
            else:
                self.visible = False
        else:
            self.visible = False

    def update_closest_target(self):
        # define closest target
        if len(self.visible_targets) > 0:
            if len(self.visible_dists) == 0:
                self.visible_targets = []
            else:
                # prev = self.closest_target
                self.closest_target = self.visible_targets[np.array(self.visible_dists).argmin()]
                # if self.closest_target != prev:
                #     print('closest target changed')

                # print(len(self.visible_targets))
                # print(len(self.targets))

    def reset_targets(self):
        self.non_passed_targets = self.targets.copy()
        for target in self.targets:
            target.passed = False
