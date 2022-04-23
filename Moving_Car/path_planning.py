import os
import pygame
import time
import random
import numpy as np

from car import Car
from cone import *
from path import *

import pp_functions.manual_controls
import pp_functions.drawing
from pp_functions.reward_function import calculate_reward


class PathPlanning:
    def __init__(self):
        self.target = Target(-1000, -1000)  # could this be a an empty list instead?
        self.car = Car(7, 10)
        self.cone = Cone(-1000, -1000, Side.LEFT, 0)  # could this be a an empty list instead?
        self.path = Path()
        self.LEVEL_ID = 'None'
        self.initialize_images()
        self.initialize_map()  # comment this if you want to start with blank sheet map

        pygame.init()
        pygame.display.set_caption("Car")

        self.width = 1280
        self.height = 720
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.fullscreen = False
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False
        self.mouse_pos_list = []
        self.total_reward = 0
        self.cruising_speed = 2
        self.ppu = 32

        self.view_offset = [0, 0]
        self.car_centre = False  # THIS DOESNT WORK YET
        self.prev_view_offset = [0, 0]
        self.moving_view_offset = False
        self.view_offset_mouse_pos_start = [0, 0]
        self.midpoint_created = False
        self.undo_done = False

        self.track = False
        self.track_number = -1
        self.track_number_changed = False
        self.time_start_sim = None

        self.time_running = 0
        self.reward = 0
        self.done = False

        self.episode_num = None
        self.num_steps = 0

        # SLAM variables
        self.slam_active = True
        matrix_size = 110
        self.mu = np.zeros(matrix_size)
        self.mu[0] = self.car.true_position.x
        self.mu[1] = self.car.true_position.y

        self.cov = np.zeros((matrix_size, matrix_size))
        for i in range(3, matrix_size):
            self.cov[i, i] = 10 ** 10

        self.u = np.zeros(3)
        self.Rt = np.array([1e-6, 1e-6, 0])
        self.obs = []
        self.c_prob = []
        self.Qt = np.zeros((2, 2))

    def update_slam_vars(self, visible_left_cones, visible_right_cones, car):
        '''
        This function updates the variables necessary to run SLAM
            - obs
            - c_prob
            - Qt
            - u
        '''
        # updating obs, c_prob, and Qt
        cone_dists = []
        cone_angles = []
        self.obs = []
        self.c_prob = np.ones(len(self.mu))
        for i in range(len(visible_left_cones)):
            observed_car_dist = visible_left_cones[i].true_dist_car + np.random.normal(loc=0, scale=1e-10)
            observed_alpha = visible_left_cones[i].alpha + np.random.normal(loc=0, scale=1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_left_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)

        for i in range(len(visible_right_cones)):
            observed_car_dist = visible_right_cones[i].true_dist_car + np.random.normal(loc=0, scale=1e-10)
            observed_alpha = visible_right_cones[i].alpha + np.random.normal(loc=0, scale=1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_right_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)

        if cone_dists and cone_angles:
            self.Qt[0, 0] = np.std(cone_dists) ** 2
            self.Qt[1, 1] = np.std(cone_angles) ** 2
        self.u = [car.velocity.x, car.angular_velocity, 0]

    def EKF_predict(self, dt):
        """
        The prediction step of the Extended Kalman Filter
        """
        n_landmarks = len(self.mu) - 3

        # This function can possibly be replaced by the information from the car itself,
        # such that we have a new x, y and theta of the vehicle.

        # Define motion model f(mu,u)
        [dtrans, drot1, drot2] = self.u

        motion = np.array([[dt * dtrans * np.cos(radians(self.mu[2]))],  # change in x coordinate
                           [dt * dtrans * -np.sin(radians(self.mu[2]))],  # change in y coordinate
                           [dt * degrees(drot1)]])  # change in theta

        # This matrix is used to apply the new motion results to the mu matrix (such that only the first 3 rows are updated)
        F = np.append(np.eye(3), np.zeros((3, n_landmarks)), axis=1)

        # Define motion model Jacobian
        # (derivative of the calculations for the motion.
        J = np.array([[0, 0, dt * dtrans * -np.sin(radians(self.mu[2]))],
                      [0, 0, dt * dtrans * -np.cos(radians(self.mu[2]))],
                      [0, 0, 0]])

        # create the G matrices (see documentation in the drive)
        G = np.eye(n_landmarks + 3) + F.T.dot(J).dot(F)  # slow line 1

        # Predict new state
        self.mu = self.mu + F.T.dot(motion)[:, 0]

        # Predict new covariance
        R_t = np.zeros((n_landmarks + 3, n_landmarks + 3))
        R_t[0, 0] = self.Rt[0]
        R_t[1, 1] = self.Rt[1]
        R_t[2, 2] = self.Rt[2]
        self.cov = G.dot(self.cov).dot(G.T) + R_t  # slow line 2

        # print('Predicted location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu_bar[0][0], mu_bar[1][0],
        #                                                                               mu_bar[2][0])

    def EKF_update(self, car, left_cones, right_cones):
        """
        The update step of the Extended Kalman Filter
        Threshold for observed before is currently 1e6 (this number has to be high, as the uncertainty
        has been set to a high number for un
        threshold for static landmark is c_prob >= 0.5. Maybe we can assume all landmarks are static
        """

        N = len(self.mu)
        for [r, theta, j] in self.obs:
            j = int(j)
            if self.cov[2 * j + 3, 2 * j + 3] >= 1e6 and self.cov[2 * j + 4, 2 * j + 4] >= 1e6:
                # define landmark estimate as current measurement
                # aka, use the distance and angle to calculate its x and y location on the map.
                self.mu[2 * j + 3] = self.mu[0] + r * np.cos(radians(theta) + radians(self.mu[2]))
                self.mu[2 * j + 4] = self.mu[1] + r * -np.sin(radians(theta) + radians(self.mu[2]))

            # if landmark is static
            if self.c_prob[j] >= 0.5:
                # See documentation for more info here.

                # compute expected observation
                delta = np.array([self.mu[2 * j + 3] - self.mu[0], self.mu[2 * j + 4] - self.mu[1]])

                q = delta.T.dot(delta)

                sq = np.sqrt(q)

                z_theta = np.arctan2(delta[1], delta[0])

                z_hat = np.array([[sq], [z_theta - radians(-self.mu[2])]])
                # z_hat = h() function

                # This matrix is used to apply the calculations only to the relevant landmark and used to transfer
                # the low Jacobian into the high Jacobian.
                F = np.zeros((5, N))
                F[:3, :3] = np.eye(3)
                # x location of cone j
                F[3, 2 * j + 3] = 1
                # y location of cone j
                F[4, 2 * j + 4] = 1

                # Do the partial diff equations and create the low jacobian
                H_z = np.array([[-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
                                [delta[1], -delta[0], -q, -delta[1], delta[0]]], dtype='float')
                # do the dot product of F to create the high Jacobian
                H = 1 / q * H_z.dot(F)

                # calculate Kalman gain
                K = self.cov.dot(H.T).dot(np.linalg.inv(H.dot(self.cov).dot(H.T) + self.Qt))

                # calculate difference between expected and real observation
                z_dif = np.array([[r], [radians(-theta)]]) - z_hat
                # normalize angular component!
                z_dif = (z_dif + np.pi) % (2 * np.pi) - np.pi

                # update state vector and covariance matrix
                self.mu = self.mu + K.dot(z_dif)[:, 0]
                self.cov = (np.eye(N) - K.dot(H)).dot(self.cov)

        # using SLAM to update landmark + car positions
        car.position.x = self.mu[0]
        car.position.y = self.mu[1]
        car.angle = self.mu[2]

        for left_cone in left_cones:
            if self.mu[2 * left_cone.id + 3] != 0:
                left_cone.position.x = self.mu[2 * left_cone.id + 3]
            if self.mu[2 * left_cone.id + 4] != 0:
                left_cone.position.y = self.mu[2 * left_cone.id + 4]

        for right_cone in right_cones:
            if self.mu[2 * right_cone.id + 3] != 0:
                right_cone.position.x = self.mu[2 * right_cone.id + 3]
            if self.mu[2 * right_cone.id + 4] != 0:
                right_cone.position.y = self.mu[2 * right_cone.id + 4]

        # print('Updated location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(self.mu[0], self.mu[1], self.mu[2]))

    def slam(self, car, visible_left_cones, visible_right_cones, left_cones, right_cones, dt):
        self.update_slam_vars(visible_left_cones, visible_right_cones, car)
        self.EKF_predict(dt)
        self.EKF_update(car, left_cones, right_cones)

    def initialize_images(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "images/car_r_30.png")
        self.car.car_image = pygame.image.load(image_path)

        # image_path7 = os.path.join(current_dir, "explosion_image.png")
        # explosion_image = pygame.image.load(image_path7)

        image_path1 = os.path.join(current_dir, "images/target_r_t.png")
        self.target.image = pygame.image.load(image_path1)

        image_path3 = os.path.join(current_dir, "images/left_cone_s.png")
        self.cone.image[Side.LEFT] = pygame.image.load(image_path3)

        image_path4 = os.path.join(current_dir, "images/right_cone_s.png")
        self.cone.image[Side.RIGHT] = pygame.image.load(image_path4)

        image_path5 = os.path.join(current_dir, "images/left_spline_s.png")
        self.path.spline_image[Side.LEFT] = pygame.image.load(image_path5)

        image_path6 = os.path.join(current_dir, "images/right_spline_s.png")
        self.path.spline_image[Side.RIGHT] = pygame.image.load(image_path6)

    def initialize_map(self):
        # random_number = random.randint(1, 7)
        random_number = 8
        self.LEVEL_ID = f"MAP_{random_number}"

        left_cones, right_cones = pp_functions.utils.load_existing_map(self.LEVEL_ID)
        self.cone.cone_list[Side.LEFT] = left_cones
        self.cone.cone_list[Side.RIGHT] = right_cones

    def reset_new_lap(self):
        # reset targets for new lap
        if (len(self.target.targets) > 0
                and len(self.target.non_passed_targets) == 0
                and (self.path.spline_linked[Side.LEFT] or self.path.spline_linked[Side.RIGHT])
                and self.track):
            self.target.reset_targets()

    def track_logic(self):
        if self.cone.first_visible_cone[Side.LEFT] != 0 and self.cone.first_visible_cone[Side.RIGHT] != 0:

            # Setting the finishing line/point
            if not self.midpoint_created and self.track:
                self.path.start_midpoint_x = np.mean([self.cone.first_visible_cone[Side.LEFT].true_position.x,
                                                      self.cone.first_visible_cone[Side.RIGHT].true_position.x])
                self.path.start_midpoint_y = np.mean([self.cone.first_visible_cone[Side.LEFT].true_position.y,
                                                      self.cone.first_visible_cone[Side.RIGHT].true_position.y])
                self.midpoint_created = True

            # Incrementing lap number by 1
            elif (np.linalg.norm(
                    (self.path.start_midpoint_x, self.path.start_midpoint_y) - self.car.true_position) < 20 / self.ppu
                  and not self.track_number_changed and self.track):
                self.track_number += 1
                lap_reward = True
                self.track_number_changed = True

            # Setting track_number_changed to false when not on finishing line
            elif (np.linalg.norm(
                    (self.path.start_midpoint_x, self.path.start_midpoint_y) - self.car.true_position) > 20 / self.ppu
                  and self.track):
                self.track_number_changed = False

    def steering(self):
        if (len(self.target.visible_targets) > 0
                and np.linalg.norm(
                    self.target.closest_target.true_position - self.car.true_position) < self.car.fov / self.ppu
                and np.linalg.norm(self.target.closest_target.true_position - self.car.true_position) > 20 / self.ppu
                and self.car.auto and not self.target.closest_target.passed):
            dist = self.target.closest_target.true_dist_car
            alpha = self.target.closest_target.alpha
            self.car.steering_angle = (self.car.max_steering * 2 / np.pi) * np.arctan(
                alpha / dist ** self.car.turning_sharpness)
            self.car.velocity.x = self.cruising_speed

        self.car.acceleration = max(-self.car.max_acceleration, min(self.car.acceleration, self.car.max_acceleration))
        self.car.steering_angle = max(-self.car.max_steering, min(self.car.steering_angle, self.car.max_steering))

    def implement_main_logic(self, dt):
        self.car.update(dt)

        for target in self.target.targets:
            target.update(self)

        for category in Side:
            for cone in self.cone.cone_list[category]:
                cone.update(self)

        # When using CarEnv, this is unnecessary (and important to keep off), as it is handled by 'done' var
        # if self.car.crashed:
        # self.exit = True

    def set_done(self, episode_time_running, episode_num, num_steps):
        self.path.compute_boundaries(self)
        self.car.car_crash_mechanic(self.cone, self.path)
        episode_ending = None

        if self.car.crashed:
            self.done = True
            print('car crashed : ' + self.LEVEL_ID)
            episode_ending = ('crash', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        # elif np.linalg.norm((7 * self.ppu, 10 * self.ppu) - self.car.true_position * self.ppu) < 40 and int(
        elif np.linalg.norm(Vector2(7 * self.ppu, 10 * self.ppu) - self.car.true_position * self.ppu) < 40 and int(
                episode_time_running) > 4:
            print("track complete! : " + self.LEVEL_ID)
            self.done = True
            episode_ending = ('success', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        elif int(episode_time_running) > 100:
            print('time limit reached : ' + self.LEVEL_ID)
            self.done = True
            episode_ending = ('time limit', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        else:
            self.done = False
            return False, episode_ending

    def get_observation(self, num_obs):

        observation = np.zeros(num_obs)

        observation[0] = np.interp(self.car.velocity.x, [0, self.car.max_velocity], [-1, 1])
        observation[1] = np.interp(self.car.angle, [-180, 180], [-1, 1])
        # observation[2] = np.interp(self.car.position.x, [0,30], [-1, 1])
        # observation[3] = np.interp(self.car.position.y, [-20,20], [-1, 1])

        for i, cone in enumerate(self.cone.polar_boundary_sample[Side.LEFT]):
            observation[2 + 2 * i] = np.interp(cone[0], [0, self.car.fov / self.ppu], [-1, 1])
            observation[3 + 2 * i] = np.interp(cone[1], [-1 * self.car.fov_range, self.car.fov_range], [-1, 1])

        for i, cone in enumerate(self.cone.polar_boundary_sample[Side.RIGHT]):
            observation[12 + 2 * i] = np.interp(cone[0], [0, self.car.fov / self.ppu], [-1, 1])
            observation[13 + 2 * i] = np.interp(cone[1], [-1 * self.car.fov_range, self.car.fov_range], [-1, 1])

        # for i in range(len(self.cone.boundary_sample[Side.LEFT][0])):
        # observation[2 + 2*i] = np.interp(self.cone.boundary_sample[Side.LEFT][0][i] - self.car.position.x, [-self.car.fov/self.ppu, self.car.fov/self.ppu], [-1, 1])
        # observation[3 + 2*i] = np.interp(self.cone.boundary_sample[Side.LEFT][1][i] - self.car.position.y, [-self.car.fov/self.ppu, self.car.fov/self.ppu], [-1, 1])

        # for i in range(len(self.cone.boundary_sample[Side.RIGHT][0])):
        # observation[12 + 2*i] = np.interp(self.cone.boundary_sample[Side.RIGHT][0][i] - self.car.position.x, [-self.car.fov/self.ppu, self.car.fov/self.ppu], [-1, 1])
        # observation[13 + 2*i] = np.interp(self.cone.boundary_sample[Side.RIGHT][1][i] - self.car.position.y, [-self.car.fov/self.ppu, self.car.fov/self.ppu], [-1, 1])

        return observation

    def run(self, method="autonomous"):

        self.initialize_images()

        if method == "autonomous":
            self.initialize_map()
        else:
            self.car.auto = False

        time_start = time.time()

        while not self.exit and not self.done:

            self.num_steps += 1

            dt = self.clock.get_time() / 500

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True

            if method == "autonomous":
                pp_functions.manual_controls.enable_dragging_screen(self, events)
            else:
                # user inputs
                pp_functions.manual_controls.user_input(self, events, dt)

            # SLAM
            if self.slam_active:
                # print(round(time_running*10, 0))
                self.slam(self.car, self.cone.visible_cone_list[Side.LEFT], self.cone.visible_cone_list[Side.RIGHT],
                          self.cone.cone_list[Side.LEFT], self.cone.cone_list[Side.RIGHT], dt)

            # Defining the time running since simulation started
            self.time_running = time.time() - time_start

            # redefining the car angle so that it is in (-180,180)
            self.car.config_angle()

            # update target list
            self.target.update_target_lists()

            # update cone list
            self.cone.update_cone_list(self)

            # calculate closest target
            self.target.update_closest_target()

            # reset targets for new lap
            self.reset_new_lap()

            # automatic steering
            self.steering()

            # computing boundary estimation
            self.path.compute_boundaries(self)

            # compute midpoint path
            self.path.generate_midpoint_path(self)

            # implement track logic
            self.track_logic()

            # car crash logic
            self.car.car_crash_mechanic(self.cone, self.path)

            # Calculate reward
            # self.reward = calculate_reward(self)

            # checking exit conditions
            self.set_done(self.time_running, self.episode_num, self.num_steps)

            # Logic
            self.implement_main_logic(dt)

            # Drawing
            pp_functions.drawing.render(self)

            self.clock.tick(self.ticks)

        pygame.quit()


if __name__ == '__main__':
    sim = PathPlanning()

    # 2 methods:
    #   1) autonomous: no user inputs, only screen dragging
    #   2) user: old simulation with user inputs
    sim.run(method="autonomous")
