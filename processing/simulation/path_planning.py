import os
import pygame
import math

from car import Car
from cone import *
from target import *
from slam import *
from clock import *
from constants import *

import pp_functions
import pp_functions.manual_controls
import pp_functions.drawing


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return Vector2(x, y)



class PathPlanning:
    def __init__(self):
        self.targets = Targets()
        self.car = Car()
        self.cones = Cones()
        self.path = Path()
        self.clock: Clock = Clock()

        self.slam = Slam(self.car)
        self.slam_active = SLAM_ACTIVATED

        self.LEVEL_ID = 'None'

        if not BLANK_MAP:
            self.initialize_map()

        pygame.init()
        pygame.display.set_caption("Car")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.initialize_images()
        self.fullscreen = False
        self.exit = False
        self.mouse_pos_list = []
        self.total_reward = 0
        self.ppu = PIXELS_PER_UNIT
        self.view_offset = [0, 0]
        self.car_centre = False  # THIS DOESNT WORK YET
        self.prev_view_offset = [0, 0]
        self.moving_view_offset = False
        self.view_offset_mouse_pos_start = [0, 0]
        self.midpoint_created = False
        self.undo_done = False
        self.track = True
        self.track_number = 0
        self.track_number_changed = False
        self.time_start_sim = None
        self.episode_time_running = 0  # THIS IS A FAKE VARIABLE!!!
        self.reward = 0
        self.done = False
        self.episode_num = None
        self.num_steps = 0
        self.cruising_speed = 1
        self.last_time_lap_changed = self.clock.time_running

    def initialize_images(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "images/car_r_30.png")

        car_image = pygame.image.load(image_path)
        self.car.car_image = pygame.transform.smoothscale(car_image.convert_alpha(), (CAR_LENGTH*PIXELS_PER_UNIT, CAR_WIDTH*PIXELS_PER_UNIT))


        # image_path7 = os.path.join(current_dir, "explosion_image.png")
        # explosion_image = pygame.image.load(image_path7)

        image_path1 = os.path.join(current_dir, "images/target_r_t.png")
        self.targets.image = pygame.image.load(image_path1)

        image_path3 = os.path.join(current_dir, "images/left_cone_s.png")
        self.cones.image[Side.LEFT] = pygame.image.load(image_path3)

        image_path4 = os.path.join(current_dir, "images/right_cone_s.png")
        self.cones.image[Side.RIGHT] = pygame.image.load(image_path4)

        image_path5 = os.path.join(current_dir, "images/left_spline_s.png")
        self.path.spline_image[Side.LEFT] = pygame.image.load(image_path5)

        image_path6 = os.path.join(current_dir, "images/right_spline_s.png")
        self.path.spline_image[Side.RIGHT] = pygame.image.load(image_path6)

    def initialize_map(self):
        # random_number = random.randint(1, 7)
        random_number = "simple"
        self.LEVEL_ID = f"MAP_{random_number}"

        left_cones, right_cones = pp_functions.utils.load_existing_map(self.LEVEL_ID)
        self.cones.list[Side.LEFT] = left_cones
        self.cones.list[Side.RIGHT] = right_cones

    def reset_new_lap(self):
        # reset targets for new lap
        if (len(self.targets.targets) > 0
                and len(self.targets.non_passed_targets) == 0
                and (self.path.spline_linked[Side.LEFT] or self.path.spline_linked[Side.RIGHT])
                and self.track):
            self.targets.reset_targets()

    def track_logic(self):
        self.path.compute_boundaries(self)
        if self.cones.first_visible_cone[Side.LEFT] != 0 and self.cones.first_visible_cone[Side.RIGHT] != 0:
            # Setting the finishing line/point
            if not self.midpoint_created and self.track:
                self.path.start_midpoint_x = np.mean([self.cones.first_visible_cone[Side.LEFT].true_position.x,
                                                      self.cones.first_visible_cone[Side.RIGHT].true_position.x])
                self.path.start_midpoint_y = np.mean([self.cones.first_visible_cone[Side.LEFT].true_position.y,
                                                      self.cones.first_visible_cone[Side.RIGHT].true_position.y])
                self.midpoint_created = True

            # Incrementing lap number by 1
            elif (np.linalg.norm(
                    Vector2(self.path.start_midpoint_x,
                            self.path.start_midpoint_y) - self.car.true_position) < 20 / self.ppu
                  and not self.track_number_changed and self.track and (self.clock.time_running - self.last_time_lap_changed > 10)):
                self.track_number += 1
                self.track_number_changed = True
                # self.targets.reset_targets()
                if self.slam_active and MODE == "race":
                    self.slam_active = False

            # Setting track_number_changed to false when not on finishing line
            elif (np.linalg.norm((self.path.start_midpoint_x - self.car.true_position[0],
                                  self.path.start_midpoint_y - self.car.true_position[1])) > 20 / self.ppu
                  and self.track):
                self.track_number_changed = False

    def implement_main_logic(self):
        self.car.update(self.clock.dt)

        for target in self.targets.targets:
            target.update(self)

        for category in Side:
            for cone in self.cones.list[category]:
                cone.update(self)

    def set_done(self, episode_time_running, episode_num, num_steps):
        self.path.compute_boundaries(self)
        self.car.car_crash_mechanic(self.cones, self.path, self.slam_active)
        episode_ending = None

        if self.car.crashed:
            self.done = True
            print('car crashed : ' + self.LEVEL_ID)
            episode_ending = ('crash', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        elif np.linalg.norm(Vector2(CAR_X_START_POSITION * PIXELS_PER_UNIT, CAR_Y_START_POSITION * PIXELS_PER_UNIT) - self.car.true_position * self.ppu) < 40 and int(
                episode_time_running) > 4:
            print("track complete! : " + self.LEVEL_ID)
            self.done = True
            self.track_number += 1
            episode_ending = ('success', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        elif int(episode_time_running) > EPISODE_TIME_LIMIT:
            print('time limit reached : ' + self.LEVEL_ID)
            self.done = True
            episode_ending = ('time limit', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        else:
            self.done = False
            return False, episode_ending

    def midpoint_steering_angle(self):
        if (len(self.targets.visible_targets) > 0
                and CAR_FIELD_OF_VIEW / PIXELS_PER_UNIT > np.linalg.norm(
                    self.targets.closest_target.position - self.car.position) > 20 / PIXELS_PER_UNIT
                and self.car.auto and not self.targets.closest_target.passed):

            dist = self.targets.closest_target.dist_car
            alpha = self.targets.closest_target.alpha
            midpoint_steering_angle = (MAX_STEERING * 2 / np.pi) * np.arctan(alpha / dist ** TURNING_SHARPNESS)
        else:
            midpoint_steering_angle = 0

        midpoint_steering_angle = max(-MAX_STEERING, min(midpoint_steering_angle, MAX_STEERING))

        return midpoint_steering_angle

    def get_observation(self, num_obs: int, noise_scale: float = 0) -> np.ndarray:
        observation = np.zeros(num_obs, dtype=np.float32)
        observation[0] = np.interp(self.car.velocity.x, [0, MAX_VELOCITY], [-1, 1])
        observation[1] = np.interp(self.car.angle, [-180, 180], [-1, 1])

        for i, cone in enumerate(self.cones.polar_boundary_sample[Side.LEFT]):
            observation[2 + 2 * i] = np.interp(cone[0], [0, CAR_FIELD_OF_VIEW / PIXELS_PER_UNIT], [-1, 1])
            observation[3 + 2 * i] = np.interp(cone[1], [-1 * CAR_FOV_RANGE, CAR_FOV_RANGE], [-1, 1])

        for i, cone in enumerate(self.cones.polar_boundary_sample[Side.RIGHT]):
            observation[12 + 2 * i] = np.interp(cone[0], [0, CAR_FIELD_OF_VIEW / PIXELS_PER_UNIT], [-1, 1])
            observation[13 + 2 * i] = np.interp(cone[1], [-1 * CAR_FOV_RANGE, CAR_FOV_RANGE], [-1, 1])

        noise = np.float32(np.random.normal(1, noise_scale, size=num_obs))  # add noise
        observation = np.clip(np.multiply(observation, noise), -1, 1)
        return observation

    def get_fake_steering_info(self):
        angle = self.car.steering_angle
        velocity = self.car.velocity[0]

        self.car.steering_angle = angle
        self.car.velocity[0] = velocity

    # def get_fake_observation(self):
    #     ANGLE_OFFSET = 0  # in radians
    #     observation = {Side.LEFT: [], Side.RIGHT: []}
    #     for category in Side:
    #         for cone in self.cones.in_fov[category]:
    #             r = cone.true_dist_car + np.random.normal(0, SLAM_NOISE)
    #             theta = cone.alpha/180*math.pi + np.random.normal(0, SLAM_NOISE)
    #             observation[category].append([r, theta, cone.position])
    #
    #     return observation
    #
    # def match_observation_to_get_ids(self, observation):
    #     observation_with_ids = {Side.LEFT: [], Side.RIGHT: []}
    #     for category in Side:
    #         for cone in self.cones.perceived[category]:
    #             for [r, theta, position] in (observation[category]):
    #                 distance_sqr = cone.dist_car**2 + r**2 - 2*r*cone.dist_car*math.cos(cone.alpha*math.pi/180 - theta)
    #                 if distance_sqr < DISTANCE_TO_MATCH:
    #                     # TODO: Use new information, not only match!
    #                     observation_with_ids[category].append(cone)
    #                     observation[category].remove([r, theta, position])
    #
    #     return observation_with_ids, observation
    #
    # def make_new_cones(self, observation):
    #     current_highest_id = len(self.cones.perceived[Side.RIGHT]) + len(self.cones.perceived[Side.LEFT])
    #     observation_with_ids = {Side.LEFT: [], Side.RIGHT: []}
    #     for category in Side:
    #         for [r, theta, position] in observation[category]:
    #             current_highest_id += 1
    #             x_y_in_dir_of_car = pol2cart(r, theta)
    #             rotate = np.matrix(
    #                 [[np.cos(-self.car.angle * np.pi / 180), -1 * np.sin(-self.car.angle * np.pi / 180)],
    #                  [np.sin(-self.car.angle * np.pi / 180), np.cos(-self.car.angle * np.pi / 180)]])
    #
    #             a_b = np.transpose(np.matrix([x_y_in_dir_of_car.x, -1 * x_y_in_dir_of_car.y]))
    #             x, y = rotate * a_b
    #             car_x, car_y = self.car.position
    #             x += car_x
    #             y += car_y
    #             new_cone = Cone(x, y, category, current_highest_id)
    #             new_cone.alpha = theta/math.pi*180
    #             new_cone.dist_car = r
    #             observation_with_ids[category].append(new_cone)

    #    return observation_with_ids

    def get_fake_observation(self):
        observation = {Side.LEFT: [], Side.RIGHT: []}
        for category in Side:
            for cone in self.cones.in_fov[category]:
                x = cone.position.x + np.random.normal(0, SLAM_NOISE)
                y = cone.position.y + np.random.normal(0, SLAM_NOISE)
                observation[category].append([x, y])

        return observation

    def match_observation_to_get_ids(self, observation):
        observation_with_ids = {Side.LEFT: [], Side.RIGHT: []}
        for category in Side:
            for cone in self.cones.perceived[category]:
                for [x, y] in (observation[category]):
                    distance_sqr = (cone.position.x - x)**2 + (cone.position.y - y)**2
                    if distance_sqr < DISTANCE_TO_MATCH:
                        # TODO: Use new information, not only match!
                        observation_with_ids[category].append(cone)
                        observation[category].remove([x, y])

        return observation_with_ids, observation

    def make_new_cones(self, observation):
        current_highest_id = len(self.cones.perceived[Side.RIGHT]) + len(self.cones.perceived[Side.LEFT])
        observation_with_ids = {Side.LEFT: [], Side.RIGHT: []}
        for category in Side:
            for [x, y] in observation[category]:
                current_highest_id += 1
                new_cone = Cone(x, y, category, current_highest_id)
                observation_with_ids[category].append(new_cone)

        return observation_with_ids

    def run(self, method):

        self.initialize_images()

        if method == "autonomous":
            self.initialize_map()
        else:
            self.car.auto = False

        while not self.exit and not self.done:

            self.num_steps += 1
            self.clock.update()
            self.episode_time_running = self.clock.time_running  # I HAVE NO CLUE IF THIS MAKES ANY SENSE

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True

            if method == "autonomous":
                pp_functions.manual_controls.enable_dragging_screen(self, events)
            else:  # user inputs
                pp_functions.manual_controls.user_input(self, events, self.clock.dt)

            # update target list
            self.targets.update_target_lists()

            # update cone list
            self.cones.update_cone_list(self)

            # SLAM
            if self.slam_active:
                self.slam.update_slam_vars(self.cones.visible[Side.LEFT], self.cones.visible[Side.RIGHT], self.car)
                self.slam.EKF_predict(self.clock.dt)
                if self.num_steps % SLAM_FRAME_LIMIT == 0 or self.num_steps < 5:
                    self.slam.EKF_update(self.car, self.cones.visible)

            # calculate closest target
            self.targets.update_closest_target()

            # computing boundary estimation
            self.path.compute_boundaries(self)

            # compute midpoint path
            self.path.generate_midpoint_path(self)

            # reset targets for new lap
            self.reset_new_lap()

            # automatic steering
            self.car.steering(self)

            # implement track logic
            self.track_logic()

            # car crash logic
            self.car.car_crash_mechanic(self.cones, self.path, self.slam_active)

            # checking exit conditions
            self.set_done(self.clock.time_running, self.episode_num, self.num_steps)

            # Logic
            self.implement_main_logic()

            # Drawing
            pp_functions.drawing.render(self)

            observation = self.get_fake_observation()
            obs_with_ids, remainder = self.match_observation_to_get_ids(observation)
            new = self.make_new_cones(remainder)
            print("NEW LINE:")
            print("obs:", observation)
            print("matched:", obs_with_ids)
            if len(new[Side.LEFT])+len(new[Side.RIGHT]) != 0:
                print("found:", new)
            for category in Side:
                self.cones.perceived[category] += new[category]
            print("perceived:", self.cones.perceived)

            #self.get_fake_steering_info()

        pygame.quit()


if __name__ == '__main__':
    sim = PathPlanning()
    sim.run(method=STEERING_METHOD)
