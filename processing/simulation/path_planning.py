import os
import pygame

from car import Car
from cone import *
from target import *
from slam import *
from clock import *

import pp_functions
import pp_functions.manual_controls
import pp_functions.drawing


class PathPlanning:
    def __init__(self, slam_active):
        self.targets = Targets()
        self.car = Car(7, 10)
        self.cones = Cones()
        self.path = Path()
        self.clock = Clock()
        self.LEVEL_ID = 'None'
        self.initialize_images()
        self.initialize_map()  # comment this if you want to start with blank sheet map

        pygame.init()
        pygame.display.set_caption("Car")

        self.width = 1280
        self.height = 720
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.fullscreen = False
        self.ticks = 60
        self.exit = False
        self.mouse_pos_list = []
        self.total_reward = 0
        self.cruising_speed = 1
        self.ppu = 32

        self.view_offset = [0, 0]
        self.car_centre = False  # THIS DOESNT WORK YET
        self.prev_view_offset = [0, 0]
        self.moving_view_offset = False
        self.view_offset_mouse_pos_start = [0, 0]
        self.midpoint_created = False
        self.undo_done = False

        self.track = False
        self.track_number = 0
        self.track_number_changed = False
        self.time_start_sim = None

        self.episode_time_running = 0  # THIS IS A FAKE VARIABLE!!!
        self.reward = 0
        self.done = False

        self.episode_num = None
        self.num_steps = 0

        # SLAM variables
        self.slam = Slam(250, self.car)
        self.slam_active = slam_active

    def initialize_images(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "images/car_r_30.png")
        self.car.car_image = pygame.image.load(image_path)

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
        random_number = "simple"  # random.randint(1, 7)
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
                  and not self.track_number_changed and self.track):
                self.track_number += 1
                self.track_number_changed = True

            # Setting track_number_changed to false when not on finishing line
            elif (np.linalg.norm((self.path.start_midpoint_x - self.car.true_position[0],
                                  self.path.start_midpoint_y - self.car.true_position[1])) > 20 / self.ppu
                  and self.track):
                self.track_number_changed = False

    def steering(self):
        if (len(self.targets.visible_targets) > 0
                and np.linalg.norm(
                    self.targets.closest_target.position - self.car.position) < self.car.fov / self.ppu
                and np.linalg.norm(self.targets.closest_target.position - self.car.position) > 20 / self.ppu
                and self.car.auto and not self.targets.closest_target.passed):
            dist = self.targets.closest_target.dist_car
            alpha = self.targets.closest_target.alpha
            self.car.steering_angle = (self.car.max_steering * 2 / np.pi) * np.arctan(
                alpha / dist ** self.car.turning_sharpness)
            self.car.velocity.x = self.cruising_speed

        self.car.acceleration = max(-self.car.max_acceleration, min(self.car.acceleration, self.car.max_acceleration))
        self.car.steering_angle = max(-self.car.max_steering, min(self.car.steering_angle, self.car.max_steering))

    def implement_main_logic(self):
        self.car.update(self.clock.get_dt())

        for target in self.targets.targets:
            target.update(self)

        for category in Side:
            for cone in self.cones.list[category]:
                cone.update(self)

        # When using CarEnv, this is unnecessary (and important to keep off), as it is handled by 'done' var
        # if self.car.crashed:
        # self.exit = True

    def set_done(self, episode_time_running, episode_num, num_steps):
        self.path.compute_boundaries(self)
        self.car.car_crash_mechanic(self.cones, self.path, self.slam_active)
        episode_ending = None

        if self.car.crashed:
            self.done = True
            print('car crashed : ' + self.LEVEL_ID)
            episode_ending = ('crash', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        elif np.linalg.norm(Vector2(7 * self.ppu, 10 * self.ppu) - self.car.true_position * self.ppu) < 40 and int(
                episode_time_running) > 4:
            print("track complete! : " + self.LEVEL_ID)
            # self.done = True
            self.track_number += 1
            episode_ending = ('success', self.LEVEL_ID, episode_num, num_steps)
            # return True, episode_ending
            return False, episode_ending

        elif int(episode_time_running) > 100:
            print('time limit reached : ' + self.LEVEL_ID)
            self.done = True
            episode_ending = ('time limit', self.LEVEL_ID, episode_num, num_steps)
            return True, episode_ending

        else:
            self.done = False
            return False, episode_ending

    def midpoint_steering_angle(self):
        if (len(self.targets.visible_targets) > 0
                and self.car.fov / self.ppu > np.linalg.norm(
                    self.targets.closest_target.position - self.car.position) > 20 / self.ppu
                and self.car.auto
                and self.targets.closest_target.passed == False):

            dist = self.targets.closest_target.dist_car
            alpha = self.targets.closest_target.alpha
            midpoint_steering_angle = (self.car.max_steering * 2 / np.pi) * np.arctan(
                alpha / dist ** self.car.turning_sharpness)
        else:
            midpoint_steering_angle = 0

        midpoint_steering_angle = max(-self.car.max_steering, min(midpoint_steering_angle, self.car.max_steering))

        return midpoint_steering_angle

    def get_observation(self, num_obs):
        observation = np.zeros(num_obs, dtype=np.float32)
        observation[0] = np.interp(self.car.velocity.x, [0, self.car.max_velocity], [-1, 1])
        observation[1] = np.interp(self.car.angle, [-180, 180], [-1, 1])

        for i, cone in enumerate(self.cones.polar_boundary_sample[Side.LEFT]):
            observation[2 + 2 * i] = np.interp(cone[0], [0, self.car.fov / self.ppu], [-1, 1])
            observation[3 + 2 * i] = np.interp(cone[1], [-1 * self.car.fov_range, self.car.fov_range], [-1, 1])

        for i, cone in enumerate(self.cones.polar_boundary_sample[Side.RIGHT]):
            observation[12 + 2 * i] = np.interp(cone[0], [0, self.car.fov / self.ppu], [-1, 1])
            observation[13 + 2 * i] = np.interp(cone[1], [-1 * self.car.fov_range, self.car.fov_range], [-1, 1])

        return observation

    def run(self, method):

        self.initialize_images()

        if method == "autonomous":
            self.initialize_map()
        else:
            self.car.auto = False

        while not self.exit and not self.done:

            self.num_steps += 1
            # Time variables
            self.clock.update()

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True

            if method == "autonomous":
                pp_functions.manual_controls.enable_dragging_screen(self, events)
            else:
                # user inputs
                pp_functions.manual_controls.user_input(self, events, self.clock.get_dt())

            self.episode_time_running = self.clock.get_time_running()  # I HAVE NO CLUE IF THIS MAKES ANY SENSE

            # redefining the car angle so that it is in (-180,180)
            self.car.config_angle()

            # update target list
            self.targets.update_target_lists()

            # update cone list
            self.cones.update_cone_list(self)

            # SLAM
            if self.slam_active:
                self.slam.update_slam_vars(self.cones.visible[Side.LEFT], self.cones.visible[Side.RIGHT], self.car)
                self.slam.EKF_predict(self.clock.get_dt())
                if self.num_steps % self.slam.frame_limit == 0 or self.num_steps < 5:
                    self.slam.EKF_update(self.car, self.cones.list)

                    # calculate closest target
                    self.targets.update_closest_target()

                    # computing boundary estimation
                    self.path.compute_boundaries(self)

                    # compute midpoint path
                    self.path.generate_midpoint_path(self)

            # reset targets for new lap
            self.reset_new_lap()

            # automatic steering
            self.steering()

            # implement track logic
            self.track_logic()

            # car crash logic
            self.car.car_crash_mechanic(self.cones, self.path, self.slam_active)

            # checking exit conditions
            self.set_done(self.clock.get_time_running(), self.episode_num, self.num_steps)

            # Logic
            self.implement_main_logic()

            # Drawing
            pp_functions.drawing.render(self)

        pygame.quit()


if __name__ == '__main__':
    sim = PathPlanning(slam_active=True)

    # 2 steering methods:
    #   1) autonomous: no user inputs, only screen dragging
    #   2) user: old simulation with user inputs
    # SLAM activated True/False
    sim.run(method="autonomous")
