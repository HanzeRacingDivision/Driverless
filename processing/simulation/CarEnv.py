import random

import gym
import numpy as np
import time
from path_planning import PathPlanning
import pygame
import pp_functions
from pp_functions.reward_function import calculate_reward
from stable_baselines3.common.env_checker import check_env

class CarEnv(gym.Env):
    def __init__(self, mode = 'cont'):
        super(CarEnv, self).__init__()

        self.mode = mode
        self.num_envs = 1

        self.episode_num = 0

        self.num_steps = 0
        
        self.pp = PathPlanning(False)

        self.LEVEL_ID = self.pp.LEVEL_ID

        self.episode_time_start = self.pp.clock.get_time_running()
        self.episode_time_running = 0
        self.total_reward = 0

        self.data_logger = {'episode_end' : []}

        if self.mode == "cont":
            self.action_space = gym.spaces.Box(low = -1, high = 1, shape=(1,), dtype=np.float32)
        elif self.mode == "discrete":
            self.action_space = gym.spaces.Discrete(5)

        self.num_obs = 2 + 5 * 2 * 2  # 2 car obs + 5 spline points * 2 (angle + dist) * 2 (left/right sides) (22)

        low = -1 * np.ones(self.num_obs)
        high = np.ones(self.num_obs)

        self.observation_space = gym.spaces.Box(low=low, high=high, shape=(self.num_obs,), dtype=np.float32)

    def generate_random_action(self):
        car_steering_angle = random.uniform(-self.pp.car.max_steering, self.pp.car.max_steering)
        car_curr_velocity = self.pp.cruising_speed
        return [car_steering_angle, car_curr_velocity]

    def render(self, mode=None):
        pp_functions.drawing.render(self.pp)
        
    def step(self, action):

        #for key in self.pp.cone.visible_cone_list.keys(): 
            #print(len(self.pp.cone.visible_cone_list[key]))

        #print(self.num_steps)

        #print(self.pp.car.position.x, self.pp.car.position.y)
        #print(self.episode_time_running)

        self.num_steps += 1

        if self.mode == "cont":
            self.pp.car.steering_angle = self.pp.car.max_steering * action[0]

        elif self.mode == "discrete":
            if action == 0:
                self.pp.car.steering_angle = 0.5 * self.pp.car.max_steering
            elif action == 1:
                self.pp.car.steering_angle = 0.5 * -1 * self.pp.car.max_steering
            elif action == 2:
                self.pp.car.steering_angle = 1 * self.pp.car.max_steering
            elif action == 3:
                self.pp.car.steering_angle = 1 * -1 * self.pp.car.max_steering
            elif action == 4:
                self.pp.car.steering_angle = 0 * self.pp.car.max_steering

        self.pp.num_steps += 1
        self.pp.clock.update()
        dt = self.pp.clock.get_dt()

        # Event queue
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                self.pp.exit = True

        self.pp.car.velocity.x = 1

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                self.pp.exit = True
        pp_functions.manual_controls.enable_dragging_screen(self.pp, events)

        self.episode_time_running = self.pp.clock.get_time_running() - self.episode_time_start

        self.pp.car.config_angle()

        # update target list
        self.pp.targets.update_target_lists()

        # update cone list
        self.pp.cones.update_cone_list(self.pp)

        # calculate closest target
        self.pp.targets.update_closest_target()

        self.pp.path.generate_midpoint_path(self.pp)

        # reset targets for new lap
        self.pp.reset_new_lap()

        # implement track logic
        self.pp.track_logic()

        # Logic
        self.pp.implement_main_logic()

        # Retrieve observation
        observation = self.pp.get_observation(self.num_obs)

        done, episode_end = self.pp.set_done(self.episode_time_running, self.episode_num, self.num_steps)
        if episode_end:
            self.data_logger['episode_end'].append(episode_end)
            self.pp.track_number = 1

        reward = calculate_reward(self)
        self.pp.reward = reward # this is only for drawing purposes
        self.total_reward += reward

        info = {}

        return observation, reward, done, info

    def reset(self):
        self.pp = PathPlanning(False)

        self.total_reward = 0
        self.num_steps = 0

        self.episode_num += 1
        
        self.episode_time_start = self.pp.clock.get_time_running()

        observation = np.zeros(self.num_obs, dtype=np.float32)
        return observation

    def deactivate_slam(self):
        self.pp.slam_active = False

    def change_mode(self, new_mode):
        self.mode = new_mode

        if self.mode == "cont":
            self.action_space = gym.spaces.Box(low = -1, high = 1, shape=(1,), dtype=np.float32)
        elif self.mode == "discrete":
            self.action_space = gym.spaces.Discrete(5)


if __name__ == "__main__":
    env = CarEnv()
    observation = env.reset()
    check_env(env)
    done = False

    while not done:
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        env.render()

        if done:
            print(env.total_reward)

        if env.pp.exit:
            break

    pygame.quit()