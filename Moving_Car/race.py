import gym
import numpy as np
import time
import pygame
import pp_functions
from pp_functions.reward_function import calculate_reward
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C, DQN
from stable_baselines3.common.evaluation import evaluate_policy

from CarEnv import CarEnv


def dics_to_cont(action):
    if action == 0:
        return 0.5
    elif action == 1:
        return -0.5
    elif action == 2:
        return 1
    elif action == 3:
        return -1
    elif action == 4:
        return 0


if __name__ == "__main__":

    model_name = "PPO"
    time_steps = 500000

    models_dir = f'models/{model_name}-1649940839'

    env = CarEnv(mode="cont")
    check_env(env)

    if model_name == 'PPO' or model_name == 'PPO-cont':
        model = PPO.load(f"{models_dir}/car_model_{time_steps}")
    elif model_name == 'A2C' or model_name == 'A2C-cont':
        model = A2C.load(f"{models_dir}/car_model_{time_steps}")
    elif model_name == 'DQN' or model_name == 'DQN-cont':
        model = DQN.load(f"{models_dir}/car_model_{time_steps}")

    episodes = 1

    for i in range(episodes):
        done = False
        observation = env.reset()
        while not done:

            if env.pp.track_number > 0:
                print(env.pp.cones.visible)
                action, _states = model.predict(observation)
                action = dics_to_cont(action)
                # action = np.interp(env.pp.midpoint_steering_angle(), [-120, 120], [-1, 1])
                action = [action]
            else:
                action = np.interp(env.pp.midpoint_steering_angle(), [-120, 120], [-1, 1])
                action = [action]

            print(action)

            observation, reward, done, info = env.step(action)
            env.render()

            if done:
                print(env.total_reward)

            if env.pp.exit:
                break

    pygame.quit()

