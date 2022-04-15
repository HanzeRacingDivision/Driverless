from ray import tune
from CarEnv import CarEnv
import gym
from stable_baselines3 import PPO, A2C, DQN
from stable_baselines3.common.env_util import make_vec_env
import os
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecEnvWrapper
import time
import numpy as np
import os.path as osp
import json
import csv
from collections import deque
from stable_baselines3.common.vec_env import VecMonitor

def linear_schedule(initial_value):
    """
    Linear learning rate schedule.
    :param initial_value: (float or str)
    :return: (function)
    """
    if isinstance(initial_value, str):
        initial_value = float(initial_value)

    def func(progress):
        """
        Progress will decrease from 1 (beginning) to 0
        :param progress: (float)
        :return: (float)
        """
        return progress * initial_value

    return func


if __name__ == "__main__":

    model_name = "PPO-cont"

    models_dir = f'models/{model_name}-{int(time.time())}'
    log_dir = f'logs/{model_name}-{int(time.time())}'

    if not os.path.exists(models_dir):
      os.makedirs(models_dir)

    if not os.path.exists(log_dir):
      os.makedirs(log_dir)

    env = CarEnv(mode = "cont")
    env = Monitor(env, log_dir)

    if model_name == 'PPO' or model_name == 'PPO-cont':
        model = PPO("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log = log_dir)
    elif model_name == 'A2C' or model_name == 'A2C-cont':
        model = A2C("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log = log_dir)
    elif model_name == 'DQN' or model_name == 'DQN-cont':
        model = DQN("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log = log_dir)

    timesteps = 10000
    for i in range(100):
      model.learn(total_timesteps = timesteps, reset_num_timesteps = False, tb_log_name = f"{model_name}")
      print(f"Saving model: {models_dir}/car_model_{timesteps * i}")
      model.save(f"{models_dir}/car_model_{timesteps * i}")

    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"mean_reward:{mean_reward:.2f} +/- {std_reward:.2f}")

    observation = env.reset()
    done = False
    while not done:
        action, _states = model.predict(observation)
        observation, reward, done, info = env.step(action)
        env.render()