import numpy as np
import pygame
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C, DQN
from constants import *

from CarEnv import CarEnv


def disc_to_cont(action):
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
    models_dir = f'models/{MODEL_NAME}-{MODEL_NUMBER}'
    env = CarEnv(mode="cont")
    check_env(env)

    if MODEL_NAME == 'PPO' or MODEL_NAME == 'PPO-cont':
        model = PPO.load(f"{models_dir}/car_model_{TIME_STEPS}")
    elif MODEL_NAME == 'A2C' or MODEL_NAME == 'A2C-cont':
        model = A2C.load(f"{models_dir}/car_model_{TIME_STEPS}")
    elif MODEL_NAME == 'DQN' or MODEL_NAME == 'DQN-cont':
        model = DQN.load(f"{models_dir}/car_model_{TIME_STEPS}")
    else:
        raise ValueError(f"{MODEL_NAME} not recognized as valid model name")

    episodes = 1

    for i in range(episodes):
        done = False
        observation = env.reset()
        while not done:

            if env.pp.track_number > 0:
                action, _states = model.predict(observation)
                if CONVERSION == "disc_to_cont":
                    action = disc_to_cont(action)
                action = [action]
                print("agent:", action)
            else:
                angle = env.pp.midpoint_steering_angle()
                action = np.interp(angle, [-120, 120], [-5, 5])
                action = [action]
                print("midpoint:", action)

            observation, reward, done, info = env.step(action)
            env.render()

            if done:
                print(env.total_reward)

            if env.pp.exit:
                break

    pygame.quit()
