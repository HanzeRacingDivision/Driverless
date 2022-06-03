import pygame
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C, DQN

from CarEnv import CarEnv

if __name__ == "__main__":

    model_name = "PPO"
    time_steps = 500000

    models_dir = f'models/{model_name}-1649940839'

    env = CarEnv(mode="discrete")
    check_env(env)

    if model_name == 'PPO' or model_name == 'PPO-cont':
        model = PPO.load(f"{models_dir}/car_model_{time_steps}")
    elif model_name == 'A2C' or model_name == 'A2C-cont':
        model = A2C.load(f"{models_dir}/car_model_{time_steps}") 
    elif model_name == 'DQN' or model_name == 'DQN-cont':
        model = DQN.load(f"{models_dir}/car_model_{time_steps}")
    else:
        raise ValueError(f"{model_name} not recognized as valid model name")

    episodes = 1

    for i in range(episodes):
        done = False
        observation = env.reset()
        while not done:
            action, _states = model.predict(observation)
            observation, reward, done, info = env.step(action)
            env.render()

            if done:
                print(env.total_reward)

            if env.pp.exit:
                break

    pygame.quit()
