from CarEnv import CarEnv
from stable_baselines3 import PPO, A2C, DQN
import os
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
import time
from typing import Union


def linear_schedule(initial_value: Union[float, str]):
    """
    Linear learning rate schedule.
    :param initial_value: (float or str)
    :return: (function)
    """

    if isinstance(initial_value, str):
        initial_value = float(initial_value)

    def func(progress: float):
        """
        Progress will decrease from 1 (beginning) to 0
        :param progress: (float)
        :return: (float)
        """

        return progress * initial_value

    return func


if __name__ == "__main__":
    mode = "cont"
    model_name = "PPO-cont"

    models_dir = f'models/{model_name}-{int(time.time())}'
    log_dir = f'logs/{model_name}-{int(time.time())}'

    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    env = CarEnv(mode=mode)
    env = Monitor(env, log_dir)

    if model_name == 'PPO' or model_name == 'PPO-cont':
        model = PPO("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log=log_dir)
    elif model_name == 'A2C' or model_name == 'A2C-cont':
        model = A2C("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log=log_dir)
    elif model_name == 'DQN' or model_name == 'DQN-cont':
        model = DQN("MlpPolicy", env, learning_rate=linear_schedule(3e-4), verbose=1, tensorboard_log=log_dir)
    else:
        raise ValueError(f"{model_name} not recognized as valid model name")

    timesteps = 10000
    for i in range(100):
        model.learn(total_timesteps=timesteps, reset_num_timesteps=False, tb_log_name=f"{model_name}")
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
