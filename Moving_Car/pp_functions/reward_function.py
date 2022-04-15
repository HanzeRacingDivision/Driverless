import numpy as np

def calculate_reward(self):

    #initialise reward
    reward = 0

    #reward for surviving a timestep
    reward += 0.01

    #reward for finding a new cone
    for key in self.pp.cone.new_visible_cone_flag.keys():
        if self.pp.cone.new_visible_cone_flag[key]:
            reward += 25

    #reward for completing the track
    if np.linalg.norm((7 * self.pp.ppu, 10 * self.pp.ppu) - self.pp.car.position * self.pp.ppu) < 40 and int(self.episode_time_running) > 4:
        reward += 500

    #reward for crashing
    if self.pp.car.crashed:
        reward = -1000

    return reward/10