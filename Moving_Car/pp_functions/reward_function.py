import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd

def calculate_reward(lap_reward, car, track_number):
    reward = 0
 
    if car.auto == True:
        if lap_reward == True and track_number > 0:
            reward += 100
            lap_reward = False
        reward += 0.1 * car.velocity.x**2/car.max_velocity
        reward += 0.005
    return reward, lap_reward

