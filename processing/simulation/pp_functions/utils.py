# misc functions
import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd

import sys

sys.path.append(os.path.abspath(os.path.join('..', 'Moving_Car')))
from cone import *


def load_existing_map(name):
    left_cones = []
    right_cones = []

    current_dir = os.getcwd()
    map_path = os.path.join(current_dir, f"levels generated/{name}.csv")
    map_file = pd.read_csv(map_path)

    for i in range(len(map_file.iloc[:, 0])):
        if map_file['Cone_Type'].iloc[i] == 'LEFT':

            left_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], Side.LEFT, i+2)
            left_cones.append(left_cone)
            # mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))             

        else:
            right_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], Side.RIGHT, i+2)
            right_cones.append(right_cone)
            # mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))

    return left_cones, right_cones


def save_map(left_cones, right_cones):
    cone_x = []
    cone_y = []
    cone_type = []
    print('SAVE MAP AS : ')
    name = input()

    for i in range(len(left_cones)):
        cone_x.append(left_cones[i].true_position.x)
        cone_y.append(left_cones[i].true_position.y)
        cone_type.append('LEFT')

    for i in range(len(right_cones)):
        cone_x.append(right_cones[i].true_position.x)
        cone_y.append(right_cones[i].true_position.y)
        cone_type.append('RIGHT')

    map_file = pd.DataFrame({'Cone_Type': cone_type,
                             'Cone_X': cone_x,
                             'Cone_Y': cone_y})

    map_file.to_csv(f'levels generated/{name}.csv')


def load_map(mouse_pos_list, ppu):
    left_cones = []
    right_cones = []
    print('LOAD MAP : ')
    name = input()

    current_dir = os.getcwd()
    map_path = os.path.join(current_dir, f"levels generated/{name}.csv")
    map_file = pd.read_csv(map_path)

    for i in range(len(map_file.iloc[:, 0])):
        print(i)
        if map_file['Cone_Type'].iloc[i] == 'LEFT':

            left_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'left', i+2)
            left_cones.append(left_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))

        else:
            right_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'right', i+2)
            right_cones.append(right_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))

    return left_cones, right_cones, mouse_pos_list


def bound_angle_180(angle):
    # car angle between (-180,180)
    temp_sign = np.mod(angle, 360)
    if temp_sign > 180:
        angle_sign = -1
    else:
        angle_sign = 1

    angle = np.mod(angle, 180) * angle_sign

    if angle < 0:
        angle = -180 - angle

    return angle
