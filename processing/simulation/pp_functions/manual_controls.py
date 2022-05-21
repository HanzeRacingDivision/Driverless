import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd

import os
import sys
sys.path.append(os.path.abspath(os.path.join('..', '')))
from cone import *
import pp_functions.utils


def enable_dragging_screen(pp, events):
    # dragging screen using left mouse button
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 or pp.moving_view_offset:
            if not pp.moving_view_offset:
                pp.moving_view_offset = True
                pp.view_offset_mouse_pos_start = pygame.mouse.get_pos()
            mouse_pos = pygame.mouse.get_pos()
            mouseDelta = [float(mouse_pos[0] - pp.view_offset_mouse_pos_start[0]),
                          float(mouse_pos[1] - pp.view_offset_mouse_pos_start[1])]
            pp.view_offset[0] = pp.prev_view_offset[0] + mouseDelta[0]
            pp.view_offset[1] = pp.prev_view_offset[1] + mouseDelta[1]

    for event in events:
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            pp.prev_view_offset[0] = pp.view_offset[0]
            pp.prev_view_offset[1] = pp.view_offset[1]
            pp.moving_view_offset = False


# User input
def user_input(pp, events, dt):
    pressed = pygame.key.get_pressed()

    # manual steering
    if pressed[pygame.K_RIGHT]:
        pp.car.steering_angle -= 50 * dt
    elif pressed[pygame.K_LEFT]:
        pp.car.steering_angle += 50 * dt
    else:
        if pp.car.steering_angle > (50 * dt):
            pp.car.steering_angle -= 120 * dt
        elif pp.car.steering_angle < -(50 * dt):
            pp.car.steering_angle += 120 * dt
        else:
            pp.car.steering_angle = 0

    # press l for left cone
    if pressed[pygame.K_l]:
        mouse_pos = (pygame.mouse.get_pos()[0] - pp.view_offset[0], pygame.mouse.get_pos()[1] - pp.view_offset[1])

        if mouse_pos in pp.mouse_pos_list:
            pass
        else:

            make_cone = True
            for i in range(len(pp.mouse_pos_list)):
                if np.linalg.norm(tuple(x - y for x, y in zip(pp.mouse_pos_list[i], mouse_pos))) < 50:
                    make_cone = False
                    break

            if make_cone:
                cone_id = len(pp.cones.list[Side.RIGHT]) + len(pp.cones.list[Side.LEFT]) + 2
                left_cone = Cone(mouse_pos[0] / pp.ppu, mouse_pos[1] / pp.ppu, 'left', cone_id)
                pp.cones.list[Side.LEFT].append(left_cone)
                pp.mouse_pos_list.append(mouse_pos)

    # press r for right cone
    if pressed[pygame.K_r]:
        mouse_pos = (pygame.mouse.get_pos()[0] - pp.view_offset[0], pygame.mouse.get_pos()[1] - pp.view_offset[1])

        if mouse_pos in pp.mouse_pos_list:
            pass
        else:

            make_cone = True
            for i in range(len(pp.mouse_pos_list)):
                if np.linalg.norm(tuple(x - y for x, y in zip(pp.mouse_pos_list[i], mouse_pos))) < 50:
                    make_cone = False
                    break

            if make_cone:
                cone_id = len(pp.cones.list[Side.RIGHT]) + len(pp.cones.list[Side.LEFT]) + 2
                right_cone = Cone(mouse_pos[0] / pp.ppu, mouse_pos[1] / pp.ppu, 'right', cone_id)
                pp.cones.list[Side.RIGHT].append(right_cone)
                pp.mouse_pos_list.append(mouse_pos)

    # if CTRL + c then clear screen
    if pressed[pygame.K_LCTRL] and pressed[pygame.K_c]:
        # resetting most vars
        pp.targets.targets = []
        pp.targets.non_passed_targets = []
        pp.cones.list[Side.LEFT] = []
        pp.cones.list[Side.RIGHT] = []
        pp.cones.visible[Side.LEFT] = []
        pp.cones.visible[Side.RIGHT] = []
        pp.cones.in_fov[Side.LEFT] = []
        pp.cones.in_fov[Side.RIGHT] = []
        pp.path.splines[Side.LEFT] = []
        pp.path.splines[Side.RIGHT] = []
        pp.path.spline_linked[Side.RIGHT] == False
        pp.path.spline_linked[Side.LEFT] == False
        pp.mouse_pos_list = []
        pp.path.splines[Side.LEFT] = 0
        pp.path.splines[Side.RIGHT] = 0
        pp.cones.first_visible_cone[Side.LEFT] = 0
        pp.cones.first_visible_cone[Side.RIGHT] = 0
        pp.cones.first_cone_found[Side.RIGHT] = False
        pp.cones.first_cone_found[Side.LEFT] = False
        pp.track_number_changed = False
        pp.car.crashed = False
        pp.total_reward = 0

    for event in events:
        # if 2 is pressed, increasing cruising speed
        # if 1 is pressed, decrease cruising speed
        if event.type == pygame.KEYDOWN and event.key == pygame.K_1:
            pp.cruising_speed -= 0.05
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_2:
            pp.cruising_speed += 0.05
        # if a pressed then toggle automatic driving
        elif event.type == pygame.KEYUP and event.key == pygame.K_a:
            if not pp.car.auto:
                pp.car.auto = True
            else:
                pp.car.auto = False
        # if f pressed then toggle pp.fullscreen
        elif event.type == pygame.KEYUP and event.key == pygame.K_f:
            if not pp.fullscreen:
                pp.fullscreen = True
            else:
                pp.fullscreen = False
        # if t pressed then set to pp.track mode
        elif event.type == pygame.KEYUP and event.key == pygame.K_t:
            if not pp.track:
                pp.track = True
            else:
                pp.track = False

        # if D then load map
        if pressed[pygame.K_d]:
            # resetting most vars before loading
            pp.targets.targets = []
            pp.targets.non_passed_targets = []
            pp.cones.list[Side.LEFT] = []
            pp.cones.list[Side.RIGHT] = []
            pp.cones.visible[Side.LEFT] = []
            pp.cones.visible[Side.RIGHT] = []
            pp.cones.in_fov[Side.LEFT] = []
            pp.cones.in_fov[Side.RIGHT] = []
            pp.path.splines[Side.LEFT] = []
            pp.path.splines[Side.RIGHT] = []
            pp.path.spline_linked[Side.RIGHT] == False
            pp.path.spline_linked[Side.LEFT] == False
            pp.mouse_pos_list = []
            pp.path.splines[Side.LEFT] = 0
            pp.path.splines[Side.RIGHT] = 0
            pp.cones.first_visible_cone[Side.LEFT] = 0
            pp.cones.first_visible_cone[Side.RIGHT] = 0
            pp.cones.first_cone_found[Side.RIGHT] = False
            pp.cones.first_cone_found[Side.LEFT] = False
            pp.track_number_changed = False
            pp.car.crashed = False
            pp.total_reward = 0

            pp.cones.list[Side.LEFT], pp.cones.list[Side.RIGHT], pp.mouse_pos_list = pp_functions.utils.load_map(
                pp.mouse_pos_list, pp.ppu)

        # if S then save map
        if pressed[pygame.K_s]:
            pp_functions.utils.save_map(pp.cones.list[Side.LEFT], pp.cones.list[Side.RIGHT])

    # dragging screen using left mouse button
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 or pp.moving_view_offset:
            if not pp.moving_view_offset:
                pp.moving_view_offset = True
                pp.view_offset_mouse_pos_start = pygame.mouse.get_pos()
            mouse_pos = pygame.mouse.get_pos()
            mouseDelta = [float(mouse_pos[0] - pp.view_offset_mouse_pos_start[0]),
                          float(mouse_pos[1] - pp.view_offset_mouse_pos_start[1])]
            pp.view_offset[0] = pp.prev_view_offset[0] + mouseDelta[0]
            pp.view_offset[1] = pp.prev_view_offset[1] + mouseDelta[1]

    for event in events:
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            pp.prev_view_offset[0] = pp.view_offset[0]
            pp.prev_view_offset[1] = pp.view_offset[1]
            pp.moving_view_offset = False

    # if CTRL + Z pressed then undo last left and right cone
    if not pp.undo_done and pressed[pygame.K_LCTRL] and pressed[pygame.K_z]:
        pp.undo_done = True
        for side in Side:
            if len(pp.cones.visible[side]) > 0:
                if pp.cones.list[side][-1] == pp.cones.visible[side][-1]:
                    pp.mouse_pos_list.remove((pp.cones.list[side][-1].true_position.x * pp.ppu,
                                              pp.cones.list[side][-1].true_position.y * pp.ppu))
                    pp.cones.list[side].pop(-1)
                    pp.cones.visible[side].pop(-1)
                else:
                    pp.mouse_pos_list.remove((pp.cones.list[side][-1].true_position.x * pp.ppu,
                                              pp.cones.list[side][-1].true_position.y * pp.ppu))
                    pp.cones.list[side].pop(-1)
            else:
                if len(pp.cones.list[side]) > 0:
                    pp.mouse_pos_list.remove((pp.cones.list[side][-1].true_position.x * pp.ppu,
                                              pp.cones.list[side][-1].true_position.y * pp.ppu))
                    pp.cones.list[side].pop(-1)

    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_z:
            pp.undo_done = False

    # manual acceleration
    if pressed[pygame.K_UP]:
        if pp.car.velocity.x < 0:
            pp.car.acceleration = pp.car.brake_deceleration
        else:
            pp.car.acceleration += 1 * dt
    elif pressed[pygame.K_DOWN] and pp.car.breaks == True:
        if pp.car.velocity.x > 0:
            pp.car.acceleration = -pp.car.brake_deceleration
        else:
            pp.car.acceleration -= 1 * dt
    elif pressed[pygame.K_SPACE]:
        if abs(pp.car.velocity.x) > dt * pp.car.brake_deceleration:
            pp.car.acceleration = -copysign(pp.car.brake_deceleration, pp.car.velocity.x)
        else:
            pp.car.acceleration = -pp.car.velocity.x / dt
    else:
        if abs(pp.car.velocity.x) > dt * pp.car.free_deceleration:
            pp.car.acceleration = -copysign(pp.car.free_deceleration, pp.car.velocity.x)
        else:
            if dt != 0:
                pp.car.acceleration = -pp.car.velocity.x / dt

    # =============================================================================
#     
#     #If t pressed then create target
#     if pressed[pygame.K_t]
#             mouse_pos = pygame.mouse.get_pos()
#             
#             if mouse_pos in pp.mouse_pos_list:
#                 pass
#             else:
#                 make_target = True
#                 for i in range(len(pp.mouse_pos_list)):
#                     if np.linalg.norm(tuple(x-y for x,y in zip(pp.mouse_pos_list[i],mouse_pos))) < 25:
#                         make_target = False
#                         break
#                 
#                 if make_target == True:
#                     
#                     target = Target(mouse_pos[0]/pp.ppu,mouse_pos[1]/pp.ppu)
#                     targets.append(target)
#                     non_passed_targets.append(target)
#                     
#                     pp.mouse_pos_list.append(mouse_pos)
# =============================================================================
