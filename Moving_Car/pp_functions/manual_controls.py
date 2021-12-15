import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd


import pp_functions.utils

 # User input
def user_input(self,
               mouse_pos_list,
               Target,
               ppu,
               targets,
               non_passed_targets,
               Cone,
               left_cones,
               right_cones,
               right_spline_linked,
               left_spline_linked,
               events,
               cruising_speed,
               car,
               track,
               fullscreen,
               current_dir,
               dt,
               circles,
               visible_left_cones,
               visible_right_cones,
               left_spline,
               right_spline,
               path_midpoints,
               path_midpoints_spline,
               first_visible_left_cone,
               first_visible_right_cone,
               first_right_cone_found,
               first_left_cone_found,
               track_number_changed,
               car_crashed,
               time_start_track,
               undo_done):
    
    pressed = pygame.key.get_pressed()
    
    #manual steering  
    if pressed[pygame.K_RIGHT]:
        car.steering -= 50 * dt
    elif pressed[pygame.K_LEFT]:
       car.steering += 50 * dt
    else:
        if(car.steering > (50 * dt)):
            car.steering -= 120 * dt
        elif(car.steering < -(50 * dt)):
            car.steering += 120 * dt
        else:
            car.steering = 0
                
                
    # press l for left cone
    if pressed[pygame.K_l]:
        mouse_pos = (pygame.mouse.get_pos()[0] - self.view_offset[0], pygame.mouse.get_pos()[1] - self.view_offset[1])
        
        if mouse_pos in mouse_pos_list:
            pass
        else:
            
            make_cone = True
            for i in range(len(mouse_pos_list)):
                if np.linalg.norm(tuple(x-y for x,y in zip(mouse_pos_list[i],mouse_pos))) < 50:
                    make_cone = False
                    break
            
            if make_cone == True:
                left_cone = Cone(mouse_pos[0]/ppu, mouse_pos[1]/ppu, 'left')
                left_cones.append(left_cone)
                mouse_pos_list.append(mouse_pos)
                
                
            
    # press r for right cone
    if pressed[pygame.K_r]:
        mouse_pos = (pygame.mouse.get_pos()[0] - self.view_offset[0], pygame.mouse.get_pos()[1] - self.view_offset[1])

        if mouse_pos in mouse_pos_list:
            pass
        else:
            
            make_cone = True
            for i in range(len(mouse_pos_list)):
                if np.linalg.norm(tuple(x-y for x,y in zip(mouse_pos_list[i],mouse_pos))) < 50:
                    make_cone = False
                    break
            
            if make_cone == True:
                right_cone = Cone(mouse_pos[0]/ppu, mouse_pos[1]/ppu, 'right')
                right_cones.append(right_cone)
                mouse_pos_list.append(mouse_pos)
    
    
    
    #if CTRL + c then clear screen
    if pressed[pygame.K_LCTRL] and pressed[pygame.K_c]:
        #resetting most vars
        targets  = []
        non_passed_targets = []
        circles = []
        left_cones = []
        right_cones = []    
        visible_left_cones = []
        visible_right_cones = []
        left_spline = []
        right_spline = []
        path_midpoints = []
        right_spline_linked == False
        left_spline_linked == False
        mouse_pos_list = []
        left_spline = 0
        right_spline = 0
        path_midpoints_spline = 0
        first_visible_left_cone = 0
        first_visible_right_cone = 0
        first_right_cone_found = False
        first_left_cone_found = False
        track_number_changed = False
        car_crashed = False
        self.total_reward = 0
        time_start_track = None
        
        
    #if 2 is pressed, increasing cruising speed
    #if 1 is pressed, decrease cruising speed
    
    for event in events:
        if event.type == pygame.KEYDOWN and event.key == pygame.K_1:
            cruising_speed -= 0.05
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_2:
            cruising_speed += 0.05
    
        
        
    #if a pressed then toggle automatic driving
    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_a: 
            if car.auto == False:
                car.auto  = True
                time_start_track = time.time()
            else:
                car.auto  = False
                
    #if f pressed then toggle fullscreen
    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_f:
            if fullscreen == False:
                fullscreen  = True
            else:
                fullscreen  = False
                
    
    #if h pressed then toggle headlight
    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_h:
            if car.headlights == False:
                car.headlights = True
            else:
                car.headlights = False
        
        
    #if t pressed then set to track mode
    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_t: 
            if track == False:
                track = True
            else:
                track = False
                
                
    #if S then save map
    if  pressed[pygame.K_s]:
        pp_functions.utils.save_map(left_cones, right_cones)
        
        
    #if D then load map
    if  pressed[pygame.K_d]:
        
        #resetting most vars before loading
        targets  = []
        non_passed_targets = []
        circles = []
        left_cones = []
        right_cones = []    
        visible_left_cones = []
        visible_right_cones = []
        left_spline = []
        right_spline = []
        path_midpoints = []
        right_spline_linked == False
        left_spline_linked == False
        mouse_pos_list = []
        left_spline = 0
        right_spline = 0
        path_midpoints_spline = 0
        first_visible_left_cone = 0
        first_visible_right_cone = 0
        first_right_cone_found = False
        first_left_cone_found = False
        track_number_changed = False
        car_crashed = False
        self.total_reward = 0
        time_start_track = None
        
        left_cones, right_cones, mouse_pos_list = pp_functions.utils.load_map(mouse_pos_list, current_dir, Cone, ppu)
        
    #dragging screen using left mouse butto
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 or self.moving_view_offset == True:
            if self.moving_view_offset == False:
                self.moving_view_offset = True
                self.view_offset_mouse_pos_start = pygame.mouse.get_pos()
            mouse_pos = pygame.mouse.get_pos()
            mouseDelta = [float(mouse_pos[0] - self.view_offset_mouse_pos_start[0]), float(mouse_pos[1] - self.view_offset_mouse_pos_start[1])]
            self.view_offset[0] = self.prev_view_offset[0] + mouseDelta[0]
            self.view_offset[1] = self.prev_view_offset[1] + mouseDelta[1]

    for event in events:
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.prev_view_offset[0] = self.view_offset[0]
            self.prev_view_offset[1] = self.view_offset[1]
            self.moving_view_offset = False

    #if CTRL + Z pressed then undo last left and right cone
    if undo_done == False and pressed[pygame.K_LCTRL] and pressed[pygame.K_z]:
        undo_done = True
        if len(visible_left_cones) > 0:
            if left_cones[-1] == visible_left_cones[-1]:
                mouse_pos_list.remove((left_cones[-1].position.x * ppu, left_cones[-1].position.y * ppu))
                left_cones.pop(-1)
                visible_left_cones.pop(-1)
            else:
                mouse_pos_list.remove((left_cones[-1].position.x * ppu, left_cones[-1].position.y * ppu))
                left_cones.pop(-1)
        else:
            if len(left_cones) > 0:
                mouse_pos_list.remove((left_cones[-1].position.x * ppu, left_cones[-1].position.y * ppu))
                left_cones.pop(-1)

        if len(visible_right_cones) > 0:
            if right_cones[-1] == visible_right_cones[-1]:
                mouse_pos_list.remove((right_cones[-1].position.x * ppu, right_cones[-1].position.y * ppu))
                right_cones.pop(-1)
                visible_right_cones.pop(-1)
            else:
                mouse_pos_list.remove((right_cones[-1].position.x * ppu, right_cones[-1].position.y * ppu))
                right_cones.pop(-1)
        else:
            if len(right_cones) > 0:
                mouse_pos_list.remove((right_cones[-1].position.x * ppu, right_cones[-1].position.y * ppu))
                right_cones.pop(-1)

    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_z:
            undo_done = False


    #manual acceleration
    if pressed[pygame.K_UP]:
        if car.velocity.x < 0:
            car.acceleration = car.brake_deceleration
        else:
            car.acceleration += 1 * dt
    elif pressed[pygame.K_DOWN] and car.breaks == True:
        if car.velocity.x > 0:
            car.acceleration = -car.brake_deceleration
        else:
            car.acceleration -= 1 * dt
    elif pressed[pygame.K_SPACE]:
        if abs(car.velocity.x) > dt * car.brake_deceleration:
            car.acceleration = -copysign(car.brake_deceleration, car.velocity.x)
        else:
            car.acceleration = -car.velocity.x / dt
    else:
        if abs(car.velocity.x) > dt * car.free_deceleration:
            car.acceleration = -copysign(car.free_deceleration, car.velocity.x)
        else:
            if dt != 0:
                car.acceleration = -car.velocity.x / dt
                
    # =============================================================================
#     
#     #If t pressed then create target
#     if pressed[pygame.K_t]
#             mouse_pos = pygame.mouse.get_pos()
#             
#             if mouse_pos in mouse_pos_list:
#                 pass
#             else:
#                 make_target = True
#                 for i in range(len(mouse_pos_list)):
#                     if np.linalg.norm(tuple(x-y for x,y in zip(mouse_pos_list[i],mouse_pos))) < 25:
#                         make_target = False
#                         break
#                 
#                 if make_target == True:
#                     
#                     target = Target(mouse_pos[0]/ppu,mouse_pos[1]/ppu)
#                     targets.append(target)
#                     non_passed_targets.append(target)
#                     
#                     mouse_pos_list.append(mouse_pos)
# =============================================================================
                
                
    return self, \
           targets, \
           non_passed_targets, \
           circles, \
           left_cones, \
           right_cones, \
           visible_left_cones, \
           visible_right_cones, \
           left_spline, \
           right_spline, \
           path_midpoints, \
           right_spline_linked, \
           left_spline_linked, \
           mouse_pos_list, \
           left_spline, \
           right_spline, \
           path_midpoints_spline, \
           first_visible_left_cone, \
           first_visible_right_cone, \
           first_right_cone_found, \
           first_left_cone_found, \
           track_number_changed, \
           car_crashed, \
           car, \
           track, \
           cruising_speed, \
           fullscreen, \
           time_start_track, \
           undo_done
