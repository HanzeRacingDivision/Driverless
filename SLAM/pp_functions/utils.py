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

def draw_line_dashed(surface, color, start_pos, end_pos, offset, width = 1, dash_length = 10, exclude_corners = True):

         'simply a function that draws dashed lines in pygame'    
     
         # convert tuples to numpy arrays
         start_pos = np.array(start_pos)
         end_pos   = np.array(end_pos)
     
         # get euclidian distance between start_pos and end_pos
         length = np.linalg.norm(end_pos - start_pos)
     
         # get amount of pieces that line will be split up in (half of it are amount of dashes)
         dash_amount = int(length / dash_length)
     
         # x-y-value-pairs of where dashes start (and on next, will end)
         dash_knots = np.array([np.linspace(start_pos[i] , end_pos[i], dash_amount) for i in range(2)]).transpose()
         dash_knots += offset
     
         return [pygame.draw.line(surface, color, tuple(dash_knots[n]), tuple(dash_knots[n+1]), width)
                 for n in range(int(exclude_corners), dash_amount - int(exclude_corners), 2)]    
     
     
def save_map(left_cones, right_cones):
    cone_x = []
    cone_y = []
    cone_type = []
    print('SAVE MAP AS : ')
    name = input()

    for i in range(len(left_cones)):
        cone_x.append(left_cones[i].position.x)
        cone_y.append(left_cones[i].position.y)
        cone_type.append('LEFT')
        
    for i in range(len(right_cones)):
        cone_x.append(right_cones[i].position.x)
        cone_y.append(right_cones[i].position.y)
        cone_type.append('RIGHT')       
        

    map_file = pd.DataFrame({'Cone_Type' : cone_type,
                                 'Cone_X' : cone_x,
                                 'Cone_Y' : cone_y})

    map_file.to_csv(f'{name}.csv')
     
def load_map(mouse_pos_list, current_dir, Cone, ppu):
    
    left_cones = []
    right_cones = []
    print('LOAD MAP : ')
    name = input()
    
    map_path = os.path.join(current_dir, f"{name}.csv")
    map_file = pd.read_csv(map_path)
    
    for i in range(len(map_file.iloc[:,0])):
        if map_file['Cone_Type'].iloc[i] == 'LEFT':

            left_cone = Cone(map_file['Cone_X'].iloc[i],map_file['Cone_Y'].iloc[i], 'left', left_cones, right_cones)
            left_cones.append(left_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))             
        
        else:
            right_cone = Cone(map_file['Cone_X'].iloc[i],map_file['Cone_Y'].iloc[i], 'right', left_cones, right_cones)
            right_cones.append(right_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))             
        
    
    return left_cones, right_cones, mouse_pos_list

def auto_load_map(mouse_pos_list, current_dir, Cone, ppu, map_name):
    
    left_cones = []
    right_cones = []
    name = map_name
    
    map_path = os.path.join(current_dir, f"{name}.csv")
    map_file = pd.read_csv(map_path)
    
    for i in range(len(map_file.iloc[:,0])):
        if map_file['Cone_Type'].iloc[i] == 'LEFT':

            left_cone = Cone(map_file['Cone_X'].iloc[i],map_file['Cone_Y'].iloc[i], 'left', left_cones, right_cones)
            left_cones.append(left_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))             
        
        else:
            right_cone = Cone(map_file['Cone_X'].iloc[i],map_file['Cone_Y'].iloc[i], 'right', left_cones, right_cones)
            right_cones.append(right_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i]*ppu,map_file['Cone_Y'].iloc[i]*ppu))             
        
    return left_cones, right_cones, mouse_pos_list

def update_target_lists(targets, non_passed_targets):
    
    #list of visibile targets and non-passed targets
    dists = []
    non_passed_dists = []
    visible_targets = []
    visible_dists = []
    
    
    #make list of visible targets and list of passed targets
    for target in targets:
        
        non_passed_dists.append(target.dist_car)
        dists.append(target.dist_car)
            
        if target.passed == True:
            non_passed_dists.remove(target.dist_car)
    
        if target.visible == True:
            visible_targets.append(target)
            visible_dists.append(target.dist_car)
            
    for target in non_passed_targets:
        if target.passed == True:
            non_passed_targets.remove(target)
            
    return visible_targets, non_passed_dists, visible_dists, dists
         
        
def update_cone_lists(left_cones, right_cones, visible_left_cones, visible_right_cones):        
    #make list of visible left cones
    len_visible_left_cones_old = len(visible_left_cones)
    
    visible_left_cones = []
    for left_cone in left_cones:
        if left_cone.visible == True:
            visible_left_cones.append(left_cone)
            
    #tracking the number of cones at each iteration and creating a flag for when a new cone is detected
    len_visible_left_cones_new = len(visible_left_cones)
    new_visible_left_cone_flag = len_visible_left_cones_new != len_visible_left_cones_old
    
    
    
    #make list of visible right cones
    len_visible_right_cones_old = len(visible_right_cones)
    
    visible_right_cones = []
    for right_cone in right_cones:
        if right_cone.visible == True:
            visible_right_cones.append(right_cone)
            
    #tracking the number of cones at each iteration and creating a flag for when a new cone is detected
    len_visible_right_cones_new = len(visible_right_cones)
    new_visible_right_cone_flag = len_visible_right_cones_new != len_visible_right_cones_old
    
    return visible_left_cones, \
           visible_right_cones, \
           len_visible_left_cones_new, \
           len_visible_right_cones_new, \
           new_visible_left_cone_flag, \
           new_visible_right_cone_flag
        
def closest_target(visible_targets, visible_dists):
    #define closest target
    closest_target = None
    if len(visible_targets) > 0:
        if len(visible_dists) == 0:
            visible_targets = []
        else:
            closest_target = visible_targets[np.array(visible_dists).argmin()]
    return closest_target, visible_targets

    
def reset_targets(targets, non_passed_targets):
    non_passed_targets = targets.copy()
    for target in targets:
        target.passed = False
    return targets, non_passed_targets
































