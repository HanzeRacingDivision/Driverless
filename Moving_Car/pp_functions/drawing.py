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
from cone import Side
from pp_functions.utils import bound_angle_180

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


def render(pp):
    
    pp.screen.fill((0, 0, 0))
    rotated = pygame.transform.rotate(pp.car.car_image, pp.car.angle)
    rect = rotated.get_rect()
     
    pos_temp = pp.car.position * pp.ppu
    pos_1 = int(pos_temp.x)
    pos_2 = int(pos_temp.y)

    def apply_view_offset(item):
        if pp.car_centre:
            return item + (pp.view_offset[0] + pp.car.position.x * pp.ppu + pp.width/2, pp.view_offset[1] + pp.car.position.y * pp.ppu + pp.height/2)
        else:
            return item + (pp.view_offset[0], pp.view_offset[1])

    if pp.car_centre:
        offset = [pp.view_offset[0] + pp.car.position.x * pp.ppu + pp.width/2, pp.view_offset[1] + pp.car.position.y * pp.ppu + pp.height/2]
    else:
        offset = [pp.view_offset[0], pp.view_offset[1]]
    
    # draw targets
    if len(pp.target.targets) > 0:
        for target in pp.target.targets:
            if target in pp.target.non_passed_targets:
                pass
                pp.screen.blit(pp.target.image, apply_view_offset(target.position * pp.ppu - (3,3)))
            else:
                pass
                #pp.screen.blit(target_image_g, target.position * ppu - (3,3))
            if target.visible == True and pp.car.auto == True:
                pass
                #pp_functions.utils.draw_line_dashed(pp.screen, (150,150,150),(pos_1,pos_2) , target.position * ppu , width = 1, dash_length = 10, exclude_corners = True)
      
    for category in Side:
        for cone in pp.cone.cone_list[category]:
            pp.screen.blit(pp.cone.image[category], apply_view_offset(cone.position * pp.ppu - (3,3)))

        for cone in pp.cone.in_fov_cone_list[category]:
            if cone.in_fov:
                draw_line_dashed(pp.screen, (150,150,150),(pos_1,pos_2) , cone.position * pp.ppu , offset, width = 1, dash_length = 10, exclude_corners = True)
    
        if pp.path.splines[category] != 0 and len(pp.path.splines[category]) > 0:
            for i in range(len(pp.path.splines[category][0])):
                pp.screen.blit(pp.path.spline_image[category], apply_view_offset(Vector2(pp.path.splines[category][0][i],pp.path.splines[category][1][i]) * pp.ppu - (3,3)))

    if pp.cone.first_visible_cone[Side.LEFT] != 0 and pp.cone.first_visible_cone[Side.RIGHT] != 0:
        draw_line_dashed(pp.screen, (255, 51, 0),(pp.cone.first_visible_cone[Side.LEFT].position.x* pp.ppu, pp.cone.first_visible_cone[Side.LEFT].position.y* pp.ppu) , (pp.cone.first_visible_cone[Side.RIGHT].position.x* pp.ppu, pp.cone.first_visible_cone[Side.RIGHT].position.y* pp.ppu),offset , width = 2, dash_length = 5, exclude_corners = True)
    
   # if path_midpoints != 0 and len(path_midpoints) > 0:
     #  print(f'midpoints : {path_midpoints}')
    #    for i in range(len(path_midpoints[0])):
    #        pp.screen.blit(target_image, Vector2(path_midpoints[0][i],path_midpoints[1][i]) * ppu - (3,3))
    
    
    #if path_midpoints_spline != 0 and len(path_midpoints_spline) > 0:
     #   for i in range(len(path_midpoints_spline[0])):
     #       pp.screen.blit(target_image, Vector2(path_midpoints_spline[0][i],path_midpoints_spline[1][i]) * ppu - (3,3))
    
    
    # draw the car sprite
    #pygame.draw.rect(pp.screen, (200,200,200), (car.position * ppu - ((rect.width / 2),(rect.height / 2)), (rect.width, rect.height))) #draws a little box around the car sprite (just for debug)
    if pp.car.crashed == False:
        pp.screen.blit(rotated, apply_view_offset(pp.car.position * pp.ppu - ((rect.width / 2),(rect.height / 2)))) #draw car
    # else:
    #     pp.screen.blit(explosion_image, car.position * ppu - ((explosion_image.get_rect().width / 2),(explosion_image.get_rect().height / 2)))
    
    # draw dotted lines between car and closest target
    #if len(pp.target.visible_targets) > 0 and pp.car.auto == True:
    #    draw_line_dashed(pp.screen, (155,255,255),(pos_1,pos_2) , pp.target.closest_target.position * pp.ppu , offset, width = 2, dash_length = 10, exclude_corners = True)

    # drawing the boundary sample (extracting it directly from the observation to ensure its working correctly) 

    for key in pp.cone.new_visible_cone_flag.keys():
        for i, sample in enumerate(pp.cone.polar_boundary_sample[key]):
            sample_x = pp.car.position.x + sample[0] * np.cos(radians(bound_angle_180(pp.car.angle + sample[1])))
            sample_y = pp.car.position.y + sample[0] * np.sin(-1 * radians(bound_angle_180(pp.car.angle + sample[1])))

            text_surf = pygame.font.Font(None, 20).render(f'S', 1, (255, 100, 100))
            text_pos = [(sample_x * pp.ppu) + offset[0], (sample_y * pp.ppu) + offset[1]]
            pp.screen.blit(text_surf, text_pos)

    for key in pp.cone.new_visible_cone_flag.keys():
        for i in range(len(pp.cone.boundary_sample[key][0])):
            text_surf = pygame.font.Font(None, 20).render(f'S', 1, (255, 255, 255))
            text_pos = [(pp.cone.boundary_sample[key][0][i] * pp.ppu) + offset[0], (pp.cone.boundary_sample[key][1][i] * pp.ppu) + offset[1]]
            pp.screen.blit(text_surf, text_pos)       

    if pp.fullscreen == False:
        text_font = pygame.font.Font(None, 30)
     #   text_surf = text_font.render(f'Angle to target : {round(alpha,1)}', 1, (255, 255, 255))
     #   text_pos = [10, 10]
     #   pp.screen.blit(text_surf, text_pos)
        
        #text_surf = text_font.render(f'Car angle : {round(pp.car.angle,1)}', 1, (255, 255, 255))
        #text_pos = [10, 15]
        #pp.screen.blit(text_surf, text_pos)

        text_surf = text_font.render(f'Reward: {pp.reward}', 1, (255, 255, 255))
        text_pos = [10, 15]
        pp.screen.blit(text_surf, text_pos)

        #text_surf = text_font.render(f'offset : {round(pp.view_offset[0],1)}, {round(pp.view_offset[1],1)} ', 1, (255, 255, 255))
        #text_pos = [10, 15]
        #pp.screen.blit(text_surf, text_pos)
        
        
        text_surf = text_font.render(f'Steering : {round(pp.car.steering_angle,1)}', 1, (255, 255, 255))
        text_pos = [10, 35]
        pp.screen.blit(text_surf, text_pos)
    
    
        text_surf = text_font.render(f'Speed : {round(pp.car.velocity.x,1)}', 1, (255, 255, 255))
        text_pos = [10, 55]
        pp.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Distance to target : {round(dist,2)}', 1, (255, 255, 255))
     #   text_pos = [10, 70]
     #   pp.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Targets passed: {len(targets) - len(non_passed_targets)}', 1, (255, 255, 255))
     #   text_pos = [10, 110]
     #   pp.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Number of targets: {len(targets)}', 1, (255, 255, 255))
     #   text_pos = [10, 130]
     #   pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Track: {pp.track}', 1, (255, 255, 255))
        text_pos = [10, 100]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Autonomous: {pp.car.auto}', 1, (255, 255, 255))
        text_pos = [10, 80]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Lap: {pp.track_number}', 1, (255, 255, 255))
        text_pos = [10, 120]
        pp.screen.blit(text_surf, text_pos)
        
        
      #  text_surf = text_font.render(f'number of visible left cones: {len(visible_left_cones)}', 1, (255, 255, 255))
      #  text_pos = [10, 120]
      #  pp.screen.blit(text_surf, text_pos)
        
       # text_surf = text_font.render(f'Headlights: {car.headlights}', 1, (255, 255, 255))
       # text_pos = [10, 190]
       # pp.screen.blit(text_surf, text_pos)
       
        text_surf = text_font.render('Press 1 and 2 to alter car speed', 1, (155, 155, 155))
        text_pos = [10, 520]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press A to toggle autonomous', 1, (155, 155, 155))
        text_pos = [10, 540]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press L to place left cone', 1, (155, 155, 155))
        text_pos = [10, 560]
        pp.screen.blit(text_surf, text_pos)            
    
        text_surf = text_font.render('Press R to place right cone', 1, (155, 155, 155))
        text_pos = [10, 580]
        pp.screen.blit(text_surf, text_pos)            
        
        text_surf = text_font.render('Press T to make track', 1, (155, 155, 155))
        text_pos = [10, 600]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press CTRL + C to clear', 1, (155, 155, 155))
        text_pos = [10, 620]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press S to save map', 1, (155, 155, 155))
        text_pos = [10, 640]
        pp.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press D to load map', 1, (155, 155, 155))
        text_pos = [10, 660]
        pp.screen.blit(text_surf, text_pos)
    else:  
        text_font = pygame.font.Font(None, 30)
        text_surf = text_font.render('Press F to exit Fullscreen', 1, (80, 80, 80))
        text_pos = [10, 690]
        pp.screen.blit(text_surf, text_pos)
    
    
    pygame.display.flip()
    
    