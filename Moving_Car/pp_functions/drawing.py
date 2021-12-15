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

def render(self,
           car_image,car,
           ppu,targets,
           non_passed_targets,
           target_image,
           left_cones,
           left_cone_image,
           right_cones,
           right_cone_image,
           visible_left_cones,
           visible_right_cones,
           left_spline,right_spline,
           left_spline_image,
           right_spline_image,
           first_visible_left_cone,
           first_visible_right_cone,
           car_crashed,
           explosion_image,
           fullscreen,
           track,
           track_number,
           car_angle):
    
    self.screen.fill((0, 0, 0))
    rotated = pygame.transform.rotate(car_image, car.angle)
    rect = rotated.get_rect()
     
    pos_temp = car.position * ppu
    pos_1 = int(pos_temp.x)
    pos_2 = int(pos_temp.y)

    def apply_view_offset(item):
        return item + (self.view_offset[0], self.view_offset[1])
     
    #circle = (pos_1,pos_2)
    #circles.append(circle)
     
    # draw headlights
    if car.headlights == True:
        pil_size = car.fov*2
    
        pil_image = Image.new("RGBA", (pil_size, pil_size))
        pil_draw = ImageDraw.Draw(pil_image)
        #pil_draw.arc((0, 0, pil_size-1, pil_size-1), 0, 270, fill=RED)
        pil_draw.pieslice((0, 0, pil_size-1, pil_size-1), -car.angle-car.fov_range, -car.angle+car.fov_range, fill= (55, 55, 35))
        
        mode = pil_image.mode
        size = pil_image.size
        data = pil_image.tobytes()
        
        image = pygame.image.fromstring(data, size, mode)
        image_rect = image.get_rect(center = (pos_1 + self.view_offset[0], pos_2 + self.view_offset[1]))
    
        self.screen.blit(image, image_rect)        
        
    
    # draw dotted line of past car locations
    #for i in range(len(circles)):
    #    pygame.draw.circle(self.screen,(155,155,155), circles[i], 1, 1)
                    
    
    # draw targets
    if len(targets) > 0:
        for target in targets:
            if target in non_passed_targets:
                pass
                self.screen.blit(target_image, apply_view_offset(target.position * ppu - (3,3)))
            else:
                pass
                #self.screen.blit(target_image_g, target.position * ppu - (3,3))
            if target.visible == True and car.auto == True:
                pass
                #pp_functions.utils.draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , target.position * ppu , width = 1, dash_length = 10, exclude_corners = True)
      
        
    if len(left_cones) > 0:
        for left_cone in left_cones:
            self.screen.blit(left_cone_image, apply_view_offset(left_cone.position * ppu - (3,3)))
            
    if len(right_cones) > 0:
        for right_cone in right_cones:
            self.screen.blit(right_cone_image, apply_view_offset(right_cone.position * ppu - (3,3)))
            
    if len(visible_left_cones) > 0:
        for left_cone in visible_left_cones:
            if left_cone.in_fov == True:
                pp_functions.utils.draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , left_cone.position * ppu , self.view_offset, width = 1, dash_length = 10, exclude_corners = True)
    
            
    if len(visible_right_cones) > 0:
        for right_cone in visible_right_cones:
            if right_cone.in_fov == True:
                pp_functions.utils.draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , right_cone.position * ppu, self.view_offset , width = 1, dash_length = 10, exclude_corners = True)
    
    
    
    if left_spline != 0 and len(left_spline) > 0:
        for i in range(len(left_spline[0])):
            self.screen.blit(left_spline_image, apply_view_offset(Vector2(left_spline[0][i],left_spline[1][i]) * ppu - (3,3)))
            
    if right_spline != 0 and len(right_spline) > 0:
     #   print(f'right spline : {right_spline}')
        for i in range(len(right_spline[0])):
            self.screen.blit(right_spline_image, apply_view_offset(Vector2(right_spline[0][i],right_spline[1][i]) * ppu - (3,3)))
    
    
    if first_visible_left_cone != 0 and first_visible_right_cone != 0:
        pp_functions.utils.draw_line_dashed(self.screen, (255, 51, 0),(first_visible_left_cone.position.x* ppu ,first_visible_left_cone.position.y* ppu) , (first_visible_right_cone.position.x* ppu ,first_visible_right_cone.position.y* ppu), self.view_offset , width = 2, dash_length = 5, exclude_corners = True,)
    
    
    
   # if path_midpoints != 0 and len(path_midpoints) > 0:
     #  print(f'midpoints : {path_midpoints}')
    #    for i in range(len(path_midpoints[0])):
    #        self.screen.blit(target_image, Vector2(path_midpoints[0][i],path_midpoints[1][i]) * ppu - (3,3))
    
    
    #if path_midpoints_spline != 0 and len(path_midpoints_spline) > 0:
     #   for i in range(len(path_midpoints_spline[0])):
     #       self.screen.blit(target_image, Vector2(path_midpoints_spline[0][i],path_midpoints_spline[1][i]) * ppu - (3,3))
    
    
    # draw the car sprite
    #pygame.draw.rect(self.screen, (200,200,200), (car.position * ppu - ((rect.width / 2),(rect.height / 2)), (rect.width, rect.height))) #draws a little box around the car sprite (just for debug)
    if car_crashed == False:
        self.screen.blit(rotated, apply_view_offset(car.position * ppu - ((rect.width / 2),(rect.height / 2)))) #draw car
    else:
        self.screen.blit(explosion_image, car.position * ppu - ((explosion_image.get_rect().width / 2),(explosion_image.get_rect().height / 2)))
    
    # draw dotted lines between car and closest target
    #if len(visible_targets) > 0 and car.auto == True:
       # pp_functions.utils.draw_line_dashed(self.screen, (155,255,255),(pos_1,pos_2) , closest_target.position * ppu , width = 2, dash_length = 10, exclude_corners = True)
    
    
    if fullscreen == False:
        text_font = pygame.font.Font(None, 30)
     #   text_surf = text_font.render(f'Angle to target : {round(alpha,1)}', 1, (255, 255, 255))
     #   text_pos = [10, 10]
     #   self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Car angle : {round(car_angle,1)}', 1, (255, 255, 255))
        text_pos = [10, 15]
        self.screen.blit(text_surf, text_pos)
        
        #text_surf = text_font.render(f'offset : {round(self.view_offset[0],1)}, {round(self.view_offset[1],1)} ', 1, (255, 255, 255))
        #text_pos = [10, 15]
        #self.screen.blit(text_surf, text_pos)
        
        
        text_surf = text_font.render(f'Steering : {round(car.steering,1)}', 1, (255, 255, 255))
        text_pos = [10, 35]
        self.screen.blit(text_surf, text_pos)
    
    
        text_surf = text_font.render(f'Speed : {round(car.velocity.x,1)}', 1, (255, 255, 255))
        text_pos = [10, 55]
        self.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Distance to target : {round(dist,2)}', 1, (255, 255, 255))
     #   text_pos = [10, 70]
     #   self.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Targets passed: {len(targets) - len(non_passed_targets)}', 1, (255, 255, 255))
     #   text_pos = [10, 110]
     #   self.screen.blit(text_surf, text_pos)
        
     #   text_surf = text_font.render(f'Number of targets: {len(targets)}', 1, (255, 255, 255))
     #   text_pos = [10, 130]
     #   self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Track: {track}', 1, (255, 255, 255))
        text_pos = [10, 100]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render(f'Autonomous: {car.auto}', 1, (255, 255, 255))
        text_pos = [10, 80]
        self.screen.blit(text_surf, text_pos)
        if track == True:
            text_surf = text_font.render(f'Lap: {track_number}', 1, (255, 255, 255))
            text_pos = [10, 120]
            self.screen.blit(text_surf, text_pos)
        
        
      #  text_surf = text_font.render(f'number of visible left cones: {len(visible_left_cones)}', 1, (255, 255, 255))
      #  text_pos = [10, 120]
      #  self.screen.blit(text_surf, text_pos)
        
       # text_surf = text_font.render(f'Headlights: {car.headlights}', 1, (255, 255, 255))
       # text_pos = [10, 190]
       # self.screen.blit(text_surf, text_pos)
        text_surf = text_font.render('Press F to enter Fullscreen', 1, (155, 155, 155))
        text_pos = [10, 500]
        self.screen.blit(text_surf, text_pos)
       
        text_surf = text_font.render('Press 1 and 2 to alter car speed', 1, (155, 155, 155))
        text_pos = [10, 520]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press A to toggle autonomous', 1, (155, 155, 155))
        text_pos = [10, 540]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press H to toggle headlights', 1, (155, 155, 155))
        text_pos = [10, 560]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press L to place left cone', 1, (155, 155, 155))
        text_pos = [10, 580]
        self.screen.blit(text_surf, text_pos)            
    
        text_surf = text_font.render('Press R to place right cone', 1, (155, 155, 155))
        text_pos = [10, 600]
        self.screen.blit(text_surf, text_pos)            
        
        text_surf = text_font.render('Press T to make track', 1, (155, 155, 155))
        text_pos = [10, 620]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press CTRL + C to clear', 1, (155, 155, 155))
        text_pos = [10, 640]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press S to save map', 1, (155, 155, 155))
        text_pos = [10, 680]
        self.screen.blit(text_surf, text_pos)
        
        text_surf = text_font.render('Press D to load map', 1, (155, 155, 155))
        text_pos = [10, 660]
        self.screen.blit(text_surf, text_pos)
    else:  
        text_font = pygame.font.Font(None, 30)
        text_surf = text_font.render('Press F to exit Fullscreen', 1, (80, 80, 80))
        text_pos = [10, 690]
        self.screen.blit(text_surf, text_pos)
    
    
    pygame.display.flip()
    
    self.clock.tick(self.ticks)