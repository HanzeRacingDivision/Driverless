import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd
import gym

import pp_functions.utils
import pp_functions.drawing
import pp_functions.car_crash
import pp_functions.reward_function
import pp_functions.manual_controls
import pp_functions.boundary_midpoints_splines


class Car:
    def __init__(self, x, y, angle = 0, length = 2, max_steering = 80, max_acceleration = 4.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 5
        self.brake_deceleration = 4
        self.free_deceleration = 1

        self.acceleration = 0.0
        self.steering = 0.0
        self.fov = 175 #150
        self.turning_sharpness = 1.8
        self.breaks = True
        self.fov_range = 60
        self.auto = False
        self.headlights = False

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.angle += degrees(angular_velocity) * dt
        
        
        
class Target:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.passed = False
        self.dist_car = 10**10
        self.alpha = 0
        self.visible = False
        
        
    def update(self, car, time_running, ppu, car_angle): 
        
        #distance to car
        self.dist_car = np.linalg.norm(self.position-car.position)
        
        # if within 20 pixels of car, target has been 'passed' by the car
        if self.passed == False and np.linalg.norm(self.position-car.position) <= 20/ppu: 
            self.passed = True
            self.visible = False
            
        #calculating angle between car angle and target
        if np.linalg.norm(self.position-car.position) < car.fov/ppu:
            
            a_b = self.position-car.position
            a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
            
            rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
            
            a_b = rotate*a_b
            
            a = a_b[0]
            b = a_b[1]
            
            beta = np.arctan(b/a)*(180/np.pi)
            alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
            self.alpha = alpha[0,0]
            
            #if the target is outside the car fov, it is no longer visible
            if np.abs(self.alpha) < car.fov_range and self.passed == False:
                self.visible = True
            else:
                self.visible = False
        else:
            self.visible = False
            
            
class Cone:
    def __init__(self, x, y, category):
        self.position = Vector2(x, y)        
        self.visible = False   
        self.in_fov = False
        self.category = category
        
    def update(self, car, time_running, ppu, car_angle): 
        
        #distance to car
        self.dist_car = np.linalg.norm(self.position-car.position)
        
        #calculating angle between car angle and cone
        if np.linalg.norm(self.position-car.position) < car.fov/ppu and car.auto == True:
            
            a_b = self.position-car.position
            a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
            
            rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
            
            a_b = rotate*a_b
            
            a = a_b[0]
            b = a_b[1]
            
            beta = np.arctan(b/a)*(180/np.pi)
            alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
            self.alpha = alpha[0,0]
            
            #if cone within car fov, set to visible
            if np.abs(self.alpha) < car.fov_range:
                self.visible = True
                self.in_fov = True
            else:
                pass
                #self.visible = False #commenting this line allows cones to be remembered
                self.in_fov = False
        else:
            pass
            #self.visible = False  #commenting this line allows cones to be remembered
            self.in_fov = False


class PathPlanning:
    def __init__(self, map_name = 'MAP_NULL'):
        pygame.init()
        pygame.display.set_caption("Car")
        width = 1280
        height = 720
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False
        self.total_reward = 0
        self.cruising_speed = 0
        self.map_name = map_name

        self.view_offset = [0.0, 0.0]
        self.prev_view_offset = [0.0, 0.0]
        self.moving_view_offset = False
        self.view_offset_mouse_pos_start = [0.0,0.0]

    def run(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "car_r_30.png")
        car_image = pygame.image.load(image_path)
        
        image_path7 = os.path.join(current_dir, "explosion_image.png")
        explosion_image = pygame.image.load(image_path7)
        
        image_path1 = os.path.join(current_dir, "target_r_t.png")
        target_image = pygame.image.load(image_path1)
        
        image_path2 = os.path.join(current_dir, "target_g_t.png")
        target_image_g = pygame.image.load(image_path2)
        
        image_path3 = os.path.join(current_dir, "left_cone_s.png")
        left_cone_image = pygame.image.load(image_path3)
        
        image_path4 = os.path.join(current_dir, "right_cone_s.png")
        right_cone_image = pygame.image.load(image_path4)

        image_path5 = os.path.join(current_dir, "left_spline_s.png")
        left_spline_image = pygame.image.load(image_path5)
        
        image_path6 = os.path.join(current_dir, "right_spline_s.png")
        right_spline_image = pygame.image.load(image_path6)

        car = Car(15,3)

        ppu = 32
        time_start = time.time()

        
        targets = []
        non_passed_targets = targets.copy()
        
        left_cones = []
        right_cones = []
        visible_left_cones = []
        visible_right_cones = []
        left_spline = 0
        right_spline = 0
        path_midpoints = 0
        path_midpoints_spline = 0
        first_visible_left_cone = 0
        first_visible_right_cone = 0
        
        alpha = 0
        circles = []
        dist = 0
        closest_target = None
        mouse_pos_list = []
        target_locations = []
        track = False
        right_spline_linked = False
        left_spline_linked = False
        fullscreen = False
        track_number = -1
        cruising_speed = 2
        cone_connect_list = False
        first_right_cone_found = False
        first_left_cone_found = False
        midpoint_created = False
        track_number_changed = False
        car_crashed = False
        start_time_set = False
        lap_reward = False
        time_start_track = None
        undo_done = False
         
        while not self.exit:
            
            if start_time_set == False:
                time_start_sim = time.time()
                start_time_set = True
            
            dt = self.clock.get_time() / 500

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True
                        
            #User input/manual controls
            self, targets, non_passed_targets, circles, left_cones, right_cones, \
            visible_left_cones, visible_right_cones, left_spline, right_spline, \
            path_midpoints, right_spline_linked, left_spline_linked, mouse_pos_list, \
            left_spline, right_spline, path_midpoints_spline, first_visible_left_cone, \
            first_visible_right_cone, first_right_cone_found, first_left_cone_found, \
            track_number_changed, car_crashed, car, track, cruising_speed,\
            fullscreen, time_start_track, undo_done \
                 = pp_functions.manual_controls.user_input(self, mouse_pos_list, Target, ppu, targets,non_passed_targets,
                                                                       Cone,left_cones,right_cones, right_spline_linked,
                                                                       left_spline_linked,events,cruising_speed,car,track,
                                                                       fullscreen,current_dir,dt,circles,visible_left_cones,
                                                                       visible_right_cones,left_spline,right_spline,path_midpoints,
                                                                       path_midpoints_spline,first_visible_left_cone,
                                                                       first_visible_right_cone,first_right_cone_found,
                                                                       first_left_cone_found,track_number_changed,car_crashed,
                                                                       time_start_track, undo_done)
                        
            #Defining the time running since simulation started
            time_running = time.time() - time_start
            
            #redefining the car angle so that it is in (-180,180)
            temp_sign = np.mod(car.angle,360)
            if temp_sign > 180:
                car_angle_sign = -1
            else:
                car_angle_sign = 1
                
            car_angle = np.mod(car.angle,180)*car_angle_sign
            
            if car_angle < 0:
                car_angle = -180 - car_angle

            #update target list
            visible_targets, \
            non_passed_dists, \
            visible_dists, \
            dists = pp_functions.utils.update_target_lists(targets, non_passed_targets)
           
            #update cone list
            visible_left_cones, \
            visible_right_cones, \
            len_visible_left_cones_new, \
            len_visible_right_cones_new, \
            new_visible_left_cone_flag, \
            new_visible_right_cone_flag = pp_functions.utils.update_cone_lists(left_cones, right_cones, visible_left_cones, visible_right_cones)
            
            #calculate closest target
            closest_target, visible_targets = pp_functions.utils.closest_target(visible_targets, visible_dists)
                
            #reset targets for new lap
            if (len(targets) > 0
            and len(non_passed_targets) == 0 
            and track == True 
            and (right_spline_linked == True or left_spline_linked == True)):
                
                targets, non_passed_targets = pp_functions.utils.reset_targets(targets, non_passed_targets)
            
            
            #automatic steering
            if (len(visible_targets) > 0 
            and np.linalg.norm(closest_target.position-car.position) < car.fov/ppu
            and np.linalg.norm(closest_target.position-car.position) > 20/ppu
            and car.auto == True 
            and closest_target.passed == False):
                
                dist = closest_target.dist_car
                alpha = closest_target.alpha
                car.steering = (car.max_steering*2/np.pi)*np.arctan(alpha/dist**car.turning_sharpness)
                car.velocity.x = cruising_speed

            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            
            #clipping car steering angle
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            #computing boundary estimation
            left_spline, \
            right_spline, \
            first_left_cone_found, \
            left_spline_linked, \
            first_right_cone_found, \
            right_spline_linked, \
            first_visible_left_cone, \
            first_visible_right_cone = pp_functions.boundary_midpoints_splines.compute_boundaries(car,
                                                                                                  left_cones,
                                                                                                  right_cones,
                                                                                                  visible_left_cones,
                                                                                                  visible_right_cones,
                                                                                                  first_left_cone_found,
                                                                                                  first_right_cone_found,
                                                                                                  new_visible_left_cone_flag,
                                                                                                  new_visible_right_cone_flag,
                                                                                                  left_spline_linked,
                                                                                                  right_spline_linked,
                                                                                                  track,
                                                                                                  left_spline,
                                                                                                  right_spline,
                                                                                                  first_visible_left_cone,
                                                                                                  first_visible_right_cone)        
                 
            #compute midpoint path
            targets, \
            non_passed_targets, \
            target_locations = pp_functions.boundary_midpoints_splines.generate_midpoint_path(car,
                                                                                              Target,
                                                                                              targets,
                                                                                              non_passed_targets,
                                                                                              target_locations,
                                                                                              ppu,
                                                                                              visible_left_cones,
                                                                                              visible_right_cones,
                                                                                              car_angle,
                                                                                              new_visible_left_cone_flag,
                                                                                              new_visible_right_cone_flag)
                    
            #Setting the finishing line/point
            if first_visible_left_cone != 0 and first_visible_right_cone != 0 and midpoint_created == False:     
                start_midpoint_x = np.mean([first_visible_left_cone.position.x,first_visible_right_cone.position.x])
                start_midpoint_y = np.mean([first_visible_left_cone.position.y,first_visible_right_cone.position.y])     
                midpoint_created = True
                
                
            #incrementing lap number by 1
            elif (first_visible_left_cone != 0 
            and first_visible_right_cone != 0 
            and np.linalg.norm((start_midpoint_x, start_midpoint_y)-car.position) < 20/ppu 
            and track_number_changed == False 
            and track == True):
                track_number += 1
                print('TIME : ', time.time() - time_start_track)
                lap_reward = True
                track_number_changed = True
                
            #setting track_number_changed to false when not on finishing line
            elif (first_visible_left_cone != 0 
            and first_visible_right_cone != 0 
            and np.linalg.norm((start_midpoint_x, start_midpoint_y)-car.position) > 20/ppu 
            and track == True):
                track_number_changed = False

            #if car hits a cone, it crashes and the simulation ends
            pp_functions.car_crash.car_crash_mechanic(self, left_cones, right_cones, car, time_start_sim)
                    
            #reward function
            reward, lap_reward = pp_functions.reward_function.calculate_reward(lap_reward, car, track_number, dt)
            self.total_reward += reward
            
                
            #if the track number = 3, end simulation
            if (first_visible_left_cone != 0 
            and first_visible_right_cone != 0 
            and np.linalg.norm((start_midpoint_x, start_midpoint_y)-car.position) < 20/ppu 
            and track_number == 3 
            and track == True):
                print('FINISHED!', 'TIME : ', time.time() - time_start_track)
                print('TOTAL REWARD:', self.total_reward)
                self.exit = True
                track_number_changed = True     
                    
            # Logic
            car.update(dt)
            
            for target in targets:
                target.update(car, time_running, ppu, car_angle)
                
            for left_cone in left_cones:
                left_cone.update(car, time_running, ppu, car_angle)
                
            for right_cone in right_cones:
                right_cone.update(car, time_running, ppu, car_angle)

            # Drawing
            pp_functions.drawing.render(self,
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
                                        car_angle)
            
        pygame.quit()

if __name__ == '__main__':
    sim = PathPlanning()
    sim.run()