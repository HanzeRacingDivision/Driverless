import os
from turtle import pos
from xml.sax.handler import DTDHandler
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
        self.true_position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.true_angle = angle

        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 5
        self.brake_deceleration = 4
        self.free_deceleration = 1
        self.angular_velocity = 0

        self.acceleration = 0.0
        self.steering = 0.0
        self.fov = 175 #150
        self.turning_sharpness = 1.8
        self.breaks = True
        self.fov_range = 60
        self.auto = False
        self.slam = True
        self.headlights = False

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        #updating true position and angle
        self.true_position += self.velocity.rotate(-self.true_angle) * dt
        self.true_angle += degrees(self.angular_velocity) * dt  

        #updating estimated position and angle
        if self.slam == False:
            self.position += self.velocity.rotate(-self.angle) * dt
            self.angle += degrees(self.angular_velocity) * dt   

        #adding some noise/sensor error to the estimated position and angle
        self.position += (np.random.normal(loc = 0, scale = 1e-10) , np.random.normal(loc = 0, scale= 1e-10))
        self.angle += np.random.normal(loc = 0, scale = 1e-10)
        
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
    def __init__(self, x, y, category, left_cones, right_cones):
        self.position = Vector2(x + np.random.normal(loc = 0, scale = 1e-10), y + np.random.normal(loc = 0, scale = 1e-10))
        self.true_position = Vector2(x,y)
        self.visible = False   
        self.in_fov = False
        self.category = category
        self.dist_car = 10**10
        self.id = len(left_cones) + len(right_cones) + 2
        
    def update(self, car, time_running, ppu, car_angle): 
        
        #distance to car
        self.dist_car = np.linalg.norm(self.position-car.position)
        self.true_dist_car = np.linalg.norm(self.true_position-car.true_position)
        
        #calculating angle between car angle and cone
        if np.linalg.norm(self.true_position-car.true_position) < car.fov/ppu:
            
            a_b = self.true_position-car.true_position
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
                self.visible = False #commenting this line allows cones to be remembered
                self.in_fov = False

        else:
            pass
            self.visible = False  #commenting this line allows cones to be remembered
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

        self.mu = np.zeros(500)
        self.mu[0] = 15
        self.mu[1] = 3

        self.cov = np.zeros((500,500))
        for i in range(3, 500):
            self.cov[i,i] = 10**10

        self.u = np.zeros(3)
        self.Rt = np.array([1e-6,1e-6,0])
        self.obs = []
        self.c_prob = []
        self.Qt = np.zeros((2,2))

    def update_slam_vars(self, visible_left_cones, visible_right_cones, car):
        '''
        This function updates the variables necessary to run SLAM 
            - obs
            - c_prob
            - Qt
            - u
        '''
        #updating obs, c_prob, and Qt
        cone_dists = []
        cone_angles = []
        self.obs = []
        self.c_prob = np.ones(len(self.mu))
        for i in range(len(visible_left_cones)):
            observed_car_dist = visible_left_cones[i].true_dist_car + np.random.normal(loc = 0, scale = 1e-10)
            observed_alpha = visible_left_cones[i].alpha + np.random.normal(loc = 0, scale = 1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_left_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)

        for i in range(len(visible_right_cones)):
            observed_car_dist = visible_right_cones[i].true_dist_car + np.random.normal(loc = 0, scale = 1e-10)
            observed_alpha = visible_right_cones[i].alpha + np.random.normal(loc = 0, scale = 1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_right_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)
        
        if cone_dists and cone_angles:
            self.Qt[0,0] = np.std(cone_dists)**2
            self.Qt[1,1] = np.std(cone_angles)**2 
        self.u = [car.velocity.x, car.angular_velocity, 0]

    def EKF_predict(self, dt):
        '''
        The prediction step of the Extended Kalman Filter
        '''
        n_landmarks = len(self.mu) - 3

        # This function can possibly be replaced by the information from the car itself,
        # such that we have a new x, y and thetha of the vehicle.

        # Define motion model f(mu,u)
        [dtrans, drot1, drot2] = self.u

        motion = np.array([[dt * dtrans * np.cos(radians(self.mu[2]))], #change in x coordinate 
                           [dt * dtrans * -np.sin(radians(self.mu[2]))],   #change in y coordinate
                           [dt * degrees(drot1)]])                                  #change in theta


        # This matrix is used to apply the new motion results to the mu matrix (such that only the first 3 rows are updated)
        F = np.append(np.eye(3), np.zeros((3, n_landmarks)), axis=1)

        # Define motion model Jacobian
        # (derivative of the calculations for the motion.
        J = np.array([[0, 0, dt * dtrans * -np.sin(radians(self.mu[2]))],
                      [0, 0, dt * dtrans * -np.cos(radians(self.mu[2]))],
                      [0, 0, 0]])

        # create the G matrices (see documentation in the drive)
        G = np.eye(n_landmarks+3) + (F.T).dot(J).dot(F)

        # Predict new state
        self.mu = self.mu + (F.T).dot(motion)[:,0]

        # Predict new covariance
        R_t = np.zeros((n_landmarks + 3,n_landmarks + 3))
        R_t[0,0] = self.Rt[0]
        R_t[1,1] = self.Rt[1]
        R_t[2,2] = self.Rt[2]
        self.cov = G.dot(self.cov).dot(G.T) + R_t

        #print('Predicted location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu_bar[0][0], mu_bar[1][0],
        #                                                                               mu_bar[2][0])


    def EKF_update(self,car, left_cones, right_cones):
        '''
        The update step of the Extended Kalman Filter
        Threshold for observed before is currently 1e6 (this number has to be high, as the uncertainty
        has been set to a high number for un
        threshold for static landmark is c_prob >= 0.5. Maybe we can assume all landmarks are static
        '''

        N = len(self.mu)
        for [r, theta, j] in self.obs:
            j = int(j)
            if self.cov[2 * j + 3, 2 * j + 3] >= 1e6 and self.cov[2 * j + 4, 2 * j + 4] >= 1e6:
                # define landmark estimate as current measurement
                # aka, use the distance and angle to calculate its x and y location on the map.
                self.mu[2 * j + 3] = self.mu[0] + r * np.cos(radians(theta) + radians(self.mu[2]))
                self.mu[2 * j + 4] = self.mu[1] + r * -np.sin(radians(theta) + radians(self.mu[2]))

            # if landmark is static
            if self.c_prob[j] >= 0.5:
                # See documentation for more info here.

                # compute expected observation
                delta = np.array([self.mu[2 * j + 3] - self.mu[0], self.mu[2 * j + 4] - self.mu[1]])

                q = delta.T.dot(delta)

                sq = np.sqrt(q)

                z_theta = np.arctan2(delta[1], delta[0])

                z_hat = np.array([[sq], [z_theta - radians(-self.mu[2])]])
                # z_hat = h() function

                # This matrix is used to apply the calculations only to the relevant landmark and used to transfer
                # the low Jacobian into the high Jacobian
                F = np.zeros((5, N))
                F[:3, :3] = np.eye(3)
                # x location of cone j
                F[3, 2 * j + 3] = 1
                # y location of cone j
                F[4, 2 * j + 4] = 1

                # Do the partial diff equations and create the low jacobian
                H_z = np.array([[-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
                                [delta[1], -delta[0], -q, -delta[1], delta[0]]], dtype='float')
                # do the dot product of F to create the high Jacobian
                H = 1 / q * H_z.dot(F)

                # calculate Kalman gain
                K = self.cov.dot(H.T).dot(np.linalg.inv(H.dot(self.cov).dot(H.T) + self.Qt))

                # calculate difference between expected and real observation
                z_dif = np.array([[r], [radians(-theta)]]) - z_hat
                # normalize angular component!
                z_dif = (z_dif + np.pi) % (2 * np.pi) - np.pi

                # update state vector and covariance matrix
                self.mu = self.mu + K.dot(z_dif)[:,0]
                self.cov = (np.eye(N) - K.dot(H)).dot(self.cov)


        #using SLAM to update landmark + car positions
        car.position.x = self.mu[0]
        car.position.y = self.mu[1]
        car.angle = self.mu[2]

        for left_cone in left_cones:
            left_cone.position.x = self.mu[2 * left_cone.id + 3]
            left_cone.position.y = self.mu[2 * left_cone.id + 4]

        for right_cone in right_cones:
            right_cone.position.x = self.mu[2 * right_cone.id + 3]
            right_cone.position.y = self.mu[2 * right_cone.id + 4]
            
        #print('Updated location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu[0], mu[1], mu[2]))
        
    def slam(self, car, visible_left_cones, visible_right_cones, left_cones, right_cones, dt):
        self.update_slam_vars(visible_left_cones, visible_right_cones, car)
        self.EKF_predict(dt)
        self.EKF_update(car, left_cones, right_cones)

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
        self.mu[0] = car.position.x
        self.mu[1] = car.position.y

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
        cruising_speed = 0.75
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
        time_prev = None

        ticker = 0
         
        while not self.exit:
            
            if start_time_set == False:
                time_start_sim = time.time()
                start_time_set = True
             
            dt = self.clock.get_time()/500

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

            #redefining the true car angle so that it is in (-180, 180)
            temp_sign = np.mod(car.true_angle,360)
            if temp_sign > 180:
                car_angle_sign = -1
            else:
                car_angle_sign = 1
                
            true_car_angle = np.mod(car.true_angle,180)*car_angle_sign
            
            if true_car_angle < 0:
                true_car_angle = -180 - true_car_angle

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

                if car.velocity.x < cruising_speed:
                    car.acceleration = 0.1
                else:
                    car.velocity.x = cruising_speed
            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            
            #clipping car steering angle
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            #computing boundary estimation
            #left_spline, \
            #right_spline, \
            #first_left_cone_found, \
            #left_spline_linked, \
            #first_right_cone_found, \
            #right_spline_linked, \
            #first_visible_left_cone, \
            #first_visible_right_cone = pp_functions.boundary_midpoints_splines.compute_boundaries(car,
                                                                                                  #left_cones,
                                                                                                  #right_cones,
                                                                                                  #visible_left_cones,
                                                                                                  #visible_right_cones,
                                                                                                  #first_left_cone_found,
                                                                                                  #first_right_cone_found,
                                                                                                  #new_visible_left_cone_flag,
                                                                                                  #new_visible_right_cone_flag,
                                                                                                  #left_spline_linked,
                                                                                                  #right_spline_linked,
                                                                                                  #track,
                                                                                                  #left_spline,
                                                                                                  #right_spline,
                                                                                                  #first_visible_left_cone,
                                                                                                  #first_visible_right_cone)   

            if len(visible_left_cones) > 1 and car.auto == True and new_visible_left_cone_flag == True:
                if first_left_cone_found == False:
                    first_visible_left_cone = visible_left_cones[0]
                    first_left_cone_found = True


            if len(visible_right_cones) > 1 and car.auto == True and new_visible_right_cone_flag == True:
                if first_right_cone_found == False:
                    first_visible_right_cone = visible_right_cones[0]
                    first_right_cone_found = True
                                                                                                 
             # SLAM
            if car.slam == True and ticker % 4 == 0: #and round(time_running*100, 0) % 25 == 0:
                #print(round(time_running*10, 0))
                self.slam(car, visible_left_cones, visible_right_cones, left_cones, right_cones, dt)                                                                                   
                 
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
            
            #updating cones and targets
            for target in targets:
                target.update(car, time_running, ppu, car_angle)
                
            for left_cone in left_cones:
                left_cone.update(car, time_running, ppu, true_car_angle)
                
            for right_cone in right_cones:
                right_cone.update(car, time_running, ppu, true_car_angle)

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
                                        car_angle,
                                        dt)

            ticker += 1
            
        pygame.quit()

if __name__ == '__main__':
    sim = PathPlanning()
    sim.run()