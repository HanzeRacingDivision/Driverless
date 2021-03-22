import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev



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
        self.category = category
        
    def update(self, car, time_running, ppu, car_angle): 
        
        #distance to car
        self.dist_car = np.linalg.norm(self.position-car.position)
        
        #calculating angle between car angle and cone
        if np.linalg.norm(self.position-car.position) < car.fov/ppu and self.visible == False and car.auto == True:
            
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
            else:
                pass
                # self.visible = False #commenting this line allows cones to be remembered
        else:
            pass
           # self.visible = False  #commenting this line allows cones to be remembered


class PathPlanning:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Car")
        width = 1280
        height = 720
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False

    def run(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "car_r_30.png")
        car_image = pygame.image.load(image_path)
        
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

        car = Car(5,11)

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
        track_number = 0
        cruising_speed = 2.5
        
        
        def draw_line_dashed(surface, color, start_pos, end_pos, width = 1, dash_length = 10, exclude_corners = True):

            'simply a function that draws dashed lines in pygame'    
        
            # convert tuples to numpy arrays
            start_pos = np.array(start_pos)
            end_pos   = np.array(end_pos)
        
            # get euclidian distance between start_pos and end_pos
            length = np.linalg.norm(end_pos - start_pos)
        
            # get amount of pieces that line will be split up in (half of it are amount of dashes)
            dash_amount = int(length / dash_length)
        
            # x-y-value-pairs of where dashes start (and on next, will end)
            dash_knots = np.array([np.linspace(start_pos[i], end_pos[i], dash_amount) for i in range(2)]).transpose()
        
            return [pygame.draw.line(surface, color, tuple(dash_knots[n]), tuple(dash_knots[n+1]), width)
                    for n in range(int(exclude_corners), dash_amount - int(exclude_corners), 2)]    

        
        while not self.exit:
            
            dt = self.clock.get_time() / 500

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True
             

            # User input
            pressed = pygame.key.get_pressed()
            
            
            
            #If t pressed then create target
            if pressed[pygame.K_t]:
                    mouse_pos = pygame.mouse.get_pos()
                    
                    if mouse_pos in mouse_pos_list:
                        continue
                    else:
                        make_target = True
                        for i in range(len(mouse_pos_list)):
                            if np.linalg.norm(tuple(x-y for x,y in zip(mouse_pos_list[i],mouse_pos))) < 25:
                                make_target = False
                                break
                        
                        if make_target == True:
                            
                            target = Target(mouse_pos[0]/ppu,mouse_pos[1]/ppu)
                            targets.append(target)
                            non_passed_targets.append(target)
                            
                            mouse_pos_list.append(mouse_pos)
                        
                        
            # press l for left cone
            if pressed[pygame.K_l]:
                mouse_pos = pygame.mouse.get_pos()
                
                if mouse_pos in mouse_pos_list:
                    continue
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
                mouse_pos = pygame.mouse.get_pos()
                
                if mouse_pos in mouse_pos_list:
                    continue
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
                
                
            #if s pressed then set to track mode
            for event in events:
                if event.type == pygame.KEYUP and event.key == pygame.K_s: 
                    if track == False:
                        track = True
                    else:
                        track = False

                
                    
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
                    
            
            #define closest target
            if len(visible_targets) > 0:
                if len(visible_dists) == 0:
                    visible_targets = []
                else:
                    closest_target = visible_targets[np.array(visible_dists).argmin()]
                #set up while loop here to find next target
            
                
            #if currently no targets left and is a track, set all targets to non-passed and continue
            if len(targets) > 0 and len(non_passed_targets) == 0 and track == True and (right_spline_linked == True or left_spline_linked == True):
                track_number += 1
                non_passed_targets = targets.copy()
                for target in targets:
                    target.passed = False


            #setting car.position for angle calculations
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            car.position = car.position# - (rect.width / (2*ppu), rect.height /(2*ppu)) - (10/ppu,0)
            
            
            #manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt
             
            #automatic steering
            
            elif len(visible_targets) > 0 and np.linalg.norm(closest_target.position-car.position) < car.fov/ppu and np.linalg.norm(closest_target.position-car.position) > 20/ppu and car.auto == True and closest_target.passed == False:
                
                dist = closest_target.dist_car
                alpha = closest_target.alpha
                car.steering = (car.max_steering*2/np.pi)*np.arctan(alpha/dist**car.turning_sharpness)
                car.velocity.x = cruising_speed
                
            else:
                if(car.steering > (50 * dt)):
                    car.steering -= 120 * dt
                elif(car.steering < -(50 * dt)):
                    car.steering += 120 * dt
                else:
                    car.steering = 0

            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))
            
            
            #cubic splines for left track boundaries
            
            if len(visible_left_cones) > 1 and car.auto == True and new_visible_left_cone_flag == True:
                x = []
                y = []
                for left_cone in visible_left_cones:
                    x_temp = left_cone.position.x
                    y_temp = left_cone.position.y
                    
                    x.append(x_temp)
                    y.append(y_temp)
                    
                if len(visible_left_cones) == 2:
                    K = 1
                elif len(visible_left_cones) == 3:
                    K = 2
                else:
                    K = 2
                    
                if len(visible_left_cones) == len(left_cones) and track == True and left_spline_linked == False:
                    x.append(x[0])
                    y.append(y[0])
                    left_spline_linked = True
                    
                tck, u = splprep([x,y], s=0, k = K)
                unew = np.arange(0, 1.01, 0.25/(len(x)**1.2)) #more cones  = less final var
                left_spline = splev(unew, tck)
                

            #cubic splines for right track boundaries

            if len(visible_right_cones) > 1 and car.auto == True and new_visible_right_cone_flag == True:
                x = []
                y = []
                for right_cone in visible_right_cones:
                    x_temp = right_cone.position.x
                    y_temp = right_cone.position.y
                    
                    x.append(x_temp)
                    y.append(y_temp)
                
                if len(visible_right_cones) == 2:
                    K = 1
                elif len(visible_right_cones) == 3:
                    K = 2
                else:
                    K = 2
                    
                if len(visible_right_cones) == len(right_cones) and track == True and right_spline_linked == False:
                    x.append(x[0])
                    y.append(y[0])
                    right_spline_linked = True
                    
                tck, u = splprep([x,y], s=0, k = K)
                unew = np.arange(0, 1.01, 0.25/(len(x)**1.2)) #more cones  = less final var
                right_spline = splev(unew, tck)
                
                
            # auto generate path based on splines/cones
            
            if len(visible_left_cones) > 1 and len(visible_right_cones) > 1 and (new_visible_right_cone_flag == True or new_visible_left_cone_flag == True): #track_number == 0 and
              
                path_midpoints_x = []#[car.position.x]
                path_midpoints_y = []#[car.position.y]
                
                for left_cone in visible_left_cones:
                    for right_cone in visible_right_cones:
                        if np.linalg.norm((left_cone.position.x - right_cone.position.x, left_cone.position.y - right_cone.position.y)) < 4:
                            path_midpoints_x.append(np.mean([left_cone.position.x,right_cone.position.x]))
                            path_midpoints_y.append(np.mean([left_cone.position.y,right_cone.position.y]))
                            
                path_midpoints = [path_midpoints_x,path_midpoints_y]
                
                path_midpoints_visible_x = []  
                path_midpoints_visible_y = []      
                path_to_sort = []
                
                #find all 'visible' midpoints - this may be inefficient
                for i in range(len(path_midpoints[0])):
                  #  print(i)
                    dist_car = np.linalg.norm(Vector2(path_midpoints[0][i],path_midpoints[1][i])-car.position)
            
                    #calculating angle between car angle and midpoint
                    if dist_car < car.fov/ppu:
                        
                        a_b = Vector2(path_midpoints[0][i],path_midpoints[1][i])-car.position
                        a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
                        
                        rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                            [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
                        
                        a_b = rotate*a_b
                        
                        a = a_b[0]
                        b = a_b[1]
                        
                        beta = np.arctan(b/a)*(180/np.pi)
                        alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
                        alpha = alpha[0,0]
                        
                        #if target within car fov, set to visible
                        if np.abs(alpha) < car.fov_range:
                            path_to_sort.append([dist_car, path_midpoints[0][i], path_midpoints[1][i]])
                            
                #ordering the path_midpoints by distance from car            
                path_to_sort.sort()
                if len(path_to_sort) > 1:
                    for i in range(len(path_to_sort)):
                        #if statement making sure we have no duplicate co-ordinates
                        if path_to_sort[i][1] in path_midpoints_visible_x or path_to_sort[i][2] in path_midpoints_visible_y:
                            pass
                        else:
                            path_midpoints_visible_x.append(path_to_sort[i][1])
                            path_midpoints_visible_y.append(path_to_sort[i][2])
                            
                            
                            
                path_midpoints_visible = [path_midpoints_visible_x, path_midpoints_visible_y]
               # path_midpoints_visible.sort()
                path_midpoints = path_midpoints_visible            
                
                if len(path_midpoints[0]) == 1:
                    path_midpoints = [[car.position.x ,path_midpoints[0][0]], [car.position.y, path_midpoints[1][0]]]
                    
                    tck, u = splprep(path_midpoints, s=1, k = 1)
                    unew = np.arange(0, 1.01, 0.5/(len(x)**0.4)) #more cones  = less final var
                    path_midpoints_spline = splev(unew, tck)
                    
                elif len(path_midpoints[0]) == 2:
                    tck, u = splprep(path_midpoints, s=1, k = 1)
                    unew = np.arange(0, 1.01, 0.5/(len(x)**0.4)) #more cones  = less final var
                    path_midpoints_spline = splev(unew, tck)
                    
                elif len(path_midpoints[0]) > 2:
                    tck, u = splprep(path_midpoints, s=1, k = 2)
                    unew = np.arange(0, 1.01, 0.5/(len(x)**0.4)) #more cones  = less final var
                    path_midpoints_spline = splev(unew, tck)
                    
                else:
                    path_midpoints_spline = []
                    
                    
                #maybe right a function that filters the spline so that they are an appropraite distance
                
                #then a function that creates targets,
                #and when passed they cant be updated
                if len(path_midpoints_spline) > 0:
                    for i in range(len(path_midpoints_spline[0])):
                        new_target_loc = [path_midpoints_spline[0][i], path_midpoints_spline[1][i]]
                        if new_target_loc in target_locations:
                            continue
                        else:
                            make_target = True
                            for j in range(len(target_locations)):
                                if np.linalg.norm(tuple(x-y for x,y in zip(target_locations[j],new_target_loc))) < 1:
                                    make_target = False
                                    break
                            
                            if make_target == True:                        
                                new_target = Target(new_target_loc[0], new_target_loc[1])
                                targets.append(new_target)
                                non_passed_targets.append(new_target)
                                target_locations.append(new_target_loc)
                 

            # Logic
            car.update(dt)
            
            for target in targets:
                target.update(car, time_running, ppu, car_angle)
                
            for left_cone in left_cones:
                left_cone.update(car, time_running, ppu, car_angle)
                
            for right_cone in right_cones:
                right_cone.update(car, time_running, ppu, car_angle)
            

            # Drawing

            self.screen.fill((0, 0, 0))
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            
            pos_temp = car.position * ppu
            pos_1 = int(pos_temp.x)
            pos_2 = int(pos_temp.y)
            
            circle = (pos_1,pos_2)
            circles.append(circle)
            
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
                image_rect = image.get_rect(center= (pos_1, pos_2))
    
                self.screen.blit(image, image_rect)        
                
            
            # draw dotted line of past car locations
            for i in range(len(circles)):
                pygame.draw.circle(self.screen,(155,155,155), circles[i], 1, 1)
                            
            
            # draw targets
            if len(targets) > 0:
                for target in targets:
                    if target in non_passed_targets:
                        pass
                        self.screen.blit(target_image, target.position * ppu - (3,3))
                    else:
                        pass
                        self.screen.blit(target_image_g, target.position * ppu - (3,3))
                    if target.visible == True and car.auto == True:
                        pass
                        draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , target.position * ppu , width = 1, dash_length = 10, exclude_corners = True)
              
            
            if len(left_cones) > 0:
                for left_cone in left_cones:
                    self.screen.blit(left_cone_image, left_cone.position * ppu - (3,3))


            if len(right_cones) > 0:
                for right_cone in right_cones:
                    self.screen.blit(right_cone_image, right_cone.position * ppu - (3,3))
                    
                    
            if left_spline != 0 and len(left_spline) > 0:
                for i in range(len(left_spline[0])):
                    self.screen.blit(left_spline_image, Vector2(left_spline[0][i],left_spline[1][i]) * ppu - (3,3))
                    
            if right_spline != 0 and len(right_spline) > 0:
             #   print(f'right spline : {right_spline}')
                for i in range(len(right_spline[0])):
                    self.screen.blit(right_spline_image, Vector2(right_spline[0][i],right_spline[1][i]) * ppu - (3,3))
            
           # if path_midpoints != 0 and len(path_midpoints) > 0:
             #  print(f'midpoints : {path_midpoints}')
           #    for i in range(len(path_midpoints[0])):
            #        self.screen.blit(target_image, Vector2(path_midpoints[0][i],path_midpoints[1][i]) * ppu - (3,3))
            
            
           # if path_midpoints_spline != 0 and len(path_midpoints_spline) > 0:
             #   for i in range(len(path_midpoints_spline[0])):
             #       self.screen.blit(target_image, Vector2(path_midpoints_spline[0][i],path_midpoints_spline[1][i]) * ppu - (3,3))
            
            
            # draw the car sprite
            #pygame.draw.rect(self.screen, (200,200,200), (car.position * ppu - ((rect.width / 2),(rect.height / 2)), (rect.width, rect.height))) #draws a little box around the car sprite (just for debug)
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2),(rect.height / 2))) #draw car
            
            # draw dotted lines between car and closest target
            if len(visible_targets) > 0 and car.auto == True:
                draw_line_dashed(self.screen, (155,255,255),(pos_1,pos_2) , closest_target.position * ppu , width = 2, dash_length = 10, exclude_corners = True)
            
        
            if fullscreen == False:
                text_font = pygame.font.Font(None, 30)
             #   text_surf = text_font.render(f'Angle to target : {round(alpha,1)}', 1, (255, 255, 255))
             #   text_pos = [10, 10]
             #   self.screen.blit(text_surf, text_pos)
                
                text_surf = text_font.render(f'Car angle : {round(car_angle,1)}', 1, (255, 255, 255))
                text_pos = [10, 15]
                self.screen.blit(text_surf, text_pos)
                
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
                    text_surf = text_font.render(f'Laps: {track_number}', 1, (255, 255, 255))
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
               
                text_surf = text_font.render('Press 1 and 2 to alter car cruising speed', 1, (155, 155, 155))
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
                
                text_surf = text_font.render('Press T to place target', 1, (155, 155, 155))
                text_pos = [10, 620]
                self.screen.blit(text_surf, text_pos)
                
                text_surf = text_font.render('Press CTRL + C to clear', 1, (155, 155, 155))
                text_pos = [10, 640]
                self.screen.blit(text_surf, text_pos)
                
                text_surf = text_font.render('Press S to make track', 1, (155, 155, 155))
                text_pos = [10, 680]
                self.screen.blit(text_surf, text_pos)
                
                text_surf = text_font.render('Press arrow keys to steer manually', 1, (155, 155, 155))
                text_pos = [10, 660]
                self.screen.blit(text_surf, text_pos)
            else:
                text_surf = text_font.render('Press F to exit Fullscreen', 1, (80, 80, 80))
                text_pos = [10, 690]
                self.screen.blit(text_surf, text_pos)
            
            
            pygame.display.flip()
            
            self.clock.tick(self.ticks)
            

        pygame.quit()


if __name__ == '__main__':
    sim = PathPlanning()
    sim.run()
    

    
    