import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
import cv2 as cv


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


class Car:
    def __init__(self, x, y, angle=-180.0, length=4, max_steering=70, max_acceleration=6.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 7
        self.brake_deceleration = 4
        self.free_deceleration = 1

        self.acceleration = 0.0
        self.steering = 0.0
        self.fov = 500 #150
        self.turning_sharpness = 1.4
        self.breaks = True
        

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
        
        
class Cone:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.passed = False
        self.visible = False
        self.dist_car = 10**10
        
    def update(self, car, time_running, ppu): 
        self.dist_car = np.linalg.norm(self.position-car.position)
        
        if self.passed == False and np.linalg.norm(self.position-car.position) <= 30/ppu and time_running > 2: 
            self.passed = True
        if np.linalg.norm(self.position-car.position) < car.fov/ppu and time_running > 2:
            self.visible = True
        else:
            self.visible = False
        


class Game:
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
        image_path = os.path.join(current_dir, "car_r_s.png")
        car_image = pygame.image.load(image_path)
        img = cv.imread(image_path)
        
        image_path1 = os.path.join(current_dir, "cone_s.png")
        cone_image = pygame.image.load(image_path1)
        
        image_path2 = os.path.join(current_dir, "fin.png")
        fin_image = pygame.image.load(image_path2)

        car = Car(36,19)
        ppu = 32
        time_start = time.time()

        
        cones = []
        
        non_passed_cones = cones.copy()
        
        alpha = 0
        beta = 0
        circles = []
        dist = 0
        a = 0
        b = 0
        closest_cone = None
        mouse_pos_list = []
        
        while not self.exit:
            
            dt = self.clock.get_time() / 500

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True
             

            # User input
            pressed = pygame.key.get_pressed()
            
            
            if pressed[pygame.K_c]:
                    mouse_pos = pygame.mouse.get_pos()
                    
                    if mouse_pos in mouse_pos_list:
                        continue
                    else:
                        cone = Cone(mouse_pos[0]/ppu,mouse_pos[1]/ppu)
                        cones.append(cone)
                        non_passed_cones.append(cone)
                        
                    mouse_pos_list.append(mouse_pos)
                    
            if pressed[pygame.K_LCTRL] and pressed[pygame.K_c]:
                cones  = []
                non_passed_cones = []

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
            
            temp_sign = np.mod(car.angle,360)
            if temp_sign > 180:
                car_angle_sign = -1
            else:
                car_angle_sign = 1
                
            car_angle = np.mod(car.angle,180)*car_angle_sign
            
            if car_angle < 0:
                car_angle = -180 - car_angle
                
            #list of visibile cones and passed cones
            
            visible_cones = []
            dists = []
            non_passed_dists = []
            
            for cone in cones:
                non_passed_dists.append(cone.dist_car)
                dists.append(cone.dist_car)
                if cone.visible == True:
                    visible_cones.append(cone)
                    
                if cone.passed == True:
                    non_passed_dists.remove(cone.dist_car)
                
            for cone in non_passed_cones:
                if cone.passed == True:
                    non_passed_cones.remove(cone)
                    
            if len(non_passed_cones) > 0:
                closest_cone = non_passed_cones[np.array(non_passed_dists).argmin()]
            else:
                non_passed_cones = cones.copy()
                for cone in cones:
                    cone.passed = False

            #manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt
             
            #automatic steering
            elif len(cones) > 0 and np.linalg.norm(closest_cone.position-car.position) < car.fov/ppu and np.linalg.norm(closest_cone.position-car.position) > 30/ppu and time_running > 2 and closest_cone.passed == False:
                a_b = closest_cone.position-car.position
                dist = closest_cone.dist_car
                a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
                
                rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                    [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
                
                a_b = rotate*a_b
                
                a = a_b[0]
                b = a_b[1]
                
                beta = np.arctan(b/a)*(180/np.pi)
                alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
                alpha = alpha[0,0]

                car.steering = (140/np.pi)*np.arctan(alpha/dist**car.turning_sharpness)
                car.velocity.x = 3
                
                
            elif len(non_passed_cones) == 0:
                car.steering = -30
                car.free_deceleration = 1
            else:
                car.steering = 0

            
            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            # Logic
            car.update(dt)
            for cone in cones:
                cone.update(car, time_running, ppu)
            

            # Drawing
            self.screen.fill((0, 0, 0))
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            
            pos_temp = car.position * ppu - (rect.width / 2, rect.height / 2)
            pos_1 = int(pos_temp.x)
            pos_2 = int(pos_temp.y)
            
            circle = (pos_1,pos_2)
            circles.append(circle)
            
            for i in range(len(circles)):
                pygame.draw.circle(self.screen,(155,155,155), circles[i], 1, 1)
            
            if len(cones) > 0:
                for cone in cones:
                    self.screen.blit(cone_image, cone.position * ppu - (5,13))
                    if cone.visible == True:
                        draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , cone.position * ppu , width = 1, dash_length = 10, exclude_corners = True)
                    
            
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2)+ round(img.shape[1]/2),( rect.height / 2) + round(img.shape[0]/2)))
            
            if len(cones) > 0:
                draw_line_dashed(self.screen, (155,255,255),(pos_1,pos_2) , closest_cone.position * ppu , width = 2, dash_length = 10, exclude_corners = True)
            
            pygame.draw.circle(self.screen,(255,255,255), (pos_1,pos_2), car.fov, 1)
            
         #   if len(non_passed_cones) == 0:
          #      self.screen.blit(fin_image, (350,250))
            
            text_font = pygame.font.Font(None, 30)
            text_surf = text_font.render(f'Angle to cone : {alpha}', 1, (255, 255, 255))
            text_pos = [10, 10]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Car angle : {car_angle}', 1, (255, 255, 255))
            text_pos = [10, 30]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'steering : {car.steering}', 1, (255, 255, 255))
            text_pos = [10, 50]
            self.screen.blit(text_surf, text_pos)

            text_surf = text_font.render(f'distance to cone : {dist}', 1, (255, 255, 255))
            text_pos = [10, 70]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Cones passed: {len(cones) - len(non_passed_cones)}', 1, (255, 255, 255))
            text_pos = [10, 110]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'number of cones: {len(cones)}', 1, (255, 255, 255))
            text_pos = [10, 130]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press C to place cone', 1, (155, 155, 155))
            text_pos = [10, 640]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press CTRL+C to clear all cones', 1, (155, 155, 155))
            text_pos = [10, 660]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press arrow keys to steer manually', 1, (155, 155, 155))
            text_pos = [10, 680]
            self.screen.blit(text_surf, text_pos)
            
            
            pygame.display.flip()
            
            self.clock.tick(self.ticks)
            

        pygame.quit()


if __name__ == '__main__':
    game = Game()
    game.run()
    