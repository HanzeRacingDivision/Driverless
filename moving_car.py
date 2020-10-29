import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
import cv2 as cv


breaks = True


class Car:
    def __init__(self, x, y, angle=0.0, length=4, max_steering=60, max_acceleration=6.0):
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
        print(img.shape)
        
        image_path1 = os.path.join(current_dir, "cone_s.png")
        cone_image = pygame.image.load(image_path1)
        
        car = Car(3,10)
        ppu = 32
        time_start = time.time()

        cone1 = Cone(23,13)
      #  cone2 = Cone(15,8)
        cone3 = Cone(35,7)
        #cone4 = Cone(20,6)
        
        alpha = 0
        beta = 0
        circles = []
        dist = 0
        
        while not self.exit:
            
            dt = self.clock.get_time() / 500

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            # User input
            pressed = pygame.key.get_pressed()

            if pressed[pygame.K_UP]:
                if car.velocity.x < 0:
                    car.acceleration = car.brake_deceleration
                else:
                    car.acceleration += 1 * dt
            elif pressed[pygame.K_DOWN] and breaks == True:
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
                
                
            if np.linalg.norm(cone1.position-car.position) <= 50/ppu and time_running > 2: 
                cone1.passed = True
                
            if np.linalg.norm(cone3.position-car.position) <= 50/ppu and time_running > 2: 
                cone3.passed = True
                
                
            #manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt
                
            elif np.linalg.norm(cone1.position-car.position) < car.fov/ppu and np.linalg.norm(cone1.position-car.position) > 50/ppu and time_running > 2 and cone1.passed == False:
             
                a_b = cone1.position-car.position
                dist = np.linalg.norm(cone1.position-car.position)
                a = a_b.x
                b = -1*a_b.y      
                
                beta = np.arctan(b/a)*(180/np.pi)
                beta = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)

                alpha = beta - car_angle
                
                if alpha > -90 and alpha < 90:  #if alpha is 'behind car' move on
                    car.steering = (120/np.pi)*np.arctan(alpha/dist**1.5)
                    car.velocity.x = 1
                   # continue
                #car.steering = 0

            elif np.linalg.norm(cone3.position-car.position) < car.fov/ppu and np.linalg.norm(cone3.position-car.position) > 50/ppu and time_running > 2 and cone3.passed == False:
               
                a_b = cone3.position-car.position
                dist = np.linalg.norm(cone3.position-car.position)
                a = a_b.x
                b = -1*a_b.y       
               
                beta = np.arctan(b/a)*(180/np.pi)
                beta = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
                        
                alpha = beta - car.angle
                
                if alpha > -90 and alpha < 90:
                   car.steering = (120/np.pi)*np.arctan(alpha/dist**1.5)
                   car.velocity.x = 1
                
            else:
                car.steering = 0

            
            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            # Logic
            car.update(dt)

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
            
            self.screen.blit(cone_image, cone1.position * ppu - (rect.width / 2, rect.height / 2))
      #      self.screen.blit(cone_image, cone2.position * ppu - (rect.width / 2, rect.height / 2))
            self.screen.blit(cone_image, cone3.position * ppu - (rect.width / 2, rect.height / 2))
        #    self.screen.blit(cone_image, cone4.position * ppu - (rect.width / 2, rect.height / 2))
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2)+ round(img.shape[1]/2),( rect.height / 2) + round(img.shape[0]/2)))
            

            pygame.draw.circle(self.screen,(255,255,255), (pos_1,pos_2), car.fov, 1)
            
            
            score_font = pygame.font.Font(None, 30)
            score_surf = score_font.render(f'Angle to cone : {alpha}', 1, (255, 255, 255))
            score_pos = [10, 10]
            self.screen.blit(score_surf, score_pos)
            
            score_surf = score_font.render(f'Car angle : {car_angle}', 1, (255, 255, 255))
            score_pos = [10, 30]
            self.screen.blit(score_surf, score_pos)
            
            score_surf = score_font.render(f'raw angle to cone : {beta}', 1, (255, 255, 255))
            score_pos = [10, 50]
            self.screen.blit(score_surf, score_pos)
            
            score_surf = score_font.render(f'steering : {car.steering}', 1, (255, 255, 255))
            score_pos = [10, 70]
            self.screen.blit(score_surf, score_pos)

            score_surf = score_font.render(f'distance to cone : {dist}', 1, (255, 255, 255))
            score_pos = [10, 90]
            self.screen.blit(score_surf, score_pos)
            
            score_surf = score_font.render(f'Cone passed: {cone1_passed}', 1, (255, 255, 255))
            score_pos = [10, 110]
            self.screen.blit(score_surf, score_pos)

# =============================================================================
#             score_surf = score_font.render(f'a : {a}, b : {b}', 1, (255, 255, 255))
#             score_pos = [10, 50]
#             self.screen.blit(score_surf, score_pos)
# =============================================================================
                        

            pygame.display.flip()
            
            self.clock.tick(self.ticks)
            

        pygame.quit()


if __name__ == '__main__':
    game = Game()
    game.run()
    

# =============================================================================
#                 if a > 0:
#                     if b > 0:
#                         continue
#                     else:
#                         beta = -1*beta
#                 else:
#                     if b > 0:
#                         beta = 180 - beta
#                     else:
#                         beta = beta - 180
# =============================================================================