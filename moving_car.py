import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np

breaks = True

class Car:
    def __init__(self, x, y, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 20
        self.brake_deceleration = 10
        self.free_deceleration = 2

        self.acceleration = 0.0
        self.steering = 0.0

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
        image_path = os.path.join(current_dir, "car_r.png")
        car_image = pygame.image.load(image_path)
        
        image_path1 = os.path.join(current_dir, "cone.png")
        cone_image = pygame.image.load(image_path1)
        
        car = Car(10,10)
        ppu = 32
        time_start = time.time()

        cone1 = Cone(25,8)
        
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
                        
    
            #manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt
            else:
                car.steering = 0
                
                        
            
            #self driving part
            time_running = time.time() - time_start
# =============================================================================
#             if time_running < 1:
#                 car.velocity.x = 5
#                 car.steering += 10
#             else:
#                 pass
# =============================================================================
            
            if np.linalg.norm(cone1.position-car.position) < 500/ppu and np.linalg.norm(cone1.position-car.position) > 50/ppu and time_running > 2:
                
                car.velocity.x = 1
                car.steering += 5         
            
            
            
            
            #deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

            # Logic
            car.update(dt)

            # Drawing
            self.screen.fill((0, 0, 0))
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            self.screen.blit(cone_image, cone1.position * ppu - (rect.width / 2, rect.height / 2))
            self.screen.blit(rotated, car.position * ppu - (rect.width / 2, rect.height / 2))
            
            pos_temp = car.position * ppu - (rect.width / 2, rect.height / 2)
            pos_1 = int(pos_temp.x + 50)
            pos_2 = int(pos_temp.y + 30)
            pygame.draw.circle(self.screen,(255,255,255), (pos_1,pos_2), 500, 1)
            
            
            pygame.display.flip()

            self.clock.tick(self.ticks)
            
            
            
        pygame.quit()


if __name__ == '__main__':
    game = Game()
    game.run()