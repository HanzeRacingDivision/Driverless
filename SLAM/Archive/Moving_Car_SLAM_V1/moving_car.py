import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw


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
    def __init__(self, x, y, angle = 0, length = 2.5, max_steering = 80, max_acceleration = 6.0):
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
        self.fov = 300 #150
        self.turning_sharpness = 1.4
        self.breaks = True
        self.fov_range = 60
        

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
        self.visible = False
        self.dist_car = 10**10
        self.alpha = 0
        self.valid = False
        
    def update(self, car, time_running, ppu, car_pos,car_angle): 
        self.dist_car = np.linalg.norm(self.position-car_pos)
        
        if self.passed == False and np.linalg.norm(self.position-car_pos) <= 30/ppu and time_running > 2: 
            self.passed = True
            self.valid = False
            
        if np.linalg.norm(self.position-car_pos) < car.fov/ppu and time_running > 2:
            self.visible = True
            
            a_b = self.position-car_pos
            a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
            
            rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
            
            a_b = rotate*a_b
            
            a = a_b[0]
            b = a_b[1]
            
            beta = np.arctan(b/a)*(180/np.pi)
            alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
            self.alpha = alpha[0,0]
            
            if np.abs(self.alpha) < 60 and self.passed == False:
                self.valid = True
            else:
                self.valid = False
        else:
            self.visible = False
            self.valid = False
        


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
        img = pygame.image.load(image_path)
        
        image_path1 = os.path.join(current_dir, "target_s.png")
        target_image = pygame.image.load(image_path1)
        
        image_path2 = os.path.join(current_dir, "target_sg.png")
        target_image_g = pygame.image.load(image_path2)

        #image_path2 = os.path.join(current_dir, "fin.png")
       # fin_image = pygame.image.load(image_path2)

        car = Car(7,14)
        ppu = 32
        time_start = time.time()

        
        targets = []
        
        non_passed_targets = targets.copy()
        
        alpha = 0
        circles = []
        dist = 0
        closest_target = None
        mouse_pos_list = []
        track = False
        
        

        
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
                        target = Target(mouse_pos[0]/ppu,mouse_pos[1]/ppu)
                        targets.append(target)
                        non_passed_targets.append(target)
                        
                    mouse_pos_list.append(mouse_pos)
            
            
            #if CTRL + c then clear screen
            if pressed[pygame.K_LCTRL] and pressed[pygame.K_c]:
                targets  = []
                non_passed_targets = []
                circles = []
                
                
                
            #if s pressed then set to track mode
            for event in events:
                if event.type == pygame.KEYUP and event.key == pygame.K_s: #and track_toggle_temp == True
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
                
                
            #list of visibile targets and passed targets
            visible_targets = []
            dists = []
            non_passed_dists = []
            valid_targets = []
            valid_dists = []


            
            #make list of visible targets and list of passed targets
            for target in targets:
                
                non_passed_dists.append(target.dist_car)
                dists.append(target.dist_car)
                
                if target.visible == True:
                    visible_targets.append(target)
                    
                if target.passed == True:
                    non_passed_dists.remove(target.dist_car)

                if target.valid == True:
                    valid_targets.append(target)
                    valid_dists.append(target.dist_car)
                    
            for target in non_passed_targets:
                if target.passed == True:
                    non_passed_targets.remove(target)
                     
                    
                    
            len_valid_targets = len(valid_targets)
            
            #define closest target
            if len(valid_targets) > 0:
                if len(valid_dists) == 0:
                    valid_targets = []
                else:
                    closest_target = valid_targets[np.array(valid_dists).argmin()]
                #set up while loop here to find next target
                

                
            else:
                #if currently no targets left and is a track, set all targets to non-passed and continue
                if track == True:
                    non_passed_targets = targets.copy()
                    for target in targets:
                        target.passed = False


            #setting car_pos for angle calculations
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            car_pos = car.position - (rect.width / (2*ppu), rect.height /(2*ppu)) - (10/ppu,0)
            
            
            #manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt
             
            #automatic steering
            
            elif len(valid_targets) > 0 and np.linalg.norm(closest_target.position-car_pos) < car.fov/ppu and np.linalg.norm(closest_target.position-car_pos) > 30/ppu and time_running > 2 and closest_target.passed == False:
                
                dist = closest_target.dist_car
                alpha = closest_target.alpha
                car.steering = (car.max_steering*2/np.pi)*np.arctan(alpha/dist**car.turning_sharpness)
                car.velocity.x = 3
                
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

            # Logic
            car.update(dt)
            for target in targets:
                target.update(car, time_running, ppu, car_pos, car_angle)
            

            # Drawing

            
            self.screen.fill((0, 0, 0))
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            
            pos_temp = car.position * ppu
            pos_1 = int(pos_temp.x)
            pos_2 = int(pos_temp.y)
            
            circle = (pos_1,pos_2)
            circles.append(circle)
            
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
                    
            # draw dotted line (histogram) of past car locations
            for i in range(len(circles)):
                pygame.draw.circle(self.screen,(155,155,155), circles[i], 1, 1)
            
            # draw targets
            if len(targets) > 0:
                for target in targets:
                    if target in non_passed_targets:
                        self.screen.blit(target_image, target.position * ppu - (9,10))
                    else:
                        self.screen.blit(target_image_g, target.position * ppu - (9,10))
                    if target.visible == True:
                        draw_line_dashed(self.screen, (150,150,150),(pos_1,pos_2) , target.position * ppu , width = 1, dash_length = 10, exclude_corners = True)
            
            
            # draw the car sprite
            #pygame.draw.rect(self.screen, (200,200,200), (car.position * ppu - ((rect.width / 2),(rect.height / 2)), (rect.width, rect.height))) #draws a little box around the car sprite (just for debug)
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2),(rect.height / 2))) #draw car
            
            # draw dotted lines between car and valid targets
            if len(valid_targets) > 0:
                draw_line_dashed(self.screen, (155,255,255),(pos_1,pos_2) , closest_target.position * ppu , width = 2, dash_length = 10, exclude_corners = True)
            
          #  pygame.draw.circle(self.screen,(255,255,255), (pos_1,pos_2), car.fov, 1)
            
            # draw strange grey rectangle behind debug text on the left
            pygame.draw.rect(self.screen,(100 - 10*dist, 100 - 10*dist, 100 - 10*dist),(10,180,12,dist*20 - 20)) 
            
        

            text_font = pygame.font.Font(None, 30)
            text_surf = text_font.render(f'Angle to target : {round(alpha,1)}', 1, (255, 255, 255))
            text_pos = [10, 10]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Car angle : {round(car_angle,2)}', 1, (255, 255, 255))
            text_pos = [10, 30]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Steering : {round(car.steering,2)}', 1, (255, 255, 255))
            text_pos = [10, 50]
            self.screen.blit(text_surf, text_pos)

            text_surf = text_font.render(f'Distance to target : {round(dist,2)}', 1, (255, 255, 255))
            text_pos = [10, 70]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Targets passed: {len(targets) - len(non_passed_targets)}', 1, (255, 255, 255))
            text_pos = [10, 110]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Number of targets: {len(targets)}', 1, (255, 255, 255))
            text_pos = [10, 130]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Track: {track}', 1, (255, 255, 255))
            text_pos = [10, 150]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press T to place target', 1, (155, 155, 155))
            text_pos = [10, 620]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press CTRL + C to clear', 1, (155, 155, 155))
            text_pos = [10, 640]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press S to make track', 1, (155, 155, 155))
            text_pos = [10, 680]
            self.screen.blit(text_surf, text_pos)
            
            text_surf = text_font.render(f'Press arrow keys to steer manually', 1, (155, 155, 155))
            text_pos = [10, 660]
            self.screen.blit(text_surf, text_pos)
            
            
            pygame.display.flip()
            
            self.clock.tick(self.ticks)
            

        pygame.quit()


if __name__ == '__main__':
    game = Game()
    game.run()
    