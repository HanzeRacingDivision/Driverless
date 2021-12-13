import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd
from ekf_slam import *


class Car:
    def __init__(self, x, y, angle=0, length=2, max_steering=80, max_acceleration=4.0):
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
        self.acceleration = 0.0
        self.steering = 0.0
        self.angular_velocity = 0
        self.fov = 250  # 150
        self.turning_sharpness = 1.8
        self.breaks = True
        self.fov_range = 60
        self.auto = False
        self.slam = False
        self.headlights = False

    def update(self, dt, car_angle):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.angle = car_angle + degrees(self.angular_velocity) * dt
        # print("car.angle:", self.angle)

       


class Target:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.passed = False
        self.dist_car = 10 ** 10
        self.alpha = 0
        self.visible = False

    def update(self, car, time_running, ppu, car_angle):

        # distance to car
        self.dist_car = np.linalg.norm(self.position - car.position)

        # if within 20 pixels of car, target has been 'passed' by the car
        if self.passed == False and np.linalg.norm(self.position - car.position) <= 20 / ppu:
            self.passed = True
            self.visible = False

        # calculating angle between car angle and target
        if np.linalg.norm(self.position - car.position) < car.fov / ppu:

            a_b = self.position - car.position
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))

            rotate = np.matrix([[np.cos(-car_angle * np.pi / 180), -1 * np.sin(-car_angle * np.pi / 180)],
                                [np.sin(-car_angle * np.pi / 180), np.cos(-car_angle * np.pi / 180)]])

            a_b = rotate * a_b

            a = a_b[0]
            b = a_b[1]

            beta = np.arctan(b / a) * (180 / np.pi)
            alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
            self.alpha = alpha[0, 0]

            # if the target is outside the car fov, it is no longer visible
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

        # distance to car
        self.dist_car = np.linalg.norm(self.position - car.position)

        # calculating angle between car angle and cone
        # (Jan) removed self.visible from the statement since otherwise cones are never set back to not visible
        if np.linalg.norm(self.position - car.position) < car.fov / ppu and (
                car.auto == True or car.slam == True):  # self.visible == False and car.auto == True:

            a_b = self.position - car.position
            a_b = np.transpose(np.matrix([a_b.x, -1 * a_b.y]))

            rotate = np.matrix([[np.cos(-car_angle * np.pi / 180), -1 * np.sin(-car_angle * np.pi / 180)],
                                [np.sin(-car_angle * np.pi / 180), np.cos(-car_angle * np.pi / 180)]])

            a_b = rotate * a_b

            a = a_b[0]
            b = a_b[1]

            beta = np.arctan(b / a) * (180 / np.pi)
            alpha = beta + 90 * (b / np.abs(b)) * np.abs((a / np.abs(a)) - 1)
            self.alpha = alpha[0, 0]

            # if cone within car fov, set to visible
            if np.abs(self.alpha) < car.fov_range:
                self.visible = True
            else:
                # pass
                self.visible = False  # commenting this line allows cones to be remembered
        else:
            # pass
            self.visible = False  # commenting this line allows cones to be remembered


class SLAM:
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

        image_path3 = os.path.join(current_dir, "left_cone_s.png")
        left_cone_image = pygame.image.load(image_path3)

        image_path4 = os.path.join(current_dir, "right_cone_s.png")
        right_cone_image = pygame.image.load(image_path4)

        image_path7 = os.path.join(current_dir, "slam_cone.png")
        slam_cone_image = pygame.image.load(image_path7)
        
        car = Car(15, 3)

        ppu = 32
        time_start = time.time()
        circles = []
        targets = []
        non_passed_targets = targets.copy()
        first_visible_left_cone = 0
        first_visible_right_cone = 0
        left_cones = []
        right_cones = []
        visible_left_cones = []
        visible_right_cones = []
        dist = 0
        mouse_pos_list = []
        fullscreen = False
        track_number = 0
        cruising_speed = 0.5
        track_number_changed = False
        # activate car.slam
        car.slam = False
        
        # ------------------------------------------------------|
        #                       SLAM.py: START                     |
        # ------------------------------------------------------|

        # redefining the car angle so that it is in (-180,180)
        temp_sign = np.mod(car.angle, 360)
        if temp_sign > 180:
            car_angle_sign = -1
        else:
            car_angle_sign = 1

        car_angle = np.mod(car.angle, 180) * car_angle_sign

        if car_angle < 0:
            car_angle = -180 - car_angle
        # State Vector [x y yaw]'
        xEst = np.vstack((car.position.x, car.position.y, radians((car_angle))))
        xTrue = np.vstack((car.position.x, car.position.y, radians((car_angle))))
        PEst = np.eye(STATE_SIZE)
        x_state = xEst
        # history
        hxEst = xEst
        hxTrue = xTrue
        hxDR = xTrue
        # cones used in SLAM.py later
        left_cones_slam = []
        right_cones_slam = []

        # if we use all true cone positions every step, make sure landmarks is only initialized once
        # reduntant once the visable cone thing works
        slam_landmark_maker = True

        min_pose_ang = 500
        max_pose_ang = 0

        # ------------------------------------------------------|
        #                       SLAM.py: END                       |
        # ------------------------------------------------------|

        def draw_line_dashed(surface, color, start_pos, end_pos, width=1, dash_length=10, exclude_corners=True):

            'simply a function that draws dashed lines in pygame'

            # convert tuples to numpy arrays
            start_pos = np.array(start_pos)
            end_pos = np.array(end_pos)

            # get euclidian distance between start_pos and end_pos
            length = np.linalg.norm(end_pos - start_pos)

            # get amount of pieces that line will be split up in (half of it are amount of dashes)
            dash_amount = int(length / dash_length)

            # x-y-value-pairs of where dashes start (and on next, will end)
            dash_knots = np.array([np.linspace(start_pos[i], end_pos[i], dash_amount) for i in range(2)]).transpose()

            return [pygame.draw.line(surface, color, tuple(dash_knots[n]), tuple(dash_knots[n + 1]), width)
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

            if cone_connect_list == True:
                map_file = pd.DataFrame({'Cone_Type': cone_type,
                                         'Cone_X': cone_x,
                                         'Cone_Y': cone_y,
                                         'Prev_Cone_Index': prev_cone_index,
                                         'Next_Cone_Index': next_cone_index})
            else:
                map_file = pd.DataFrame({'Cone_Type': cone_type,
                                         'Cone_X': cone_x,
                                         'Cone_Y': cone_y})

            map_file.to_csv(f'{name}.csv')

        def load_map(mouse_pos_list):

            left_cones = []
            right_cones = []
            print('LOAD MAP : ')
            name = input()

            map_path = os.path.join(current_dir, f"{name}.csv")
            map_file = pd.read_csv(map_path)

            for i in range(len(map_file.iloc[:, 0])):
                if map_file['Cone_Type'].iloc[i] == 'LEFT':

                    left_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'left')
                    left_cones.append(left_cone)
                    mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))

                else:
                    right_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'right')
                    right_cones.append(right_cone)
                    mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))
            return left_cones, right_cones, mouse_pos_list

        while not self.exit:

            dt = self.clock.get_time() / 500

            # Event queue
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.QUIT:
                    self.exit = True

            # User input
            pressed = pygame.key.get_pressed()

            # press l for left cone
            if pressed[pygame.K_l]:
                mouse_pos = pygame.mouse.get_pos()

                if mouse_pos in mouse_pos_list:
                    continue
                else:

                    make_cone = True
                    for i in range(len(mouse_pos_list)):
                        if np.linalg.norm(tuple(x - y for x, y in zip(mouse_pos_list[i], mouse_pos))) < 50:
                            make_cone = False
                            break

                    if make_cone == True:
                        left_cone = Cone(mouse_pos[0] / ppu, mouse_pos[1] / ppu, 'left')
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
                        if np.linalg.norm(tuple(x - y for x, y in zip(mouse_pos_list[i], mouse_pos))) < 50:
                            make_cone = False
                            break

                    if make_cone == True:
                        right_cone = Cone(mouse_pos[0] / ppu, mouse_pos[1] / ppu, 'right')
                        right_cones.append(right_cone)
                        mouse_pos_list.append(mouse_pos)

            # if CTRL + c then clear screen
            if pressed[pygame.K_LCTRL] and pressed[pygame.K_c]:
                # resetting most vars
                circles = []
                left_cones = []
                right_cones = []
                visible_left_cones = []
                visible_right_cones = []
                mouse_pos_list = []
                path_midpoints_spline = 0
                first_visible_left_cone = 0
                first_visible_right_cone = 0
                first_right_cone_found = False
                first_left_cone_found = False
                track_number_changed = False

            # if 2 is pressed, increasing cruising speed
            # if 1 is pressed, decrease cruising speed

            for event in events:
                if event.type == pygame.KEYDOWN and event.key == pygame.K_1:
                    cruising_speed -= 0.05
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_2:
                    cruising_speed += 0.05

            # if f pressed then toggle fullscreen
            for event in events:
                if event.type == pygame.KEYUP and event.key == pygame.K_f:
                    if fullscreen == False:
                        fullscreen = True
                    else:
                        fullscreen = False

            # if h pressed then toggle headlight
            for event in events:
                if event.type == pygame.KEYUP and event.key == pygame.K_h:
                    if car.headlights == False:
                        car.headlights = True
                    else:
                        car.headlights = False

            # if e is pressed activate slam
            for event in events:
                if event.type == pygame.KEYUP and event.key == pygame.K_e:
                    if car.slam == False:
                        car.slam = True
                    else:
                        car.slam = False

            # if S then save map
            if pressed[pygame.K_s]:
                save_map(left_cones, right_cones)

            # if D then load map
            if pressed[pygame.K_d]:
                # resetting most vars before loading
                targets = []
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
                # left_spline = 0
                # right_spline = 0
                path_midpoints_spline = 0
                first_visible_left_cone = 0
                first_visible_right_cone = 0
                first_right_cone_found = False
                first_left_cone_found = False
                track_number_changed = False

                left_cones, right_cones, mouse_pos_list = load_map(mouse_pos_list)

            # manual acceleration
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

            # redefining the car angle so that it is in (-180,180)
            temp_sign = np.mod(car.angle, 360)
            if temp_sign > 180:
                car_angle_sign = -1
            else:
                car_angle_sign = 1

            car_angle = np.mod(car.angle, 180) * car_angle_sign

            if car_angle < 0:
                car_angle = -180 - car_angle

            # setting car.position for angle calculations
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()
            car.position = car.position  # - (rect.width / (2*ppu), rect.height /(2*ppu)) - (10/ppu,0)

            # manual steering
            if pressed[pygame.K_RIGHT]:
                car.steering -= 50 * dt
            elif pressed[pygame.K_LEFT]:
                car.steering += 50 * dt

            # deceleration
            car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))
            car.steering = max(-car.max_steering, min(car.steering, car.max_steering))


            # Logic
            car_prev_pos_x = car.position.x
            car_prev_pos_y = car.position.y
            car.update(dt, car_angle)
            if dt != 0:
                linear_vel = (abs(car_prev_pos_x - car.position.x) + abs(car_prev_pos_y - car.position.y)) / dt
                angular_vel = radians(car_angle) / dt
            for idx, left_cone in enumerate(left_cones):
                left_cone.update(car, time_running, ppu, car_angle)
                # print(f"left [{idx}]: {left_cone.alpha if left_cone.visible else ''}")

            for idx, right_cone in enumerate(right_cones):
                right_cone.update(car, time_running, ppu, car_angle)
                # print(f"right [{idx}]: {right_cone.alpha if right_cone.visible else ''}")

            # make list of visible left cones
            len_visible_left_cones_old = len(visible_left_cones)

            visible_left_cones = []
            for left_cone in left_cones:
                if left_cone.visible == True:
                    visible_left_cones.append(left_cone)

            # tracking the number of cones at each iteration and creating a flag for when a new cone is detected
            len_visible_left_cones_new = len(visible_left_cones)
            new_visible_left_cone_flag = len_visible_left_cones_new != len_visible_left_cones_old

            # make list of visible right cones
            len_visible_right_cones_old = len(visible_right_cones)

            visible_right_cones = []
            for right_cone in right_cones:
                if right_cone.visible == True:
                    visible_right_cones.append(right_cone)

            # tracking the number of cones at each iteration and creating a flag for when a new cone is detected
            len_visible_right_cones_new = len(visible_right_cones)
            new_visible_right_cone_flag = len_visible_right_cones_new != len_visible_right_cones_old

            # ------------------------------------------------------|
            #                       SLAM.py: START                     |
            # ------------------------------------------------------|

            # if we use all landmarks initally, reduntant if visable cones work
            # if car.slam and slam_landmark_maker:
            #     for cone in right_cones:
            #         right_cones_slam.append([cone.position.x, cone.position.y])
            #     for cone in left_cones:
            #         left_cones_slam.append([cone.position.x, cone.position.y])
            #     landmarks = np.vstack((left_cones_slam, right_cones_slam))
            #     slam_landmark_maker = False
            #     print("Landmarks")
            #     print(landmarks)
            #     print(len(landmarks))
            # TODO
            # if E is pressed do SLAM.py at every step
            if car.slam:

                # get list of visble cones in right format + stack them as landmark input
                visible_right_cones_slam = []
                visible_left_cones_slam = []
                for cone in visible_right_cones:
                    visible_right_cones_slam.append([cone.position.x, cone.position.y])
                for cone in visible_left_cones:
                    visible_left_cones_slam.append([cone.position.x, cone.position.y])

                if visible_right_cones_slam != [] and visible_left_cones_slam != []:
                    landmarks = np.vstack((visible_left_cones_slam, visible_right_cones_slam))
                elif visible_right_cones_slam != []:
                    landmarks = np.asarray(visible_right_cones_slam)
                elif visible_left_cones_slam != []:
                    landmarks = np.asarray(visible_left_cones_slam)

                linear_velocity = car.velocity.x  # [m/s]

                # has to be negative for some reason
                angular_velocity = car.angular_velocity  # [rad/s]
                U = np.array([[linear_velocity, angular_velocity]]).T
                print("U", U)
                print("My vals: ", linear_vel, angular_vel)
                # Here we convert our landmark positions (visible cones) into measurements (dist, angle, num)
                # changed it so that we always use our estimated position to calculate the measurements
                if "landmarks" in dir():
                    xTrue, landmark_measurements, ud = observation(x_state, U, landmarks, dt)
                    xTrue = np.vstack([[car.position.x], [car.position.y], [radians(car_angle+180)]])
                    print("U", ud)
                    print('z', landmark_measurements)
                    print("Visible landmarks", landmarks)
                    # print("Input to SLAM.py", xEst, PEst, U, landmark_measurements, dt)
                    xEst, PEst = ekf_slam(xEst, PEst, U, landmark_measurements, dt)
                    
                    if xEst[2] > max_pose_ang:
                        max_pose_ang = xEst[2]
                    if xEst[2] < min_pose_ang:
                        min_pose_ang = xEst[2]
                    print("min max is", min_pose_ang, max_pose_ang)
                    x_state = xEst[0:STATE_SIZE]
                    # print("Slams estimate")
                    # print(x_state)
                    slam_landmark_est = xEst[STATE_SIZE + 1:len(xEst)]
                    # print('start')
                    # print(x_state)
                    # print(xTrue)
                    # print(landmarks)
                    # print(landmark_measurements)

                    # store data history
                    hxEst = np.hstack((hxEst, x_state))
                    hxTrue = np.hstack((hxTrue, xTrue))

            # ------------------------------------------------------|
            #                      SLAM.py: END                        |
            # ------------------------------------------------------|

            # Drawing

            self.screen.fill((0, 0, 0))
            rotated = pygame.transform.rotate(car_image, car.angle)
            rect = rotated.get_rect()

            pos_temp = car.position * ppu
            pos_1 = int(pos_temp.x)
            pos_2 = int(pos_temp.y)

            circle = (pos_1, pos_2)
            circles.append(circle)

            # draw headlights
            if car.headlights == True:
                pil_size = car.fov * 2

                pil_image = Image.new("RGBA", (pil_size, pil_size))
                pil_draw = ImageDraw.Draw(pil_image)
                # pil_draw.arc((0, 0, pil_size-1, pil_size-1), 0, 270, fill=RED)
                pil_draw.pieslice((0, 0, pil_size - 1, pil_size - 1), -car.angle - car.fov_range,
                                  -car.angle + car.fov_range, fill=(55, 55, 35))

                mode = pil_image.mode
                size = pil_image.size
                data = pil_image.tobytes()

                image = pygame.image.fromstring(data, size, mode)
                image_rect = image.get_rect(center=(pos_1, pos_2))

                self.screen.blit(image, image_rect)

                # draw dotted line of past car locations
            for i in range(len(circles)):
                pygame.draw.circle(self.screen, (155, 155, 155), circles[i], 1, 1)

            if len(left_cones) > 0:
                for left_cone in left_cones:
                    self.screen.blit(left_cone_image, left_cone.position * ppu - (3, 3))

            if len(right_cones) > 0:
                for right_cone in right_cones:
                    self.screen.blit(right_cone_image, right_cone.position * ppu - (3, 3))

            # car.slam cone drawing as you mgiht see smth is fucked
            if car.slam and "landmarks" in dir():
                if len(slam_landmark_est) > STATE_SIZE + 1:
                    i = 0
                    while (i < len(slam_landmark_est) - 1):
                        y = slam_landmark_est[i]
                        i += 1
                        x = slam_landmark_est[i]
                        i += 1
                        cone_pos = Vector2(x, y)
                        self.screen.blit(slam_cone_image, cone_pos * ppu - (3, 3))

            if first_visible_left_cone != 0 and first_visible_right_cone != 0:
                draw_line_dashed(self.screen, (255, 51, 0),
                                 (first_visible_left_cone.position.x * ppu, first_visible_left_cone.position.y * ppu),
                                 (first_visible_right_cone.position.x * ppu, first_visible_right_cone.position.y * ppu),
                                 width=2, dash_length=5, exclude_corners=True)
            if car.slam:
                pygame.draw.circle(self.screen, (200, 150, 150),
                                   (round(float(x_state[0]), 2) * ppu, round(float(x_state[1]), 2) * ppu), 5, 5)
                # self.screen.blit(rotated, (round(float(x_state[0]), 2))
                pygame.draw.circle(self.screen, (200, 150, 150),
                                   (round(float(x_state[0]), 2) * ppu, round(float(x_state[1]), 2) * ppu), 5, 5)
                tuple([round(float(x), 2) * ppu for x in x_state[0:2]])
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2), (rect.height / 2)))  # draw car

            # draw the car sprite
            # pygame.draw.rect(self.screen, (200,200,200), (car.position * ppu - ((rect.width / 2),(rect.height / 2)), (rect.width, rect.height))) #draws a little box around the car sprite (just for debug)
            self.screen.blit(rotated, car.position * ppu - ((rect.width / 2), (rect.height / 2)))  # draw car

            if fullscreen == False:
                text_font = pygame.font.Font(None, 30)
                #   text_surf = text_font.render(f'Angle to target : {round(alpha,1)}', 1, (255, 255, 255))
                #   text_pos = [10, 10]
                #   self.screen.blit(text_surf, text_pos)

                text_surf = text_font.render(f'Car angle : {round(car_angle, 1)}', 1, (255, 255, 255))
                text_pos = [10, 15]
                self.screen.blit(text_surf, text_pos)

                text_surf = text_font.render(f'Steering : {round(car.steering, 1)}', 1, (255, 255, 255))
                text_pos = [10, 35]
                self.screen.blit(text_surf, text_pos)

                text_surf = text_font.render(f'Speed : {round(car.velocity.x, 1)}', 1, (255, 255, 255))
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

            
                text_surf = text_font.render(f'True [X, Y, alpha]:'
                                             f' {[round(car.position.x, 2), round(car.position.y, 2), round(car_angle, 2)]}', 1,
                                             (255, 255, 255))
                text_pos = [10, 130]
                self.screen.blit(text_surf, text_pos)
                # SLAM.py pose print
                # TODO
                # ADD SLAM.py landmark prints (maybe with uncertainty radius)
                if car.slam:
                    text_surf = text_font.render(f'Est. [X, Y, alpha]:'
                                                 f' {[round(float(x_state[0]), 2), round(float(x_state[1]), 2), round(float(degrees(x_state[2])), 2)]}', 1,
                                                 (255, 255, 255))
                    text_pos = [10, 150]
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

                text_surf = text_font.render('Press E to start SLAM.py', 1, (155, 155, 155))
                text_pos = [10, 700]
                self.screen.blit(text_surf, text_pos)
            else:
                text_surf = text_font.render('Press F to exit Fullscreen', 1, (80, 80, 80))
                text_pos = [10, 690]
                self.screen.blit(text_surf, text_pos)

            pygame.display.flip()

            self.clock.tick(self.ticks)

        pygame.quit()


if __name__ == '__main__':
    # EKF state covariance
    # Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2
    #
    # #  Simulation parameter
    # Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
    # R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2
    #
    # MAX_RANGE = 60.0  # maximum observation range
    # M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
    # STATE_SIZE = 3  # State size [x,y,yaw]
    # LM_SIZE = 2  # LM state size [x,y]

    show_animation = True

    sim = SLAM()
    sim.run()
