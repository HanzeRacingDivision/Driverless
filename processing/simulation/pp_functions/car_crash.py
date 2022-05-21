import os
import pygame
from math import sin, radians, degrees, copysign
from pygame.math import Vector2
import time
import numpy as np
from PIL import Image, ImageDraw
from scipy.interpolate import splprep, splev
import pandas as pd

# Car crash mechanic
def car_crash_mechanic(self, left_cones, right_cones, car, time_start_sim):
    if len(left_cones) > 0 or len(right_cones):
        car_crashed = False
        
        #checking left cones for crash
        for i in range(len(left_cones)):
            if np.linalg.norm(tuple(x-y for x,y in zip([car.true_position.x, car.true_position.y], [left_cones[i].true_position.x, left_cones[i].true_position.y]))) < 0.4:
                car_crashed = True
                break
            
        #crashing right cones
        if car_crashed == False:
            for i in range(len(right_cones)):
                if np.linalg.norm(tuple(x-y for x,y in zip([car.true_position.x, car.true_position.y], [right_cones[i].true_position.x, right_cones[i].true_position.y]))) < 0.4:
                    car_crashed = True
                    break
                
    # =============================================================================
    #     #checking left_spline for crash
    #     if car_crashed == False and left_spline != 0:
    #         for i in range(len(left_spline[0])):
    #             if np.linalg.norm(tuple(x-y for x,y in zip([car.position.x, car.position.y], [left_spline[0][i],left_spline[1][i]]))) < 0.25:
    #                 car_crashed = True
    #                 break                
    #  
    #     #checking right_spline for crash
    #     if car_crashed == False and right_spline != 0:
    #         for i in range(len(right_spline[0])):
    #             if np.linalg.norm(tuple(x-y for x,y in zip([car.position.x, car.position.y], [right_spline[0][i],right_spline[1][i]]))) < 0.25:
    #                 car_crashed = True
    #                 break        
    # 
    # =============================================================================
                                    
               
        if car_crashed == True:
            print('CAR CRASHED!!')
            print('TIME : ', time.time() - time_start_sim)
            self.exit = True
            #change car image to explosion 