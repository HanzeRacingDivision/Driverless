
import time
import numpy as np  #general math library

from Map import Map





class simCar(Map.Car):
    """a class that simulates the car
        (replace Map.Car object with this)"""
    def __init__(self):
        Map.Car.__init__(self) #init Car object
        
        self.acceleration = 0.7 #m/s^2
        self.steer_accel = 1.0 #radians/s
        
        self.min_velocity = -1
        self.max_velocity = 3
        
        self.lastSimUpdateTime = time.time()
    
    def sendSpeedAngle(self, speed, angle):
        """set desired speed and steering angle
            (fake version of carMCU class)"""
        ## if you wanted to simulate overshoot, or steering change velocity/acceleration, here is the place to do it
        ## for now, i'm lazy and i dont know how the real car handles anyway, so we'll just keep it super simple:
        self.desired_velocity = speed
        self.desired_steering = angle
    
    def getFeedback(self):
        """handle acceleration (both velocity and steering), to simulate getting sensor feedback from car
            (fake version of carMCU class)"""
        ## in the realCar class, speed and steering is updated in car.update(), but for the simulation we'll just update those here
        rightNow = time.time()
        dt = rightNow - self.lastSimUpdateTime
        
        if(abs(self.desired_velocity - self.velocity) > abs((self.acceleration * dt)/2)): #if there's still room to improve
            if(self.desired_velocity > self.velocity):
                self.velocity = self.velocity + (self.acceleration * dt)
            else:
                self.velocity = self.velocity - (self.acceleration * dt)
        #self.velocity = min(max(self.velocity, self.min_velocity), self.max_velocity)
        
        if(abs(self.desired_steering - self.steering) > abs((self.steer_accel * dt)/2)):
            if(self.desired_steering > self.steering):
                self.steering = self.steering + (self.steer_accel * dt)
            else:
                self.steering = self.steering - (self.steer_accel * dt)
        #self.steering = min(max(self.steering, -self.maxSteeringAngle), self.maxSteeringAngle)
        
        self.lastSimUpdateTime = rightNow
        