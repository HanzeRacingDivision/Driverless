import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use


#this shouldn't be here (because this is intended only for CAR simulation code), but there's no better place for it (YET)
import time
def simClockExample(clockStart): #usage: 'mapObject.setClock(simClockExample)'  now, when mapObject.clock() is called, simClockExample(mapObject) will be called
    return((time.time()-clockStart)*2)



class simCar(Map.Car):
    """a class that simulates the car
        (replace Map.Car object with this)"""
    def __init__(self):
        Map.Car.__init__(self) #init Car object
        
        self.acceleration = 0.7 #m/s^2
        self.steer_accel = 1.5 #radians/s
        
        self.min_velocity = -1
        self.max_velocity = 3
        
        self.simulationVariables = None
    
    def update(self, dTime, dDist=None):
            """ update the position of the car, based on velocity, steering and time-passage """
            if(dDist is None):
                dDist = self.velocity*dTime
            
            #turning math
            if((abs(self.steering) > 0.001) and (abs(self.velocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                turning_radius = self.wheelbase/np.tan(self.steering)
                angular_velocity = self.velocity/turning_radius
                arcMov = angular_velocity * dTime
                
                #one way to do it
                # turning_center = GF.distAnglePosToPos(turning_radius, self.angle+(np.pi/2), self.position) #get point around which car turns
                # newPosition = GF.distAnglePosToPos(turning_radius, self.angle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long
                
                #another way of doing it
                forwardMov = np.sin(arcMov)*turning_radius #sin(arc)*turning radius = movement paralel with (old) carAngle
                lateralMov = turning_radius - (np.cos(arcMov)*turning_radius) #sin(arc)*turning radius = movement perpendicular to (old) carAngle
                movAngle = np.arctan2(lateralMov, forwardMov) #
                diagonalMov = forwardMov/np.cos(movAngle) #the length of a line between the start of the arc and the end of the arc
                newPosition = GF.distAnglePosToPos(diagonalMov, self.angle+movAngle, self.position)
                # newPosition[0] = self.position[0] + diagonalMov * np.cos(self.angle+movAngle) #same as using distAnglePosToPos
                # newPosition[1] = self.position[1] + diagonalMov * np.sin(self.angle+movAngle)
                
                #update position
                self.angle += arcMov
                self.position[0] = newPosition[0] #keep the numpy array object the same (instead of replacing it with a whole new array every time)
                self.position[1] = newPosition[1]
            else:
                # self.position[0] = self.position[0] + (dTime * self.velocity * np.cos(self.angle)) #for some reason += doesnt work at the start
                # self.position[1] = self.position[1] + (dTime * self.velocity * np.sin(self.angle))
                self.position[0] = self.position[0] + (dDist * np.cos(self.angle)) #for some reason += doesnt work at the start
                self.position[1] = self.position[1] + (dDist * np.sin(self.angle))
                
            return(True)
    
    def simulateFeedback(self, dTime):
        """handle acceleration (both velocity and steering), to simulate getting sensor feedback from car"""
        if(abs(self.desired_velocity - self.velocity) > abs((self.acceleration * dTime)/2)): #if there's still room to improve
            if(self.desired_velocity > self.velocity):
                self.velocity = self.velocity + (self.acceleration * dTime)
            else:
                self.velocity = self.velocity - (self.acceleration * dTime)
        elif(abs(self.desired_velocity) < 0.1): #full stop
            self.velocity = 0.0
        #self.velocity = min(max(self.velocity, self.min_velocity), self.max_velocity)
        
        if(abs(self.desired_steering - self.steering) > abs((self.steer_accel * dTime)/2)):
            if(self.desired_steering > self.steering):
                self.steering = self.steering + (self.steer_accel * dTime)
            else:
                self.steering = self.steering - (self.steer_accel * dTime)
        #self.steering = min(max(self.steering, -self.maxSteeringAngle), self.maxSteeringAngle)
        