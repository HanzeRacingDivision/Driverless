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
        self.min_velocity = -1
        self.max_velocity = 3

        self.steering_velocity = 0.0
        self.steering_accel_max = np.deg2rad(1850) # (constant) steering acceleration at full motor power
        self.steering_friction = 7.5 # (constant) as rotation speed increases, so does friction
        self.steering_target_margin = np.deg2rad(0.1) # (constant) acceptable margin of error for steering control loop
        self.steerSimSubdt = 0.01 # (constant) delta time of steering sub-simulation loop
        
        # self.simTrueCar = None # a (nested) simCar object used for positionalDrift.  MOVED to Map.simVars.car
    
    def update(self, dTime, dDist=None, applyUpdate=True):
            """ update the position of the car, based on velocity, steering and time-passage """
            if(dDist is None):
                dDist = self.velocity*dTime
            
            returnVal = [np.zeros((2)), 0.0]
            ## turning math
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
                
                returnVal[1] = arcMov
                #returnVal[0] = newPosition - self.position # numpy array addition
                returnVal[0] = np.array([newPosition[0]-self.position[0], newPosition[1]-self.position[1]])
                if(applyUpdate):
                    ## update position
                    self.angle += arcMov
                    self.position[0] = newPosition[0] #keep the numpy array object the same (instead of replacing it with a whole new array every time)
                    self.position[1] = newPosition[1]
            else:
                returnVal[0] = np.array([(dDist * np.cos(self.angle)), (dDist * np.sin(self.angle))])
                if(applyUpdate):
                    self.position[0] = self.position[0] + returnVal[0][0] #for some reason += doesnt work at the start
                    self.position[1] = self.position[1] + returnVal[0][1]
            
            return(returnVal) # return position and angle
    
    def simulateFeedback(self, dTime):
        """handle acceleration (both velocity and steering), to simulate getting sensor feedback from car"""
        if(abs(self.desired_velocity - self.velocity) > abs((self.acceleration * dTime)/2)): #if there's still room to improve
            if(self.desired_velocity > self.velocity):
                self.velocity = self.velocity + (self.acceleration * dTime)
            else:
                self.velocity = self.velocity - (self.acceleration * dTime)
        elif(abs(self.desired_velocity) < 0.1): #full stop
            self.velocity = 0.0
        self.velocity = min(max(self.velocity, self.min_velocity), self.max_velocity)
        
        ## simulate steering motor
        for _ in range(int(round(dTime/self.steerSimSubdt, 0))): # run a whole-ass sub-simulation just for the steering motor
            steerAbsDiff = abs(self.steering - self.desired_steering) # absolute angle difference
            # calculate ratio between time_remaining_if_you_start_braking_now and current_velocity using the area under the curve, 2*dist (2x because i'm thinking square, not triangle). note: doesnt use steering_friction!
            steerRequiredDecel = ((self.steering_velocity**2) / (2*steerAbsDiff) if (steerAbsDiff > 0.000001) else self.steering_accel_max) # (see comment above for math) avoid divide by 0 and substitute some high number in case it is 0
            if((steerAbsDiff > self.steering_target_margin) or (steerRequiredDecel > (self.steering_accel_max * 0.25))):
                steerAccel = (self.steering_accel_max if (self.desired_steering > self.steering) else -self.steering_accel_max)
                if(steerRequiredDecel > (self.steering_accel_max * 0.75)): # simulate the microcontroller's decision to start decelerating
                    steerAccel = (-self.steering_accel_max if (self.steering_velocity > 0.0) else self.steering_accel_max) # brake steering motor
                steerAccel -= self.steering_friction * self.steering_velocity # higher steering velocity means more friction.
                self.steering_velocity += self.steerSimSubdt * steerAccel # update steering velocity
                self.steering += self.steerSimSubdt * self.steering_velocity + ((self.steerSimSubdt**2)/2.0)*steerAccel # SUVAT applied to rotational distance
            else:
                break # if the steering system has reached its target (and there is no chance of overshoot), stop itterating (saves a little time)
        ## TBD: simplify formulas for steering motor to approximate it more efficiently...
        self.steering = min(max(self.steering, -self.maxSteeringAngle), self.maxSteeringAngle)
        