## this file contains some of the functions for properly interfacing with the kartMCU ESP32
## for some of the more knitty-gritty serial communication code, see HWserialConn.py

## TODO:

import numpy as np
from path_planning import PathPlanning
from constants import *
import GF.generalFunctions as GF  #(homemade) some useful functions for everyday ease of use
from HWserialConn import kartMCUserialClass
from log.HWserialConnLogging import kartMCUserialLogger

USE_RAW_DATA = True  # if true, the kart will provide raw values (so they can be processed in python instead of on the kartMCU)

WHEEL_CIRCUMFERENCE = np.pi * 0.3 # (meters) circumference of the car's wheels
ENCO_COUNT_TO_METERS = (WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12) # raw encoder count to meter convesion multiplier (for each wheel)

STEERING_RAW_TO_RADIANS = MAX_STEERING / 3500  # TBD!

class realCar(PathPlanning, kartMCUserialClass, kartMCUserialLogger):
    """a class that simulates the car
        (replace Map.Car object with this)"""
    def __init__(self, clockFunc, logfilename=None):
        PathPlanning.__init__(self)  # init Car object
        kartMCUserialClass.__init__(self, clockFunc)  # init Car object
        kartMCUserialLogger.__init__(self, logfilename)  # init logger
        
        #self.desired_steering_raw = np.int32(0) # see the @property for this value
        self.lastEncoderVals = None
    
    def __del__(self): # the automatically generated __del__ function only calls 1 (custom?) __del__ function, the first parent class's one. This solves that
        kartMCUserialClass.__del__(self) # make sure to close the serial
        kartMCUserialLogger.__del__(self) # make sure to close the logfile

    def setSteeringEnable(self, enabled):
        """enable/disable the steering motor (so a human can drive)
            (just a macro to requestSetSteeringEnable())"""
        self.requestSetSteeringEnable(self, enabled) # pass 'self' as carToUse
    
    def setPedalPassthroughEnable(self, enabled):
        """enable/disable the throttle-pedal passthrough/override (so a human can drive)
            (just a macro to requestSetPedalPassthroughEnable())"""
        self.requestSetPedalPassthroughEnable(self, enabled) # pass 'self' as carToUse
    
    @property
    def desired_steering_raw(self):
        return(np.int32(self.car.desired_steering / STEERING_RAW_TO_RADIANS))
    
    def _rawSteeringToReal(self, rawSteerVal: np.int32):
        return(rawSteerVal * STEERING_RAW_TO_RADIANS)
    
    def _rawEncodersToMeters(self, rawEncoderValues: np.array):
        return(np.array([rawEncoderValues*ENCO_COUNT_TO_METERS[i] for i in range(4)], dtype=np.float32))

    def _encodersToForwardSpeed(self, encoderValues: np.array, dTime: float):
        ## TODO: calculate an appropriate forward velocity based on the distance-travelled of each wheel
        ## i'm thinking just the average of all wheels might work, or maybe just of the 2 rear wheels or something, i have to think about this some more...
        return((encoderValues[0] - self.lastEncoderVals[0]) / dTime)
    
    def _encodersToForwardMovement(self, encoderValues: np.array):
        ## TODO: calculate an appropriate forward distance-travelled based on the distance-travelled of each wheel
        ## i'm thinking just the average of all wheels might work, or maybe just of the 2 rear wheels or something, i have to think about this some more...
        return(encoderValues[0] - self.lastEncoderVals[0])

    def _update_pos(self, dTime, dDist=None, applyUpdate=True):
        """ update the position of the car, based on velocity, steering and time-passage """
        if(dDist is None):
            dDist = self.velocity*dTime
        
        returnVal = [np.zeros((2)), 0.0] # init var
        ## turning math
        if((abs(self.car.steering) > 0.001) and (abs(self.velocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
            turning_radius = WHEELBASE/np.tan(self.car.steering)
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
            returnVal[0] = np.array([newPosition[0]-self.car.position.x, newPosition[1]-self.car.position.y])
            if(applyUpdate):
                ## update position
                self.car.angle += arcMov
                self.car.position.x = newPosition[0] #keep the numpy array object the same (instead of replacing it with a whole new array every time)
                self.car.position.y = newPosition[1]
        else:
            returnVal[0] = np.array([(dDist * np.cos(self.car.angle)), (dDist * np.sin(self.car.angle))])
            if(applyUpdate):
                self.car.position.x = self.car.position.x + returnVal[0][0] #for some reason += doesnt work at the start
                self.car.position.y = self.car.position.x + returnVal[0][1]
        
        return(returnVal) # return position and angle

    def update(self):
        if(not self.is_ready):
            print("can't run realCar.update(), the connection is not ready:", self.is_ready)
            return(False)
        newData = self.requestKartData(carToUse=self, raw=USE_RAW_DATA) # sends desiredSpeed and retrieves sensor data
        if(newData is None):
            print("realCar.update() failed to retrieve data from the kart!")
            return(False)
        self.lastFeedbackTimestamp = self.clockFunc() # save when the sensors last provided feedback
        self.logPacket(newData, self.lastFeedbackTimestamp) # first of all, log the packet
        dDist = 0.0 # init var
        if(USE_RAW_DATA): # TODO: raw data
            self.steering = self._rawSteeringToReal(newData['steerAngle'])
            convertedEncoders = self._rawEncodersToMeters(newData['encoders'])
            self.velocity = self._encodersToForwardSpeed(convertedEncoders)
            dDist = self._encodersToForwardMovement(convertedEncoders)
        else:
            self.steering = newData['steerAngle']
            self.velocity = self._encodersToForwardSpeed(newData['encoders'])
            dDist = self._encodersToForwardMovement(newData['encoders'])
        self._update_pos(self.clockFunc()-self.lastUpdateTimestamp, dDist, True)
        self.lastUpdateTimestamp = self.clockFunc() # save when the car last updated its position (also done at SLAM)
        self.lastEncoderVals = newData['encoders'] # save (the distance-travelled of each wheel) for next time