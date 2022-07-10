## this file contains some of the functions for properly interfacing with the LiDARs
## for some of the more knitty-gritty serial communication code, see HWserialConn.py

## TODO:
# - inaccuracy threshold
# - expected pointcount (at dist) threshold (we know how many points should be measured, if there are too many, it's probably not a cone)

import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use
from HWserialConn import lidarESPserialClass
from log.HWserialConnLogging import LIDARserialLogger


class lidarClass(lidarESPserialClass, LIDARserialLogger):
    """a class that handles the connection to a LIDAR (IRL, not simulated)
        (make one of these for every lidar on the car)"""
    def __init__(self, clockFunc, identifierIndex=0, logfilename=None):
        Map.Car.__init__(self) # init Car object
        lidarESPserialClass.__init__(self, clockFunc, identifierIndex) # init Car object
        LIDARserialLogger.__init__(self, logfilename) # init logger
        
    
    def __del__(self): # the automatically generated __del__ function only calls 1 (custom?) __del__ function, the first parent class's one. This solves that
        lidarESPserialClass.__del__(self) # make sure to close the serial
        LIDARserialLogger.__del__(self) # make sure to close the logfile
    
    def setMaxRange(self, carToUse: Map.Car, newMaxRange):
        """set max range
            (just a macro to requestSetMaxRange())"""
        self.requestSetMaxRange(carToUse, newMaxRange) # pass 'self' as carToUse

    def getCones(self, carToUse: Map.Car):
        if(not self.is_ready):
            print("can't run lidarClass.getCones(), the connection is not ready:", self.is_ready)
            return([])
        lidarData = self.requestLidarData(carToUse) # sends position and angle while retrieving cone positions
        for conePos in lidarData: # lidarData is always an array, but sometimes empty (if requestLidarData failed)
            self.logConePos(conePos)
        ## here might be a good time to do some extra validity checking. Stuff like a simple ['inaccuracy'] threshold, or maybe checking whether the ['pointCount'] makes sense for a cone at that distance
        ## although that checking could also be done on the lidarESP already
        return([(conePos['pos'], None) for conePos in lidarData])