## this file contains some of the functions for properly interfacing with the LiDARs
## for some of the more knitty-gritty serial communication code, see HWserialConn.py

## TODO:
# - inaccuracy threshold
# - expected pointcount (at dist) threshold (we know how many points should be measured, if there are too many, it's probably not a cone)

from typing import Callable
import numpy as np
from requests import delete

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use
from HWserialConn import lidarESPserialClass
from log.HWserialConnLogging import LIDARserialLogger

INACCURACY_THRESHOLD = 1000000000 # the inaccuracy (the sum of positional error sqaured divided by pointCount) gives some indication of measurement quality
ACCEPTABLE_POINTCOUNT_DISCREPANCY = 10 # the lower this is, the closer the real pointCount has to be to the expected one (keep in mind rotation and resolution artifacts)

## the accurate way: (see 'lidar resolution vs range calculator.xlsx' for explanatory diagram)
LIDAR_SURFACE_REFLECTION_ARC = np.deg2rad(90) # at some point, the surface is insufficiently perpendicular to correctly reflect the laser
LIDAR_SAMPLERATE = 7500 # (Hz) lidar samples per second
LIDAR_ROTATIONS_PER_SECOND = 5 # (Hz) lidar full rotations per second (5 RPS == 300 RPM)
LIDAR_ANGULAR_RESOLUTION = (LIDAR_ROTATIONS_PER_SECOND * 2 * np.pi) / LIDAR_SAMPLERATE # radians between samples
LIDAR_REFLECTION_ARC_RATIO = lambda coneDiam : ((coneDiam/2) * np.sin(LIDAR_SURFACE_REFLECTION_ARC/2))
LIDAR_SAMPLES_AT_DIST = lambda dist, coneDiam : int((2 * np.arcsin(LIDAR_REFLECTION_ARC_RATIO(coneDiam) / dist)) / LIDAR_ANGULAR_RESOLUTION)

# MIN_BLOB_CONE_LEN = 4
# RANGE_LIMIT = lambda coneDiam : min(12.0, LIDAR_REFLECTION_ARC_RATIO(coneDiam) / np.sin(MIN_BLOB_CONE_LEN*LIDAR_ANGULAR_RESOLUTION/2)) # recalculate true max range based on the parameters above. NOTE: comment this to overwrite
# print("realLidar RANGE_LIMIT for all lidars:", [LIDAR_REFLECTION_ARC_RATIO(Map.Cone.coneLidarDiam(Map.Car.lidarOffsets[i][2]) for i in range(len(Map.Car.lidarOffsets)))])

class lidarClass(lidarESPserialClass, LIDARserialLogger):
    """a class that handles the connection to a LIDAR (IRL, not simulated)
        (make one of these for every lidar on the car)"""
    def __init__(self, clockFunc: Callable, carToUse: Map.Car, identifierIndex=0, logfilename=None):
        Map.Car.__init__(self) # init Car object
        lidarESPserialClass.__init__(self, clockFunc, identifierIndex) # init Car object
        LIDARserialLogger.__init__(self, logfilename) # init logger
        self.carToUse = carToUse
        self.identifierIndex = identifierIndex # save it (specifically for calculating the lidar height (for calculating the cone diam))
    
    def __del__(self): # the automatically generated __del__ function only calls 1 (custom?) __del__ function, the first parent class's one. This solves that
        lidarESPserialClass.__del__(self) # make sure to close the serial
        LIDARserialLogger.__del__(self) # make sure to close the logfile
    
    def setMaxRange(self, newMaxRange: int):
        """set max range
            (just a macro to requestSetMaxRange())"""
        self.requestSetMaxRange(self.carToUse, newMaxRange) # pass 'self' as carToUse
    
    def requestReady(self):
        return super().requestReady(self.carToUse)

    def getCones(self):
        if(not self.is_ready):
            print("can't run lidarClass.getCones(), the connection is not ready:", self.is_ready)
            return([])
        lidarData = self.requestLidarData(self.carToUse) # sends position and angle while retrieving cone positions
        for conePosType in lidarData: # lidarData is always an array, but sometimes empty (if requestLidarData failed)
            self.logConePos(conePosType)
        ## here might be a good time to do some extra validity checking. Stuff like a simple ['inaccuracy'] threshold, or maybe checking whether the ['pointCount'] makes sense for a cone at that distance
        ## although that checking could also be done on the lidarESP already
        #returnList = [(conePosType['pos'], None) for conePosType in lidarData] # TODO: save more useful data
        returnList = []
        coneDiam = Map.Cone.coneLidarDiam(self.carToUse.lidarOffsets[self.identifierIndex][2])
        for conePosType in lidarData:
            if(conePosType['inaccuracy'] < INACCURACY_THRESHOLD):
                dist = GF.distAngleBetwPos(self.carToUse.calcLidarPos(self.identifierIndex), conePosType['pos'])[0]
                expectedPointCount = LIDAR_SAMPLES_AT_DIST(dist, coneDiam)
                if(abs(conePosType['pointCount'] - expectedPointCount) < ACCEPTABLE_POINTCOUNT_DISCREPANCY):
                    returnList.append((conePosType['pos'], None))
                else:
                    print("ignored lidar measurement because the expected pointcount was WAY off:", expectedPointCount, conePosType['pointCount'], dist)
            else:
                print("ignored lidar measurement because the inaccuracy is garbage:", conePosType['inaccuracy'], INACCURACY_THRESHOLD)
        return(returnList)