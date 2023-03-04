# see changelog and README for explenation

simulation = False # True

bufferLandmarks = True

makeConesOnlyFirstLap = True
## TODO?: makeConesAfterMapload = False #whether to completely rely on the loaded map, or to allow for new cone detection
delaySLAMuntillDriving = True # NOTE: only affects SLAM's pos updates, not it's cone placement (SLAM needs to place new cones in order to start driving, it's a fun paradox)

autoTrackDiscovery = False

useDrawer = False # whether to draw stuff on the screen (pretty useful for debugging and user-interfacing, but it does consume a bunch of processing power)
printConnectionDebug = True # whether to explicitely print out the details of the kartMCU and LiDAR connections (serial port stuff)
saveMapOnClose = False # please enable when doing any sort of relevant testing (but having this set to False will prevent your folders overflowing with map files)

importCmdlineMapsAsBoth = True # if enabled, maps imported from the cmdline will be imported as both an undiscovered map AND the discovered cones. This is mostly for demo or test purposes

from Map import Map
import generalFunctions as GF

import map_loader       as ML
import coneConnecting   as CC
import pathFinding      as PF
import pathPlanningTemp as PP    #autodriving and splines

import SLAM_DIY as SLAM  # TODO: replace/improve SLAM_DIY

from log.mainLog import mapLoggerClass # some basic logging for the masterMap

import realCar        as RC
from HWserialConn import shuffleSerials

import time # used for time.sleep()
import numpy as np
import sys #used for cmdline arguments

class masterMapClass(Map):
    """a wrapper for the Map class which adds/initializes things
        the Map class is the same for both simulations and IRL runs,
         this class changes (a little) based on the situation"""
    def __init__(self):
        Map.__init__(self) #init map class
        self.car = RC.realCar(self.clock)
        self.car.pathFolData = PP.pathPlannerCarData()
        self.pathFolData = PP.pathPlannerMapData()


if __name__ == "__main__":
    try:
        masterMap = masterMapClass()

        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            ML.load_map(sys.argv[1], masterMap)
            if(useDrawer):
                import os
                _, drawer.lastMapFilename = os.path.split(sys.argv[1]) # save the name of the loaded mapfile
            print("loaded cmdline map:", drawer.lastMapFilename)
        
        #### connect the kartMCU and lidar(s):
        defaultExclusionList = ["COM256",] # a quick fix for the bad MB drivers (which make an un-blockable COM port for no reason)
        ## first the kartMCU:
        while(not masterMap.car.connect(comPort=None, autoFind=True, tryAny=True, exclusionList=defaultExclusionList, printDebug=printConnectionDebug)):
            time.sleep(0.5) # a nice long delay, to avoid spamming the terminal output
        masterMap.car.doHandshakeIndef(resetESP=True, printDebug=True)
        if(not masterMap.car.is_ready):
            print("realCar handshake failed or something, i don't feel like dealing with this")
            raise(Exception("nah, bro"))
        ## initialize the lidar(s)
        ## now make sure the serial ports are actually connected to the correct objects:
        shuffleSerials(masterMap.car)#, *lidars) # pass all things with a handshakeSerial, so they can be shuffled untill correct
        
        ## now that all the connections are established, let's start initializing some stuff:
        masterMap.car.setSteeringEnable(True) # enable/disable the steering motor (so a human can drive the kart)
        masterMap.car.setPedalPassthroughEnable(True) # enable/disable the steering motor (so a human can drive the kart)
        
        
        if(bufferLandmarks):
            landmarkBufferLengthThreshold = 5 #if the number of landmarks in the buffer >= this threshold, then process them immediately (dont wait for the timer)
            landmarkBufferTimer = masterMap.clock()
            landmarkBufferTimeInterval = 0.2
        
        masterMap.car.slamData = masterMap.clock() #store time since last SLAM update in car.slamData
        #SlamUpdateInterval = landmarkBufferTimeInterval
        
        simDriftVelocityError = simDriftVelocityError = 0.0 # init var
        mapLogger = mapLoggerClass()

        lastCones = {False : None, True : None} # the cones at the end of each boundry. Attempt to connect these
        autoPathFindTimer = masterMap.clock()
        autoPathFindInterval = CC.coneConnecter.findFirstConesDelay # initialized to how long the car should wait before building attempting the first connections
        
        ######################################################################################## main loop ###############################################################################
        while (True):
            loopStart = masterMap.clock()
            loopSpeedTimers = [('start', time.time()),]

            if(autoTrackDiscovery and ((masterMap.clock() - autoPathFindTimer) > autoPathFindInterval) and (not masterMap.targets_full_circle)):
                autoPathFindTimer = masterMap.clock()
                if((lastCones[False] is not None) and (lastCones[True] is not None)):
                    for LorR in lastCones: # iterates over keys!
                        connectSuccess, winningCone = CC.connectCone(masterMap, lastCones[LorR], applyResult=True, printDebug=False)
                        if(connectSuccess):
                            lastCones[LorR] = winningCone
                    #if(cone list actually changed):
                    limitCounter = 0; limit = 3
                    while(PF.makePath(masterMap, printDebug=False) and (limitCounter<limit)): #stops when path can no longer be advanced
                        limitCounter += 1
                    # PP.makePathSpline(masterMap) # NOTE: moved to pathPlanningTemp.py to make sure it's only done when the first lap is completed
                else:
                    autoPathFindInterval = CC.coneConnecter.findFirstConesInterval # from now on, try this more often (untill it succeeds, at which point we can go even faster)
                    onlyOneSide = (False if (lastCones[False] is not None) else (True if (lastCones[True] is not None) else None))
                    firstCones = CC.findFirstCones(masterMap, onlyOneSide)
                    if(((firstCones[False] is not None) or (lastCones[False] is not None)) and ((firstCones[True] is not None) or (lastCones[True] is not None))): # not strictly needed, but waiting a little longer means more sensor data can come in
                        lastCones = CC.connectFirstCones(masterMap, firstCones, onlyOneSide, makeFinish=True)
                        if((lastCones[False] is not None) and (lastCones[True] is not None)):
                            autoPathFindInterval = CC.coneConnecter.defaultConeConnectInterval

            if(masterMap.car.pathFolData.auto):
                PP.calcAutoDriving(masterMap, printDebug=False)
                loopSpeedTimers.append(('calcAutoDriving', time.time()))

            

            doNothing = 0
            mapLogger.logMap(masterMap)
            loopSpeedTimers.append(('mapLogger', time.time()))

            # print("speed times:", [(loopSpeedTimers[i][0], round((loopSpeedTimers[i][1]-loopSpeedTimers[i-1][1])*1000, 1)) for i in range(1,len(loopSpeedTimers))])
            
            ## Setting steering
            masterMap.car.desired_steering = 0.3
            # print("desired steering:", masterMap.car.desired_steering)
            
            ## Actual steering
            print("steering:", masterMap.car.steering)
            
            ## Setting velocity
            masterMap.car.desired_velocity = 1
            # print("desired velocity:", masterMap.car.desired_velocity)
            
            ## Actual velocity
            print("velocity:", masterMap.car.velocity)

            masterMap.car.update() # retrieve data from the kartMCU and transmit the desired speed/steering
            loopSpeedTimers.append(('car.update', time.time()))

            loopEnd = masterMap.clock() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
            if((loopEnd-loopStart) < 0.015): #60FPS limiter (optional)
                time.sleep(0.0155-(loopEnd-loopStart))
            elif((loopEnd-loopStart) > (1/5)):
                print("main process running slow", 1/(loopEnd-loopStart))
    finally:
        print("main ending")
        if(not simulation):
            try:
                masterMap.car.desired_velocity = 0.0;   masterMap.car.desired_steering = 0.0
                masterMap.car.setSteeringEnable(False) # this has the useful side-effect of sending the desired velocity and steering to the kartMCU (hopefully stopping it)
                masterMap.car.disconnect()
                print("closing car comm success")
            except:
                print("couldn't stop car communication")
    print("main ended")
