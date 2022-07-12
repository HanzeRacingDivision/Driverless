# see changelog and README for explenation

simulation = False

bufferLandmarks = True
simulatePositionalDrift = False # (only for simulations) the sensor data for steering&velocity have some error IRL, this simulates that (used to test SLAM)

makeConesAfterMapload = False #whether to completely rely on the loaded map, or to allow for new cone detection
## TBD: stop cone creation after a drag-drop mapload (currently only cmdline argumented mapfiles stop cone creation)
makeConesOnlyFirstLap = True
delaySLAMuntillDriving = False

useDrawer = True # whether to draw stuff on the screen (pretty useful for debugging and user-interfacing, but it does consume a bunch of processing power)
printConnectionDebug = True # whether to explicitely print out the details of the kartMCU and LiDAR connections (serial port stuff)

from doctest import master
from socket import TCP_NODELAY
from Map import Map
import GF.generalFunctions as GF

import map_loader       as ML
import coneConnecting   as CC
import pathFinding      as PF
import pathPlanningTemp as PP    #autodriving and splines

import SLAM_DIY as SLAM  # TODO: replace/improve SLAM_DIY

from log.mainLog import mapLoggerClass # some basic logging for the masterMap

if(simulation):
    from Map import mapSimVarClass
    import simulatedCar   as SC
    import simulatedLidar as SL
    import simulatedVision as SV  # simulated computerVision (at least useful to determine the color of detected cones)
else:
    import realCar        as RC
    import realLidars     as RL
    from HWserialConn import shuffleSerials
    # import realVisionReworked as RV

if(useDrawer):
    import drawDriverless as DD
    import pygameUI       as UI

import time # used for time.sleep()
import numpy as np
import sys #used for cmdline arguments

if(simulatePositionalDrift and (not simulation)):
    print("can't use simulatePositionalDrift with 'simulation' set to False")
    simulatePositionalDrift = False

class masterMapClass(Map):
    """a wrapper for the Map class which adds/initializes things
        the Map class is the same for both simulations and IRL runs,
         this class changes (a little) based on the situation"""
    def __init__(self):
        Map.__init__(self) #init map class
        
        if(simulation):
            self.car = SC.simCar() #simCar has Map.Car as a parent class, so all regular Car stuff will still work
            self.simVars = mapSimVarClass()
        else:
            self.car = RC.realCar(self.clock)
        
        self.car.pathFolData = PP.pathPlannerCarData()
        self.pathFolData = PP.pathPlannerMapData()


if __name__ == "__main__":
    try:
        masterMap = masterMapClass()

        makeNewCones = True #whether to allow the creation of cones at all (determined by things like makeConesAfterMapload)
        if(simulation):
            masterMap.simVars.undiscoveredCones = True # (only for simulations) load mapfiles into .simVars instead of directly into the masterMap, to simulate 'discovering' them
        
        drawer = drawer3D = None # init vars
        if(useDrawer): # note: drawer initialization is moved above load_map with sys.argv specifically so it can save the map correctly
            resolution = [1280, 720]
            DD.pygameInit(resolution)
            drawer = DD.pygameDrawer(masterMap, DD.window, resolution)
            drawer3D = DD.pygameDrawer3D(masterMap, DD.window, resolution)

        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            if(simulation and masterMap.simVars.undiscoveredCones):
                ML.mapLoader.load_map(sys.argv[1], masterMap.simVars)
            else:
                ML.mapLoader.load_map(sys.argv[1], masterMap)
            if(useDrawer):
                import os
                _, drawer.lastMapFilename = os.path.split(sys.argv[1]) # save the name of the loaded mapfile
            if(not makeConesAfterMapload):
                makeNewCones = False
        
        if(simulation):
            masterMap.simVars.lidarSimVars = [SL.simulatedLidarVariables() for _ in masterMap.car.lidarOffsets]
        else:
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
            lidars = [RL.lidarClass(masterMap.clock, masterMap.car, lidarIndex) for lidarIndex in range(   1   )]
            for lidarIndex in range(len(lidars)):
                while(not lidars[lidarIndex].connect(comPort=None, autoFind=True, tryAny=True, exclusionList=(defaultExclusionList + [masterMap.car.comPort,] + [lidars[j].comPort for j in range(lidarIndex)]), printDebug=printConnectionDebug)):
                    time.sleep(0.5) # wait a little bit, to avoid spamming the terminal
                lidars[lidarIndex].doHandshakeIndef(resetESP=True, printDebug=True)
            ## now make sure the serial ports are actually connected to the correct objects:
            shuffleSerials(masterMap.car, *lidars) # pass all things with a handshakeSerial, so they can be shuffled untill correct
            
            ## now that all the connections are established, let's start initializing some stuff:
            masterMap.car.setSteeringEnable(False) # enable/disable the steering motor (so a human can drive the kart)
            masterMap.car.setPedalPassthroughEnable(True) # enable/disable the steering motor (so a human can drive the kart)
            for lidar in lidars:
                lidar.setMaxRange(500) # set the max lidar range (in millimeters)
                time.sleep(0.5) # needed for lidar.requestReady to be accurate
                print("lidar requestReady success:", lidar.requestReady())
        
        lidarConeBuff = [] #init var
        cameraConeBuff = [] 
        
        if(bufferLandmarks):
            landmarkBufferLengthThreshold = 5 #if the number of landmarks in the buffer >= this threshold, then process them immediately (dont wait for the timer)
            landmarkBufferTimer = masterMap.clock()
            landmarkBufferTimeInterval = 0.2
        
        masterMap.car.slamData = masterMap.clock() #store time since last SLAM update in car.slamData
        #SlamUpdateInterval = landmarkBufferTimeInterval
        
        simDriftVelocityError = simDriftVelocityError = 0.0 # init var
        if(simulatePositionalDrift):
            # store another simCar in masterMap.simVars.car and update its position without error
            masterMap.simVars.car = SC.simCar() # create a second virtual car to anker from (which will have no drift)
            masterMap.simVars.car.position = masterMap.car.position.copy() # copy initial conditions
            masterMap.simVars.car.angle = masterMap.car.angle
            simDriftVelocityError = 0.1
            simDriftSteeringError = np.deg2rad(5)

        mapLogger = mapLoggerClass()

        lastCones = []
        
        ######################################################################################## main loop ###############################################################################
        while (DD.windowKeepRunning if useDrawer else True):
            loopStart = masterMap.clock()
            loopSpeedTimers = [('start', time.time()),]

            # if((len(masterMap.conelists[False])>0) and (len(masterMap.conelists[True])>0)):
            #     try to connec last cones
            # else:
            #     if()
            #     find first cones
            
            if(masterMap.car.pathFolData.auto):
                PP.calcAutoDriving(masterMap)
                loopSpeedTimers.append(('calcAutoDriving', time.time()))

            if(simulation):
                if(simulatePositionalDrift):
                    # update() the masterMap.simVars.car without error and then add error for the masterMap.car update()
                    masterMap.simVars.car.desired_velocity = masterMap.car.desired_velocity;  masterMap.simVars.car.desired_steering = masterMap.car.desired_steering
                    masterMap.simVars.car.simulateFeedback(loopStart - masterMap.simVars.car.lastUpdateTimestamp)
                    masterMap.simVars.car.update(loopStart - masterMap.simVars.car.lastUpdateTimestamp) #update (drifted) position/angle
                    masterMap.simVars.car.lastUpdateTimestamp = loopStart
                    ## now mess with the velocity/steering a little
                    if(masterMap.simVars.car.velocity > 0.01):
                        masterMap.car.velocity = abs(np.random.normal(masterMap.simVars.car.velocity, simDriftVelocityError)) #abs() is just to make sure it doesnt go negative
                    else:
                        masterMap.car.velocity = 0.0
                    masterMap.car.steering = np.random.normal(masterMap.simVars.car.steering, simDriftSteeringError)
                    masterMap.car.update(loopStart - masterMap.car.lastUpdateTimestamp) #update true (hidden) position
                else:
                    masterMap.car.simulateFeedback(loopStart - masterMap.car.lastUpdateTimestamp)
                    masterMap.car.update(loopStart - masterMap.car.lastUpdateTimestamp)
                masterMap.car.lastUpdateTimestamp = loopStart
            else: # with real car:
                masterMap.car.update() # retrieve data from the kartMCU and transmit the desired speed/steering
                # masterMap.car.lastFeedbackTimestamp = loopStart # save when the sensors last provided feedback                    # moved to car.update function!
                # masterMap.car.lastUpdateTimestamp = loopStart # save when the car last updated its position (also done at SLAM)   # moved to car.update function!
            loopSpeedTimers.append(('car.update', time.time()))

            ## lidar update:
            if(simulation): # simulate the lidar:
                lidarCones = SL.getCones(masterMap) # returns an array (len = num_of_lidars), containing arrays (len = num_of_cones_spotted) containing a tuple (position, simulated_measurement_points)
                for dataPerLidar in lidarCones:
                    lidarConeBuff += dataPerLidar # for each lidar, add the spotted cones (and the measurement points) to the overall buffer
            else: # get data from lidar (microcontroller(s)):
                for lidar in lidars:
                    lidarConeBuff += lidar.getCones()
            loopSpeedTimers.append(('get lidar data', time.time()))

            ## camera update:
            if(simulation): # simulate the camera:
                cameraConeBuff += SV.getCones(masterMap)
            else: # get data from the camera:
                ## TBD!
                doNothing = 0
            loopSpeedTimers.append(('get camera data', time.time()))
            
            if( (((masterMap.clock() - landmarkBufferTimer) > landmarkBufferTimeInterval) 
                 or (len(lidarConeBuff) > landmarkBufferLengthThreshold))
                   if bufferLandmarks else (len(lidarConeBuff) > 0) ):
                #if(((masterMap.clock() - landmarkBufferTimer) < landmarkBufferTimeInterval) if bufferLandmarks else False):
                #    print("landmarkBufferLengthThreshold crossed prematurely:", len(lidarConeBuff), ">", landmarkBufferLengthThreshold, " after only", round(masterMap.clock() - landmarkBufferTimer, 3), "seconds (/clock units)")
                
                if(masterMap.car.pathFolData.auto if delaySLAMuntillDriving else True):
                    makeNewConesTemp = makeNewCones
                    if(makeConesOnlyFirstLap and makeNewCones):
                        makeNewConesTemp = (masterMap.car.pathFolData.laps < 1) #only on the first lap
                    SLAM.updatePosition(masterMap, [lidarConeBuff, cameraConeBuff], (1.0, 1.0), makeNewConesTemp)
                    #masterMap.car.lastUpdateTimestamp = loopStart
                
                lidarConeBuff = [];   cameraConeBuff = [] # empty the buffers
                if(bufferLandmarks):
                    landmarkBufferTimer = masterMap.clock()
                    #landmarkBufferTimer += landmarkBufferTimeInterval # dangerous
                loopSpeedTimers.append(('SLAM', time.time()))
            
            if(useDrawer):
                if(drawer.extraViewMode):
                    drawer3D.drawTargetConeLines = drawer.drawTargetConeLines #a manual fix for some UI badness (ARC_TODO improve 3D drawer integration?)
                    drawer3D.drawConeSlamData = drawer.drawConeSlamData
                    drawer3D.redraw()
                else:
                    drawer.redraw()
                DD.frameRefresh()
                UI.handleAllWindowEvents(drawer) #handle all window events like key/mouse presses, quitting and most other things
                loopSpeedTimers.append(('drawer', time.time()))
            
            mapLogger.logMap(masterMap)
            loopSpeedTimers.append(('mapLogger', time.time()))

            # print("speed times:", [(loopSpeedTimers[i][0], round((loopSpeedTimers[i][1]-loopSpeedTimers[i-1][1])*1000, 1)) for i in range(1,len(loopSpeedTimers))])

            loopEnd = masterMap.clock() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
            if((loopEnd-loopStart) < 0.015): #60FPS limiter (optional)
                time.sleep(0.0155-(loopEnd-loopStart))
            elif((loopEnd-loopStart) > (1/5)):
                print("main process running slow", 1/(loopEnd-loopStart))
    finally:
        print("main ending")
        if(useDrawer):
            try:
                DD.pygameEnd() #correctly shut down pygame window
                print("drawer stopping done")
            except:
                print("couldn't run pygameEnd()")
        if(not simulation):
            try:
                masterMap.car.desired_velocity = 0.0;   masterMap.car.desired_steering = 0.0
                masterMap.car.setSteeringEnable(False) # this has the useful side-effect of sending the desired velocity and steering to the kartMCU (hopefully stopping it)
                masterMap.car.disconnect()
                print("closing car comm success")
            except:
                print("couldn't stop car communication")
            try:
                #close lidar communication interface(s) here
                for lidar in lidars:
                    lidar.disconnect()
                print("closing lidar comm success")
            except: 
                print("couldn't stop lidar communication")
    print("main ended")
