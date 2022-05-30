# see cahngelog and README for explenation

simulation = True

bufferLandmarks = True
simulatePositionalDrift = True

makeConesAfterMapload = False #whether to completely rely on the loaded map, or to allow for new cone detection
## TBD: stop cone creation after a drag-drop mapload (currently only cmdline argumented mapfiles stop cone creation)
makeConesOnlyFirstLap = True
delaySLAMuntillDriving = False

useDrawer = True

from Map import Map
import GF.generalFunctions as GF

import map_loader       as ML
import coneConnecting   as CC
import pathFinding      as PF
import pathPlanningTemp as PP    #autodriving and splines

import SLAM_DIY as SLAM

if(simulation):
    import simulatedCar   as SC
    import simulatedLidar as SL
else:
    import carMainConn    as RC
    import lidarConn      as RL

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
    def __init__(self):
        Map.__init__(self) #init map class
        
        if(simulation):
            self.car = SC.simCar() #simCar has Map.Car as a parent class, so all regular Car stuff will still work
            self.simulationVariables = [SL.simulatedLidarVariables() for _ in self.car.lidarOffsets]
        else:
            self.car = RC.realCar()
        
        self.car.pathFolData = PP.pathPlannerCarData()
        self.pathFolData = PP.pathPlannerMapData()


if __name__ == "__main__":
    try:
        masterMap = masterMapClass()

        makeNewCones = True #whether to allow the creation of cones at all (determined by things like makeConesAfterMapload)
        
        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            ML.mapLoader.load_map(sys.argv[1], masterMap)
            if(not makeConesAfterMapload):
                makeNewCones = False
        
        drawer = drawer3D = None
        if(useDrawer):
            resolution = [1280, 720]
            DD.pygameInit(resolution)
            drawer = DD.pygameDrawer(masterMap, DD.window, resolution)
            drawer3D = DD.pygameDrawer3D(masterMap, DD.window, resolution)
        
        if(not simulation):
            doNothing = 0
            #start car communication here
            #start lidar communication(s) here
        
        lidarConeBuff = [] #init var
        cameraConeBuff = [] 
        
        if(bufferLandmarks):
            landmarkBufferLengthThreshold = 5 #if the number of landmarks in the buffer >= this threshold, then process them immediately (dont wait for the timer)
            landmarkBufferTimer = masterMap.clock()
            landmarkBufferTimeInterval = 0.25
        
        masterMap.car.slamData = masterMap.clock() #store time since last SLAM update in car.slamData
        #SlamUpdateInterval = landmarkBufferTimeInterval
        
        simDriftVelocityError = simDriftVelocityError = 0.0 # init var
        if(simulatePositionalDrift):
            masterMap.car.simulationVariables = SC.simCar() # create a second virtual car to anker from (which will have no drift)
            masterMap.car.simulationVariables.position = masterMap.car.position.copy() # copy initial conditions
            masterMap.car.simulationVariables.angle = masterMap.car.angle
            simDriftVelocityError = 0.1
            simDriftSteeringError = np.deg2rad(5)
        
        timeSinceLastUpdate = masterMap.clock()
        while (DD.windowKeepRunning if useDrawer else True):
            loopStart = masterMap.clock()
            
            if(masterMap.car.pathFolData.auto):
                PP.calcAutoDriving(masterMap)
                #masterMap.car.sendSpeedAngle(masterMap.car.desired_velocity, masterMap.car.desired_steering) #(spam) send instruction (or simulate doing so)
                if(not simulation):
                    ## send actuation instructions to carMCU
                    doNothing = 0
                    #connToCar.send((masterMap.car.desired_velocity, masterMap.car.desired_steering))
            
            if(simulation):
                if(simulatePositionalDrift):
                    # using the simCar stored in car.simulationVariables
                    masterMap.car.simulationVariables.desired_velocity = masterMap.car.desired_velocity;  masterMap.car.simulationVariables.desired_steering = masterMap.car.desired_steering
                    masterMap.car.simulationVariables.simulateFeedback(loopStart - timeSinceLastUpdate)
                    masterMap.car.simulationVariables.update(loopStart - timeSinceLastUpdate) #update (drifted) position/angle
                    masterMap.car.simulationVariables.lastFeedbackTimestamp = loopStart
                    ## now mess with the velocity/steering a little
                    if(masterMap.car.simulationVariables.velocity > 0.01):
                        masterMap.car.velocity = abs(np.random.normal(masterMap.car.simulationVariables.velocity, simDriftVelocityError)) #abs() is just to make sure it doesnt go negative
                    else:
                        masterMap.car.velocity = 0.0
                    masterMap.car.steering = np.random.normal(masterMap.car.simulationVariables.steering, simDriftSteeringError)
                    masterMap.car.update(loopStart - timeSinceLastUpdate) #update true (hidden) position
                    
                    # ## using the data stored as a list in car.simulationVariables
                    # masterMap.car.velocity = masterMap.car.simulationVariables[2];   masterMap.car.steering = masterMap.car.simulationVariables[3] #retrieve true (hidden) values
                    # masterMap.car.simulateFeedback(loopStart - timeSinceLastUpdate)
                    # masterMap.car.simulationVariables[2] = masterMap.car.velocity;   masterMap.car.simulationVariables[3] = masterMap.car.steering #store true (hidden) values
                    
                    # tempPos = masterMap.car.position;   tempAngle = masterMap.car.angle #store (drifted) position/angle in a temporary variable
                    # masterMap.car.position = masterMap.car.simulationVariables[0];   masterMap.car.angle = masterMap.car.simulationVariables[1] #retrieve true (hidden) values
                    # masterMap.car.update(loopStart - timeSinceLastUpdate) #update true (hidden) position
                    # masterMap.car.simulationVariables[0] = masterMap.car.position;   masterMap.car.simulationVariables[1] = masterMap.car.angle #store true (hidden) values
                    # masterMap.car.position = tempPos;   masterMap.car.angle = tempAngle #retrieve (drifted) position
                    
                    # ## now mess with the velocity/steering a little
                    # if(masterMap.car.velocity > 0.01):
                    #     masterMap.car.velocity = abs(np.random.normal(masterMap.car.velocity, simDriftVelocityError)) #abs() is just to make sure it doesnt go negative
                    # masterMap.car.steering = np.random.normal(masterMap.car.steering, simDriftSteeringError)
                    
                    # masterMap.car.update(loopStart - timeSinceLastUpdate) #update (drifted) position/angle
                else:
                    masterMap.car.simulateFeedback(loopStart - timeSinceLastUpdate)
                    masterMap.car.update(loopStart - timeSinceLastUpdate)
                masterMap.car.lastFeedbackTimestamp = loopStart
            else:
                #get steering & speed data from car and run  masterMap.car.update
                doNothing = 0
            
            timeSinceLastUpdate = loopStart #save time (from start of loop) to be used next time

            ## lidar update:
            if(simulation):
                # simulate the lidar
                lidarCones = SL.getCones(masterMap) # returns an array (len = num_of_lidars), containing arrays (len = num_of_cones_spotted) containing a tuple (position, simulated_measurement_points)
                for dataPerLidar in lidarCones:
                    lidarConeBuff += dataPerLidar # for each lidar, add the spotted cones (and the measurement points) to the overall buffer
            else:
                #get data from lidar (microcontroller(s))
                doNothing = 0
            
            if( (((masterMap.clock() - landmarkBufferTimer) > landmarkBufferTimeInterval) 
                 or (len(lidarConeBuff) > landmarkBufferLengthThreshold))
                   if bufferLandmarks else (len(lidarConeBuff) > 0) ):
                #if(((masterMap.clock() - landmarkBufferTimer) < landmarkBufferTimeInterval) if bufferLandmarks else False):
                #    print("landmarkBufferLengthThreshold crossed prematurely:", len(lidarConeBuff), ">", landmarkBufferLengthThreshold, " after only", round(masterMap.clock() - landmarkBufferTimer, 3), "seconds (/clock units)")
                
                if(masterMap.car.pathFolData.auto if delaySLAMuntillDriving else True):
                    makeNewConesTemp = makeNewCones
                    if(makeConesOnlyFirstLap and makeNewCones):
                        makeNewConesTemp = (masterMap.car.pathFolData.laps < 1) #only on the first lap
                    SLAM.updatePosition(masterMap, [lidarConeBuff, []], (1.0, 1.0), makeNewConesTemp, True)
                    #doNothing = 0
                
                lidarConeBuff = [] # empty the buffer
                if(bufferLandmarks):
                    landmarkBufferTimer = masterMap.clock()
                    #landmarkBufferTimer += landmarkBufferTimeInterval # dangerous
            
            if(useDrawer):
                if(drawer.extraViewMode):
                    drawer3D.drawTargetConeLines = drawer.drawTargetConeLines;  drawer3D.drawConeSlamData = drawer.drawConeSlamData #a manual fix for some multicore UI issues
                    drawer3D.redraw()
                else:
                    drawer.redraw()
                DD.frameRefresh()
                UI.handleAllWindowEvents(drawer) #handle all window events like key/mouse presses, quitting and most other things
            
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
               #close car communication interface here
               doNothing = 0
               print("closing car comm success")
           except:
               print("couldn't stop car communication")
           try:
               #close lidar communication interface here
               doNothing = 0
               print("closing lidar comm success")
           except: 
               print("couldn't stop lidar communication")
    print("main ended")
