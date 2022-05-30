
## this version changes the structure to split the (pickle-able) Map object and the functions that interact with it
## drawDriverless.py is split into a drawing file (drawDriverless.py) and a UI file, pygameUI.py
## map_loader, coneConnecting, pathFinding and (now) pathPlanningTemp were static classes (no variables), so these are now used as such (no longer integrated into map object)
##  note: i think you could also pull some functions out of the base Map class (make them global), but i'm not sure that actually gets pickled at all
## the 3D (camera FPV) rendering class is also added to drawDriverless.py
## the position of the car is now based on the rear-axle (which is the center of rotation in traditional Ackerman math), instead of the center of the chassis.
## the following functions have become multi-core:
##     - drawing
##     - remote map viewing
##     - carMCU serial interactions
##     - lidar


###TBD: 
# documentation
# camera
# precompiling numba once (not repeatedly) (mostly done)


simulation = True

useLidar = True
simulateLidar = True
storeBlob = False #enabling this will let you visualize the lidar data much more easily, but will also increase the mapsize (and therefore pickle-load) TENFOLD!

bufferLandmarks = True
useSLAM = True

simulatePositionalDrift = True

makeConesOnlyFirstLap = True #only works if pathPlanningPresent
delaySLAMuntillDriving = True

useRemote = False
useDrawer = True

reverseStart = True  #a debugging boolean, used specifically for the IRL tests done on 8-9 Sept 2021

from Map import Map
import GF.generalFunctions as GF

# import map_loader as ML  #note: imported in __main__
# import coneConnecting as CC
# import pathFinding    as PF
import pathPlanningTemp as PP

if(simulation):
    import simulatedCar as SC
    # if(useLidar):
    #     print("can't use lidar in simulation mode!")
    #     useLidar = False
else:
    import carMCUclass  as RC

if(useLidar):
    if(simulateLidar):
        import lidarBlobProcSimulated as LP
    else:
        import lidarBlobProc as LP

if(useSLAM):
    import SLAM_DIY as SLAM

if(simulatePositionalDrift and (not simulation)):
    print("can't use simulatePositionalDrift with 'simulation' set to False")
    simulatePositionalDrift = False

# import drawDriverless as DD
# import pygameUI       as UI

#import mapTransSock   as MS

import phantom        as PH

import time
import numpy as np
import sys #used for cmdline arguments

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)


class masterMapClass(Map):
    def __init__(self):
        Map.__init__(self) #init map class
        
        if(simulation):
            self.car = SC.simCar() #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        else:
            self.car = RC.realCar()
        
        if(reverseStart):
            self.car.position[0] += 2*((self.car.wheelbase/2) + self.car.chassis_length_offset + (self.car.chassis_length/2))
            self.car.angle = np.pi
        
        #enable components here
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = useSLAM
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerCarData()
            self.pathFolData = PP.pathPlannerMapData()


class drawProcess(MP.Process):
    """a multiprocessing pygameDrawer"""
    def __init__(self, connToMaster, sharedMemInit, resolution, asynchronous=True):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.sharedMemInit = sharedMemInit
        self.resolution = resolution
        self.asynchronous = asynchronous
        
        self.keepalive = MP.Semaphore() # this (functional) boolean stops the while() loop in run()
    
    def run(self):
        #self.keepalive.release() #for running this process again after it was ended
        try:
            import drawDriverless as DD
            import pygameUI       as UI
            #import phantom        as PH  #already imported
            slaveSharedMem = SM.sharedMemReadWrapper(*self.sharedMemInit)
            initMap = slaveSharedMem.readObj(self.keepalive) #wait for the first map object to come in (with no timeout?)
            DD.pygameInit(self.resolution)
            drawer = DD.pygameDrawer(initMap, DD.window, self.resolution)
            drawer3D = DD.pygameDrawer3D(initMap, DD.window, self.resolution)
            while((self.keepalive.get_value()) and DD.windowKeepRunning):
                loopStart = time.time()
                if(self.asynchronous):
                    if(slaveSharedMem.poll()): #if a new map is available
                        drawer.mapToDraw = slaveSharedMem.readObj(self.keepalive) #get master map
                        drawer3D.mapToDraw = drawer.mapToDraw
                else:
                    drawer.mapToDraw = slaveSharedMem.readNewObj(self.keepalive) #get master map
                    drawer3D.mapToDraw = drawer.mapToDraw
                
                if(drawer.extraViewMode):
                    drawer3D.drawTargetConeLines = drawer.drawTargetConeLines;  drawer3D.drawConeSlamData = drawer.drawConeSlamData #a manual fix for some multicore UI issues
                    drawer3D.redraw()
                else:
                    drawer.redraw()
                DD.frameRefresh()
                
                phantomMap = PH.phantomClass()
                UI.handleAllWindowEvents(drawer, phantomMap) #handle all window events like key/mouse presses, quitting and most other things
                
                if(phantomMap._hasData_):
                    self.connToMaster.send(phantomMap) #if there were any UI changes
                    if(self.asynchronous):
                        slaveSharedMem.lastFrameCounter = slaveSharedMem.frameCounter.value #assume you have the latest map object
                        drawer.mapToDraw = slaveSharedMem.readNewObj(self.keepalive) #block untill a new master map is received
                        drawer3D.mapToDraw = drawer.mapToDraw
                loopEnd = time.time()
                if((loopEnd-loopStart) > 0.2):
                    print("drawer running slow", 1/(loopEnd-loopStart))
        finally:
            print("drawProcess ending")
            try:
                DD.pygameEnd() #correctly shut down pygame window
            except:
                print("couldn't run pygameEnd()")
        return()
    
    def stopLoop(self, alsoJoin=True, joinTimeout=5):
        if(self.keepalive.get_value()):
            self.keepalive.acquire()
        if(alsoJoin and self.is_alive()):
            self.join(joinTimeout)
            if(self.is_alive()):
                print("stopLoop join failed, process still alive!")
            else:
                self.keepalive.release() #setup for next start (not needed)

MAX_BLOB_HISTORY = 10
def makeCone(mapToUse, conePos, blob):
    """a TEMPORARY function, which turns blobs into cones without thinking too much about it.
        to be replaced by SLAM!"""
    overlaps, overlappingCone = mapToUse.overlapConeCheck(conePos)
    if(overlaps):
        ## SLAM code should replace this
        if(type(overlappingCone.slamData) is list):
            if(storeBlob):
                overlappingCone.slamData.append((conePos, mapToUse.clock(), blob, overlappingCone.slamData[-1][-1]+1)) #cone position, timestamp, blob, totalSpotCount
            else:
                overlappingCone.slamData.append((conePos, overlappingCone.slamData[-1][-1]+1)) #cone position, totalSpotCount
            if(len(overlappingCone.slamData)>MAX_BLOB_HISTORY): #limit number of sightings to remember
                overlappingCone.slamData.pop(1) #delete the 2nd oldest data (keep the very first sighting (for simulateLidar))
            overlappingCone.position[0] = GF.average(np.array([item[0][0] for item in overlappingCone.slamData]))
            overlappingCone.position[1] = GF.average(np.array([item[0][1] for item in overlappingCone.slamData]))
        else: #cone was not spotted by lidar before
            print("makeCone() warning: adding 'slamData' to an existing cone")
            if(storeBlob):
                overlappingCone.slamData = [(conePos, mapToUse.clock(), blob, 1)]
            else:
                overlappingCone.slamData = [(conePos, 1)]
    else:
        ## SLAM code could replace this
        leftOrRight = (GF.get_norm_angle_between(mapToUse.car.position, conePos, mapToUse.car.angle) < 0.0) #if the angle relative to car is negative (CW), it's a right-side cone
        conePlaceSuccess, coneInList = mapToUse.addCone(conePos, leftOrRight, False)
        #if(conePlaceSuccess) # overlap check already done earlier
        if(storeBlob):
            coneInList.slamData = [(conePos, mapToUse.clock(), blob, 1)]
        else:
            coneInList.slamData = [(conePos, 1)] #less data, faster code

if __name__ == "__main__":
    try:
        masterMap = masterMapClass()
        
        import map_loader as ML
        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            ML.mapLoader.load_map(sys.argv[1], masterMap)
        
        connToSlaves, connToMaster = MP.Pipe(False) #disable duplex (for now)
        masterSharedMem = SM.makeNewSharedMem('driverless', 10*1000*1000) #note: max (pickled) object size is HALF of this size, due to the double-buffering
        
        import generalUI      as UI
        if(useDrawer):
            drawer = drawProcess(connToMaster, masterSharedMem.passToProcess, [1280, 720], False)
            drawer.start()
        
        if(useRemote):
            import mapTransSock   as MS
            remote = MS.remoteProcess(connToMaster, masterSharedMem.passToProcess, '', 65432, 0.2)
            remote.start()
        
        connToCar = None; connToCarMaster = None; carSerial = None #init vars
        if(not simulation):
            connToCar, connToCarMaster = MP.Pipe(True) #a two-way pipe to the carMCUserialProcess (which (encodes/decodes and) passes it on to/from the serial connection)
            carSerial = RC.carMCUserialProcess(connToCarMaster, masterSharedMem.passToProcess, comPort="/dev/ttyUSB0", autoFind=False, autoreconnect=True)
            carSerial.start()
        
        connToLidar = None; connToLidarMaster = None; lidar = None; lidarConeBuff = [] #init var
        if(useLidar):
            connToLidar, connToLidarMaster = MP.Pipe(True) #a two-way pipe through which the lidar process sends blobs (cones), and the master sends the lidar's position
            lidar = LP.lidarProcess(connToLidarMaster, masterSharedMem.passToProcess, comPort="/dev/ttyUSB1")
            lidar.start()
            lidarConeBuff = [] #an appendable list, to which blobs are added. entries are removed when the blobs have been used
        
        if(bufferLandmarks):
            landmarkBufferLengthThreshold = 5 #if the number of landmarks in the buffer >= this threshold, then process them immediately (dont wait for the timer)
            landmarkBufferTimer = masterMap.clock()
            landmarkBufferTimeInterval = 0.25
        
        if(useSLAM):
            masterMap.car.slamData = masterMap.clock() #store time since last SLAM update in car.slamData
            #SlamUpdateInterval = landmarkBufferTimeInterval
            
        if(simulatePositionalDrift):
            # if(simulatePositionalDrift):
            #     self.car.simulationVariables = PH.phantomClass() #needlessly fancy/complicated, but versatile
            # masterMap.car.simulationVariables._put_(('position',), np.zeros(2)) #needlessly fancy
            # masterMap.car.simulationVariables._put_(('angle',), 0.0)
            
            if(simulatePositionalDrift):
                masterMap.car.simulationVariables = SC.simCar() #inefficient in terms of pickling size, but rather simple to implement
                masterMap.car.simulationVariables.position = masterMap.car.position.copy()
                masterMap.car.simulationVariables.angle = masterMap.car.angle
                
            # masterMap.car.simulationVariables = [masterMap.car.position.copy(), masterMap.car.angle, 0.0, 0.0] #[position, angle, velocity, steering]
            
            simDriftVelocityError = 0.1
            simDriftSteeringError = np.deg2rad(5)
        
        timeSinceLastUpdate = masterMap.clock()
        while (drawer.is_alive() if useDrawer else True):
            loopStart = masterMap.clock()
            
            if((masterMap.car.pathFolData.auto) if (masterMap.pathPlanningPresent and (masterMap.car.pathFolData is not None)) else False):
                PP.calcAutoDriving(masterMap)
                #masterMap.car.sendSpeedAngle(masterMap.car.desired_velocity, masterMap.car.desired_steering) #(spam) send instruction (or simulate doing so)
                if(not simulation):
                    connToCar.send((masterMap.car.desired_velocity, masterMap.car.desired_steering))
            
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
                if(connToCar.poll()):
                    carData = None #init var
                    while(connToCar.poll()):
                        carData = connToCar.recv()
                        #you can update for every feedback, or you can update for only the newest 
                    #print("received from car:", carData)
                    timestamp, steeringAngle, newTotalDist = carData #(unpack tuple)
                    dTime = timestamp - masterMap.car.lastFeedbackTimestamp
                    dDist = newTotalDist - masterMap.car.totalDistTraveled
                    masterMap.car.lastFeedbackTimestamp = timestamp
                    masterMap.car.totalDistTraveled = newTotalDist
                    masterMap.car.steering = steeringAngle
                    ## note: SLAM should replace most of this stuff, so the lack of real filetering here is temporary
                    if(dTime <= 0.0001):
                        print("bad timestamp?!", dTime)
                    else:
                        #masterMap.car.velocity = (masterMap.car.velocity + (dDist/dTime))/2 #i don't trust the (low-resolution) rotary encoder on the car enough to get the right velocity without some averaging)
                        masterMap.car.velocity = dDist/dTime
                        masterMap.car.update(dTime, dDist) #to be replaced by SLAM?
            
            ## lidar update:
            if(useLidar):
                if(not simulateLidar):
                    connToLidar.send(masterMap.car) #always send a position update
                while(connToLidar.poll()):
                    lidarConeBuff.append(connToLidar.recv())
            
            if( (((masterMap.clock() - landmarkBufferTimer) > landmarkBufferTimeInterval)
                 or (len(lidarConeBuff) > landmarkBufferLengthThreshold)) 
                   if bufferLandmarks else True):
                #if(((masterMap.clock() - landmarkBufferTimer) < landmarkBufferTimeInterval) if bufferLandmarks else False):
                #    print("landmarkBufferLengthThreshold crossed:", len(lidarConeBuff), ">", landmarkBufferLengthThreshold)
                
                if(useSLAM and ((not delaySLAMuntillDriving) or (delaySLAMuntillDriving and masterMap.car.pathFolData.auto))):
                    makeNewCones = True
                    if(masterMap.pathPlanningPresent and (masterMap.car.pathFolData is not None) and makeConesOnlyFirstLap):
                        makeNewCones = (masterMap.car.pathFolData.laps < 1) #only on the first lap
                    SLAM.updatePosition(masterMap, [lidarConeBuff, []], (1.0, 1.0), makeNewCones, storeBlob)
                else: ## backup code to process lidar data in a simpler (non-position correcting!) way
                    while(len(lidarConeBuff) > 0):
                        conePos, blob = lidarConeBuff[0]
                        #blobFirstPointTime = blob['timestamp']
                        #blobLastPointTime = blob['appendTimestamp']
                        makeCone(masterMap, conePos, blob)
                        lidarConeBuff.pop(0)
                
                lidarConeBuff = []
                if(bufferLandmarks):
                    landmarkBufferTimer = masterMap.clock()
                    #landmarkBufferTimer += landmarkBufferTimeInterval
            
            timeSinceLastUpdate = loopStart #save time (from start of loop) to be used next time
            
            while(connToSlaves.poll()):
                phantomMap = connToSlaves.recv()
                PH.mergePhantom(masterMap, phantomMap, UI.UIparser)
            pickleSize = masterSharedMem.pickle(masterMap) #pickle whole map every time (because you can't really pickle only the updated parts)
            #print("map pickleSize:", pickleSize) #debugging info
            
            loopEnd = masterMap.clock() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
            if((loopEnd-loopStart) < 0.015): #60FPS limiter (optional)
                time.sleep(0.0155-(loopEnd-loopStart))
            elif((loopEnd-loopStart) > 0.2):
                print("main process running slow", 1/(loopEnd-loopStart))
    finally:
        print("main ending")
        try:
            masterSharedMem.close()
            masterSharedMem.unlink()
            print("(main) sharedMem closing done")
        except:
            print("couldn't close sharedMem")
        try:
            connToSlaves.close()
            connToMaster.close()
            print("(main) pipe closing done")
        except: 
            print("couldn't close map pipes")
        if(useDrawer):
            try:
                drawer.stopLoop()
                print("(main) drawer stopping done")
            except:
                try:
                    if(drawer.is_alive()):
                        drawer.terminate()
                        drawer.join(3)
                        print("(main) drawer stopping maybe done")
                finally:
                    if(drawer.is_alive()):
                        print("couldn't stopLoop drawer")
        if(not simulation):
            try:
                connToCar.close()
                connToCarMaster.close()
                print("(main) carSerial pipe closing done")
            except: 
                print("couldn't close carSerial pipes")
            try:
                carSerial.stopLoop()
                print("(main) carSerial stopping done")
            except:
                try:
                    if(carSerial.is_alive()):
                        carSerial.terminate()
                        carSerial.join(3)
                        print("(main) carSerial stopping maybe done")
                finally:
                    if(carSerial.is_alive()):
                        print("couldn't stopLoop carSerial")
        if(useLidar):
            try:
                connToLidar.close()
                connToLidarMaster.close()
                print("(main) lidar pipe closing done")
            except: 
                print("couldn't close lidar pipes")
            try:
                lidar.stopLoop()
                print("(main) lidar stopping done")
            except:
                try:
                    if(lidar.is_alive()):
                        lidar.terminate()
                        lidar.join(3)
                        print("(main) lidar stopping maybe done")
                finally:
                    if(lidar.is_alive()):
                        print("couldn't stopLoop lidar")
        if(useRemote):
            try: #stop 'remote' process last, becuase it usually gives the most trouble
                remote.stopLoop()
                print("(main) remote stopping done")
            except:
                try:
                    if(remote.is_alive()):
                        remote.terminate()
                        remote.join(3)
                        print("(main) remote stopping maybe done")
                finally:
                    if(remote.is_alive()):
                        print("couldn't stopLoop remote")
    print("main ended")
