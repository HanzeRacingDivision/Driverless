
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


###TBD: 
# lidar
# slam
# documentation
# camera
# precompiling numba once (not repeatedly) (mostly done)


simulation = True


from Map import Map

# import map_loader as ML  #note: imported in __main__
# import coneConnecting as CC
# import pathFinding    as PF
import pathPlanningTemp as PP

if(simulation):
    import simulatedCar as SC
else:
    import carMCUclass  as RC

# import drawDriverless as DD
# import pygameUI       as UI

#import mapTransSock   as MS

import phantom        as PH

import time
#import numpy as np
import sys #used for cmdline arguments

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)


class masterMapClass(Map):
    def __init__(self):
        Map.__init__(self) #init map class
        
        if(simulation):
            self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        else:
            self.car = RC.realCar(self.clock)
        
        #enable components here
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
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
                if(self.asynchronous):
                    if(slaveSharedMem.poll()): #if a new map is available
                        drawer.mapToDraw = slaveSharedMem.readObj(self.keepalive) #get master map
                        drawer3D.mapToDraw = drawer.mapToDraw
                else:
                    drawer.mapToDraw = slaveSharedMem.readNewObj(self.keepalive) #get master map
                    drawer3D.mapToDraw = drawer.mapToDraw
                
                if(drawer.extraViewMode):
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



if __name__ == "__main__":
    try:
        import generalUI      as UI
        import mapTransSock   as MS
        
        masterMap = masterMapClass()
        
        import map_loader as ML
        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            ML.mapLoader.load_map(sys.argv[1], masterMap)
        
        connToSlaves, connToMaster = MP.Pipe(False) #disable duplex (for now)
        masterSharedMem = SM.makeNewSharedMem('driverless', 10*1000*1000) #note: max (pickled) object size is HALF of this size, due to the double-buffering
        drawer = drawProcess(connToMaster, masterSharedMem.passToProcess, [1280, 720], False)
        drawer.start()
        
        remote = MS.remoteProcess(connToMaster, masterSharedMem.passToProcess, '', 65432, 0.2)
        remote.start()
        
        connToCar = None; connForSerial = None; carSerial = None #init var
        if(not simulation):
            connToCar, connForSerial = MP.Pipe(True) #a two-way pipe to the carMCUserialProcess (which (encodes/decodes and) passes it on to/from the serial connection)
            carSerial = RC.carMCUserialProcess(connForSerial, masterSharedMem.passToProcess, comPort="COM5", autoFind=False, autoreconnect=True)
            carSerial.start()
        
        timeSinceLastUpdate = masterMap.clock()
        while drawer.is_alive():
            rightNow = masterMap.clock()
            
            if((masterMap.car.pathFolData.auto) if (masterMap.pathPlanningPresent and (masterMap.car.pathFolData is not None)) else False):
                PP.calcAutoDriving(masterMap)
                #masterMap.car.sendSpeedAngle(masterMap.car.desired_velocity, masterMap.car.desired_steering) #(spam) send instruction (or simulate doing so)
                if(not simulation):
                    connToCar.send((masterMap.car.desired_velocity, masterMap.car.desired_steering))
            
            if(simulation):
                masterMap.car.simulateFeedback()
                masterMap.car.update(rightNow - timeSinceLastUpdate) #to be replaced by SLAM?
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
            
            timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
            
            while(connToSlaves.poll()):
                phantomMap = connToSlaves.recv()
                PH.mergePhantom(masterMap, phantomMap, UI.UIparser)
            masterSharedMem.pickle(masterMap) #pickle whole map every time (because you can't really pickle only the updated parts)
            
            FPSrightNow = masterMap.clock() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
            if((FPSrightNow-rightNow) < 0.015): #60FPS limiter (optional)
                time.sleep(0.0155-(FPSrightNow-rightNow))
        
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
                print("couldn't stopLoop drawer")
        try:
            remote.stopLoop()
            print("(main) remote stopping done")
        except:
            try:
                if(remote.is_alive()):
                    remote.terminate()
                    remote.join(3)
                    print("(main) remote stopping maybe done")
            finally:
                print("couldn't stopLoop remote")
        if(not simulation):
            try:
                connToCar.close()
                connForSerial.close()
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
                    print("couldn't stopLoop carSerial")
    print("main ended")
