
#NOTE: i've solved a problem in a bit of a terrible way: a master record of where the cone actually is MUST be kept, for the simulation to add error correctly.
# instead of saving a seperate master record (which has all sorts of synchronization issues and stuff), i just keep the oldest lidar data (the first blob) forever (i keep deleting the 2nd oldest blob history)
# see 'makeCone()' and look for a 'slamData.pop(1)' if you solved this itterative-error-addition-drift another way

STD_DEV_LIDAR_ERROR = 2.0 #in mm !
#DATAPOINTS_PER_DISTANCE_NUMERATOR = 3.6 #calibrated, not calculated, but the lidar i'm using has roughly 0.75deg measurement freq, so 


RANGE_LIMIT = 1300 #lidar range limit in mm (blobs that partly exceed the limit will be cut off, so be careful

import lidarBlobs as LB

from Map import Map
import GF.generalFunctions as GF

import numpy as np
import time

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)

DATAPOINTS_PER_DISTANCE_NUMERATOR = 3.6 * (Map.Cone.coneLidarDiam / (Map.Cone.coneDiam/2))


class lidarProcess(MP.Process):
    """a multiprocessing lidar (serial connection and blobifier)"""
    def __init__(self, connToMaster, sharedMemInit, comPort=None):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.sharedMemInit = sharedMemInit
        self.comPort = comPort
        #self.autoreconnect = autoreconnect
        
        self.keepalive = MP.Semaphore() # this (functional) boolean stops the while() loop in run()
    
    def run(self):
        try:
            slaveSharedMem = SM.sharedMemReadWrapper(*self.sharedMemInit)
            initMap = slaveSharedMem.readObj(self.keepalive) #wait for the first map object to come in (with no timeout?)
            
            # lastCarUpdateTime = time.time()
            # warningPrinted = False #just a debugging variable
            
            while(self.keepalive.get_value()):
                loopStart = time.time()
                # while(self.connToMaster.poll()):
                #     lastCarUpdateTime = time.time()
                #     self.connToMaster.recv() #just throw away the data
                #     warningPrinted = False #debugging var
                # if((time.time() - lastCarUpdateTime) > 0.5): #if no lidar position update has been received for too long, then fall back to unpickling the whole map
                #     if(not warningPrinted):
                #         warningPrinted = True
                #         print("(lidarBlobProcSim) warning!: no car position updates received through self.connToMaster, defaulting to unpickling the whole map object (slow)")
                if(slaveSharedMem.poll()): #if a new map is available
                    initMap = slaveSharedMem.readObj(self.keepalive) #get master map (there is no real reason to overwrite initMap, but why not right)
                
                lidarPos = initMap.car.position
                if(hasattr(initMap.car, 'simulationVariables')):
                    if(type(initMap.car.simulationVariables) is list):
                        #print("found simulated real position:", initMap.car.simulationVariables)
                        lidarPos = initMap.car.simulationVariables[0]
                    elif(initMap.car.simulationVariables is not None): #(type(initMap.car.simulationVariables) is SC.simCar):
                        lidarPos = initMap.car.simulationVariables.position
                
                nearbyConeList = initMap.distanceToCone(lidarPos, None, sortBySomething='SORTBY_ANGL', simpleThreshold=(RANGE_LIMIT/1000.0))
                for i in range(len(nearbyConeList)):
                    cone, distAngle = nearbyConeList[i]
                    outerEdgeAngles = [distAngle[1] - np.arctan2(Map.Cone.coneDiam/2, distAngle[0]), distAngle[1] + np.arctan2(Map.Cone.coneDiam/2, distAngle[0])]
                    nearbyConeList[i].append(outerEdgeAngles)
                    makeNewBlob = True
                    for j in range(i):
                        #radRangeOne only works with GF_noNumba, but it could be slightly faster maybe
                        if(not makeNewBlob):
                            continue #little extra loop speed
                        if(GF.radRange(outerEdgeAngles[0], nearbyConeList[j][2][0], nearbyConeList[j][2][1]) or 
                           GF.radRange(outerEdgeAngles[1], nearbyConeList[j][2][0], nearbyConeList[j][2][1])):
                            if(distAngle[0] < nearbyConeList[j][1][0]): #if that cone is further away (and therefore obstructed by the current cone)
                                if(len(nearbyConeList[j]) > 3):# if a blob was made for that cone,
                                    #print("(lidarBlobProcSim) popping blob:", nearbyConeList[j][3])
                                    nearbyConeList[j].pop(3) #delete that (visually obstructed) blob
                            else: # if that cone is closer than the current one (and so the current one is visually obstructed)
                                makeNewBlob = False
                    
                    if(makeNewBlob):
                        adjustedConeDiam = Map.Cone.coneLidarDiam #TBD: calculate the diamter of the cone AT THE HEIGHT OF THE LIDAR (this does not have to be done dynamically, it can be constant)
                        lidarPoints = int(6) #init var
                        if(distAngle[0] > 0.01):#avoid divide by zero, and generally approaching unreasonable numbers
                            lidarPoints = int(DATAPOINTS_PER_DISTANCE_NUMERATOR / distAngle[0]) #the number of datapoints decreases with distance
                        lidarPointsAngleRange = np.deg2rad(60)
                        invCarToConeAngle = GF.radInv(distAngle[1])
                        dataPointAngles = [((-lidarPointsAngleRange + ((lidarPointsAngleRange/(lidarPoints-1))*i*2))+invCarToConeAngle) for i in range(lidarPoints)]
                        realActualConePos = (cone.position if (cone.slamData is None) else cone.slamData[0][0]) #the simulated lidar must use a constant cone position to 'measure' from
                        if(hasattr(initMap.car, 'simulationVariables')):
                            trueLidarPos = None;  trueLidarAngle = None
                            if(type(initMap.car.simulationVariables) is list):
                                #print("found simulated real position:", initMap.car.simulationVariables)
                                trueLidarPos = initMap.car.simulationVariables[0]
                                trueLidarAngle = initMap.car.simulationVariables[1]
                            elif(initMap.car.simulationVariables is not None): #(type(initMap.car.simulationVariables) is SC.simCar):
                                trueLidarPos = initMap.car.simulationVariables.position
                                trueLidarAngle = initMap.car.simulationVariables.angle
                            if(trueLidarPos is not None):
                                distAngleFromTrue = GF.distAngleBetwPos(trueLidarPos, realActualConePos)
                                angleError = initMap.car.angle - trueLidarAngle
                                realActualConePos = GF.distAnglePosToPos(distAngleFromTrue[0], distAngleFromTrue[1] + angleError, initMap.car.position)
                        newBlob = None;  appendSuccess=True; #init var
                        for j in range(len(dataPointAngles)):
                            if(not appendSuccess):
                                continue
                            randomMeasurementError = np.random.normal() * (STD_DEV_LIDAR_ERROR/1000) #add error in a normal distribution (you could scale with dist, but whatever)
                            pointPos = GF.distAnglePosToPos((adjustedConeDiam/2) + randomMeasurementError, dataPointAngles[j], realActualConePos)
                            if(j == 0):
                                newBlob = LB.blobCreate(pointPos, initMap.car.position, initMap.clock())
                            else:
                                appendSuccess = LB.blobAppend(newBlob, pointPos, initMap.car.position, initMap.clock())
                                # if(not appendSuccess):
                                #     print("(lidarBlobProcSim) bad blob append?")
                        if(appendSuccess):
                            nearbyConeList[i].append(newBlob)
                
                invalidConePoses = 0
                for i in range(len(nearbyConeList)):
                    if(len(nearbyConeList[i]) > 3):
                        posIsValid, conePos = LB.blobToConePos(nearbyConeList[i][3])
                        if(posIsValid): #only send if the pos was successfully calculated
                            self.connToMaster.send((conePos, nearbyConeList[i][3]))
                        else:
                            #print("blob with invalid conePos:", nearbyConeList[i][3])
                            invalidConePoses += 1
                if(invalidConePoses > 0):
                    print("invalidConePoses:", invalidConePoses)
                
                loopEnd = time.time() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
                if((loopEnd-loopStart) < 0.2): #5 'rotations' per second limiter (optional)
                    time.sleep(0.2-(loopEnd-loopStart))
                else:
                    print("(lidarBlobProcSim) running slow", 1.0/(loopEnd-loopStart))
        finally:
            print("simulated lidar ending")