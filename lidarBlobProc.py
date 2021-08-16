

##TBD: when the car moves quickly, the lidar may need to use an intermittend positional update function (like simCar.update) to better estimate where it is inbetween real updates
## this becomes more relevant the faster the car goes, but there may even be a second usage,
## it should(?) be possible to run the position update function with negative time, to better account for the delays in the communication pipeline(s) (both the serial connection from the lidar, and blobifying take time)
## the code would look something like 'lidarPos = intermitUpdate(dTime=(time_since_last_real_update - lidarCommDelayConstant))'

global RANGE_LIMIT, MIN_BLOB_CONE_LEN
RANGE_LIMIT = 750 #lidar range limit in mm (blobs that partly exceed the limit will be cut off, so be careful
MIN_BLOB_CONE_LEN = 3 #a blob must have at least this many points to be return a cone pos

import camsense_X1 as CX
#import lidarBlobs as LB
import lidarBlobsNoList as LB #a little faster

from Map import Map
import GF.generalFunctions as GF

import numpy as np
import time

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)


def uponBlobExist(blob): #a little (callback) for when blobs are finished. (stored in blob.uponExist)
    connToMaster = blob.extraData
    blob.extraData = None #i dont want to send these over the pipe (and they're no longer needed anyway)
    blob.uponExist = None #remove the pointer to this callback function from the blob
    posIsValid, conePos = blobToConePos(blob)
    if(posIsValid): #only send if the pos was successfully calculated
        connToMaster.send((conePos, blob))

def blobToConePos(blob): #calculate the position the cone would have over here, to save some processing time on the main thread
    global MIN_BLOB_CONE_LEN
    adjustedConeDiam = Map.Cone.coneDiam * 0.5 #TBD: calculate the diamter of the cone AT THE HEIGHT OF THE LIDAR (this does not have to be done dynamically, it can be constant)
    if(len(blob.points) < MIN_BLOB_CONE_LEN):
        print("warning: blob too few points to make into cone")
        return(False, None)
    if(len(blob.origins) != len(blob.points)):
        print("warning: less origins than points!:", len(blob.origins), len(blob.points))
    coneCenterPos = [[],[]]
    #coneCenterPos = np.empty((2,len(blob._lines))) #TBD
    if(len(blob._lines)>0):
        perpAdd = np.pi/2 # you could assume (or calculate only once) the direction of the perpendicular angle, by knowing the rotation direction of the lidar
        #sim1.debugLines = []
        if(len(blob.origins)>0): #if there is an origin point available
            baseAngle = GF.get_norm_angle_between(blob.origins[int(len(blob.origins)/2)], blob.points[int(len(blob.points)/2)], 0.0)
            if(abs(GF.radDiff(blob.lineData(0)[1]-perpAdd, baseAngle)) < abs(GF.radDiff(blob.lineData(0)[1]+perpAdd, baseAngle))):
                perpAdd = -perpAdd
                print("warning: lidar measurement not simply CW?")
            ## alternatively, one could use the fact that the lidar always spins 1 way (CW), and therefore the points are always added in that order, so angles between them have a constant relation to the origin
        for i in range(len(blob._lines)):
            #perpAngle = ((blob.lineData(i)[1]+perpAdd) if (abs(GF.radDiff(blob.lineData(i)[1]+perpAdd, baseAngle)) < abs(GF.radDiff(blob.lineData(i)[1]-perpAdd, baseAngle))) else (blob.lineData(i)[1]-perpAdd))
            superAdjustedConeRadius = np.cos(np.arcsin((blob.lineData(i)[2]/2) / (adjustedConeDiam/2))) * (adjustedConeDiam/2)
            perpAngle = blob.lineData(i)[1] + perpAdd
            lineCenter = GF.distAnglePosToPos(blob.lineData(i)[2]/2, blob.lineData(i)[1], blob.points[i])
            conePos = GF.distAnglePosToPos(superAdjustedConeRadius, perpAngle, lineCenter)
            coneCenterPos[0].append(conePos[0]);   coneCenterPos[1].append(conePos[1]) #format it as [xpositions, ypositions] lists, to make getting average() easier
        conePos = np.array([GF.average(np.array(coneCenterPos[0])), GF.average(np.array(coneCenterPos[1]))])
        return(True, conePos)
    else:
        if(len(blob.origins)>0):
            conePos = GF.distAnglePosToPos(adjustedConeDiam, GF.get_norm_angle_between(blob.origins[0], blob.points[0], 0.0), blob.points[0])
            return(True, conePos)
        else:
            print("1 point, no origin")
            return(False, None)
    

def packetCallback(lidarSelf, newPacket, pointsAdded):
    startAngle = newPacket['startAngle']/CX.DEG_DIV
    angleDif = GF.degDiff(newPacket['startAngle']/CX.DEG_DIV, newPacket['endAngle']/CX.DEG_DIV) #(rollover proof) IMPORTANT: packet angles are still integers, divide by 64 to get degrees (0 to 360)
    angleStep = angleDif/7 #note: there are actually 8 datapointes per packet, but i'm assuming the endAngle is AT the 8th datapoint (and the startAngle is AT the first)
    global RANGE_LIMIT
    for i in range(newPacket['dataFilled']-pointsAdded, newPacket['dataFilled']): #if it's not a new packet, but an existing one with new points, this will skip the old ones
        if((newPacket['measurements'][i] < RANGE_LIMIT) and (newPacket['measurements'][i] != CX.NO_MEASUREMENT)):
            angle = startAngle+angleStep*i
            ## correctedLidarAngle:  get 'middle' angle, convert to (-180,180) with derRoll, convert to radians, set 0-angle at car-rear (instead of front) with radInv and *-1 because lidar degrees are CW and Map rotation is CCW
            lidarAngleError = np.deg2rad(-15) #TBD save this somewhere else, maybe in camsense_X1
            correctedLidarAngle = (-1)*GF.radInv(np.deg2rad(angle)) + lidarAngleError #NOTE: radRoll (or a function that calls it, like radInv) is recommended, also: the *(-1) is to fix CW/CCW issues
            
            lidarPos = lidarSelf.callbackExtraArg[1][0] #the position of the lidar is used to get the point position. Currently, the lidar is mounted directly above the car's (rear-axle/center of rotation) position
            lidarMountAngle = lidarSelf.callbackExtraArg[1][1]
            pointPos = GF.distAnglePosToPos(newPacket['measurements'][i]/1000.0, lidarMountAngle + correctedLidarAngle, lidarPos) #get map position from relative position (asuming lidar is mounter on car)
            isNewBlob, newBlob = LB.blobify(pointPos, lidarPos, lidarSelf.callbackExtraArg[2]())
            
            if(isNewBlob):
                #newBlob.uponExist = lambda blobObj : print("exist:", len(blobObj.points), len(blobObj._lines))  #example
                #newBlob.uponDeletion = lambda blobObj, reason : print("delete:", len(blobObj.points), len(blobObj._lines), blobObj.exists, reason)  #example
                newBlob.uponExist = uponBlobExist
                newBlob.uponExistAppendTimeout = (60.0/lidarSelf.RPM())*0.25 #IMPORTANT: (seconds per rotation) * quarter, this timout also makes sure blobs are not appended to on subsequent rotations
                newBlob.extraData = lidarSelf.callbackExtraArg[0] #pass connToMaster

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
            lidar = CX.camsense_X1(self.comPort, initMap.clock)
            lidarInitPos = (initMap.car.position, initMap.car.angle)
            lidar.callbackExtraArg = [self.connToMaster, lidarInitPos, initMap.clock]
            lidar.postParseCallback = packetCallback #once everything is ready, start sending data
            
            lastCarUpdateTime = time.time()
            warningPrinted = False #just a debugging variable
            
            while(self.keepalive.get_value()):
                if(self.connToMaster.poll()):
                    lastCarUpdateTime = time.time()
                    lidar.callbackExtraArg[1] = self.connToMaster.recv() #update the lidar position data
                    warningPrinted = False #debugging var
                elif((time.time() - lastCarUpdateTime) > 0.5): #if no lidar position update has been received for too long, then fall back to unpickling the whole map
                    if(not warningPrinted):
                        warningPrinted = True
                        print("(lidarBlobProc) warning!: no car position updates received through self.connToMaster, defaulting to unpickling the whole map object (slow)")
                    if(slaveSharedMem.poll()): #if a new map is available
                        initMap = slaveSharedMem.readObj(self.keepalive) #get master map (there is no real reason to overwrite initMap, but why not right)
                        lidar.callbackExtraArg[1] = (initMap.car.position, initMap.car.angle)
                
                lidar.run() #read serial data and when/if a full packet is ready, attempt to blobify it (using the packetCallback), and once a blob is ready, send it to the master process
                LB.checkBlobAge(initMap.clock())
                #time.sleep(0.002) #the lidar process can't really afford to sleep
        finally:
            try:
                lidar._serialPort.close()
                print("closed lidar serial port")
            except:
                print("coudln't close lidar serial port")

# def initLidarBlobProcess(lidarInitArgs, lidarStartPos):#you might want to have a slight delay between 
#     processPipe = MP.Pipe()
#     processDataLock = MP.Lock() #not really used, but whatever
#     lidarProcess = MP.Process(target=lidarProcessTarget, args=(lidarInitArgs, lidarStartPos, "liDaRam", processDataLock, processPipe[1]))
#     lidarProcess.start()
#     lidarSetupReady = False
#     lidarData = None;   sharedMem = None
#     while(not lidarSetupReady):
#         if(processPipe[0].poll()):
#             gottenThing = processPipe[0].recv()
#             if((((gottenThing[0] == "lidarRDY") and (len(gottenThing)==2)) if (type(gottenThing[0]) is str) else False) if (type(gottenThing) is tuple) else False):
#                 print("got memName:", gottenThing[1])
#                 lidarData, sharedMem = CX.sharedMemArray(gottenThing[1])
#                 lidarSetupReady = True
#         else:
#             #print("waiting for lidarData")
#             time.sleep(0.2)
#     return(lidarProcess, processPipe[0], processDataLock, lidarData, sharedMem)


# if __name__ == "__main__": #test code
#     lidarProcess, lidarPipe, lidarLock, lidarData, lidarMem = initLidarBlobProcess(('COM7',), [np.zeros(2), 0.0])
#     try:
#         print("(test) main init")
#         sendPosTimer = time.time()
#         while(True):
#             testStartTime = time.time()
            
#             if(lidarPipe.poll()):
#                 print("got lidarPipe data:", lidarPipe.recv())
#             if((time.time()-sendPosTimer)>2.0):
#                 sendPosTimer = time.time()
#                 lidarPipe.send(("posUpdate", np.array([0.0, 0.0]), 0.0))
                
#             testEndTime = time.time()
#             if((testEndTime-testStartTime)>0.001):
#                 print("test time:",testEndTime-testStartTime)
#             time.sleep(0.1)
#     finally:
#         stopLidarProc(lidarPipe, lidarMem, lidarProcess)