import camsense_X1 as CX
import lidarBlobs as LB
import multiprocessing as MP

import generalFunctions as GF

import numpy as np
import time



def uponBlobExist(blob):
    processPipeCon = blob.extraData
    blob.extraData = None #i dont want to send these over the pipe (and they're no longer needed anyway)
    blob.uponExist = None
    processPipeCon.send(("blob", blob, time.time()))


def callbackFunc(lidarSelf, newPacket, pointsAdded):
    startAngle = newPacket['startAngle']/CX.DEG_DIV
    angleDif = GF.degDiff(newPacket['startAngle']/CX.DEG_DIV, newPacket['endAngle']/CX.DEG_DIV) #(rollover proof) IMPORTANT: packet angles are still integers, divide by 64 to get degrees (0 to 360)
    angleStep = angleDif/7 #note: there are actually 8 datapointes per packet, but i'm assuming the endAngle is AT the 8th datapoint (and the startAngle is AT the first)
    debugThresh = 500
    for i in range(newPacket['dataFilled']-pointsAdded, newPacket['dataFilled']): #if it's not a new packet, but an existing one with new points, this will skip the old ones
        if((newPacket['measurements'][i] < debugThresh) and (newPacket['measurements'][i] != CX.NO_MEASUREMENT)):
            angle = startAngle+angleStep*i
            ## correctedLidarAngle:  get 'middle' angle, convert to (-180,180) with derRoll, convert to radians, set 0-angle at car-rear (instead of front) with radInv and *-1 because lidar degrees are CW and Map rotation is CCW
            lidarAngleError = np.deg2rad(-15) #TBD save this somewhere else, maybe in camsense_X1
            correctedLidarAngle = (-1)*GF.radInv(np.deg2rad(angle)) + lidarAngleError #NOTE: radRoll is done in radInv, if you remove this please use degRoll or radRoll instead
            
            lidarPos = lidarSelf.callbackExtraArg[1][0] #the position of the lidar is used to get the point position
            lidarMountAngle = lidarSelf.callbackExtraArg[1][1]
            pointPos = GF.distAnglePosToPos(newPacket['measurements'][i]/1000.0, lidarMountAngle + correctedLidarAngle, lidarPos) #get map position from relative position (asuming lidar is mounter on car)
            isNewBlob, newBlob = LB.blobify(pointPos, lidarPos)
            
            if(isNewBlob):
                #newBlob.uponExist = lambda blobObj : print("exist:", len(blobObj.points), len(blobObj._lines))  #example
                #newBlob.uponDeletion = lambda blobObj, reason : print("delete:", len(blobObj.points), len(blobObj._lines), blobObj.exists, reason)  #example
                newBlob.uponExist = uponBlobExist
                newBlob.uponExistAppendTimeout = (60.0/lidarSelf.RPM())*0.25 #IMPORTANT: (seconds per rotation) * quarter, this timout also makes sure blobs are not appended to on subsequent rotations
                newBlob.extraData = lidarSelf.callbackExtraArg[0] #pass processPipeCon

def lidarProcessTarget(lidarInitArgs, lidarStartPos, sharedMemName, processDataLock, processPipeCon):
    lidar = CX.camsense_X1(*lidarInitArgs)
    lidar.callbackExtraArg = [processPipeCon, lidarStartPos]
    print("MPvars init:", lidar.startMultiProcessVars(sharedMemName, processDataLock)) #returns True on success
    processPipeCon.send(("lidarRDY", lidar.getSharedMemName()))
    lidar.postParseCallback = callbackFunc #once everything is ready, start sending data
    try:
        while(True):
            ##normal operation:
            lidar.run()
            LB.checkBlobAge()
            if(processPipeCon.poll()):
                gottenThing = processPipeCon.recv()
                if((((gottenThing[0] == "posUpdate") and (len(gottenThing)==3)) if (type(gottenThing[0]) is str) else False) if (type(gottenThing) is tuple) else False):
                    lidar.callbackExtraArg[1][0] = gottenThing[1]
                    lidar.callbackExtraArg[1][1] = gottenThing[2]
                else:
                    print("lidarPipe invalid object:", gottenThing)
            #time.sleep(0.002)
    finally:
        try:
            lidar._serialPort.close()
            print("closed lidar serial port")
        except:
            print("coudln't close lidar serial port")
        try:
            lidar.closeSharedMem()
        except:
            print("couldnt close lidar sharedMem")

def initLidarBlobProcess(lidarInitArgs, lidarStartPos):#you might want to have a slight delay between 
    processPipe = MP.Pipe()
    processDataLock = MP.Lock() #not really used, but whatever
    lidarProcess = MP.Process(target=lidarProcessTarget, args=(lidarInitArgs, lidarStartPos, "liDaRam", processDataLock, processPipe[1]))
    lidarProcess.start()
    lidarSetupReady = False
    lidarData = None;   sharedMem = None
    while(not lidarSetupReady):
        if(processPipe[0].poll()):
            gottenThing = processPipe[0].recv()
            if((((gottenThing[0] == "lidarRDY") and (len(gottenThing)==2)) if (type(gottenThing[0]) is str) else False) if (type(gottenThing) is tuple) else False):
                print("got memName:", gottenThing[1])
                lidarData, sharedMem = CX.sharedMemArray(gottenThing[1])
                lidarSetupReady = True
        else:
            #print("waiting for lidarData")
            time.sleep(0.2)
    return(lidarProcess, processPipe[0], processDataLock, lidarData, sharedMem)


def stopLidarProc(lidarPipe, lidarMem, lidarProcess):
    try:
        lidarPipe.close()
    except:
        print("couldn't close lidarPipe")
    try:
        lidarMem.close()
        lidarMem.unlink()
    except:
        print("couldn't close/unlink lidarMem")
    try:
        lidarProcess.join()
    except:
        print("couldn't join lidarProcess")

class carMCUserialProcess(MP.Process):
    """a multiprocessing carMCUserial"""
    def __init__(self, connToMaster, sharedMemInit, comPort=None, autoFind=False, autoreconnect=True):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.sharedMemInit = sharedMemInit
        #self.connectAtInit = connectAtInit
        self.comPort = comPort
        self.autoFind = autoFind
        self.autoreconnect = autoreconnect
        
        self.keepalive = MP.Semaphore() # this (functional) boolean stops the while() loop in run()
    
    def run(self):
        try:
            print("TBD")
        finally:
            print("carMCUserialProcess ending")
            try:
                carConn.disconnect()
            except:
                print("couldn't disconnect carSerial")
                
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