import camsense_X1 as CX
import lidarBlobs as LB
import time
import numpy as np

import generalFunctions as GF
from Map import Map
import map_loader as ML
import coneConnecting as CC
import pathFinding    as PF
import pathPlanningTemp as PP
import simulatedCar   as SC
import carMCUclass as RC

import mapTransSock   as MS
import threading as thr

import sys #used for importing files (map_loader) from commandline (DOS run argument)

class pygamesimLocal(ML.mapLoader, CC.coneConnecter, PF.pathFinder, PP.pathPlanner):
    def __init__(self):
        Map.__init__(self) #init map class
        ML.mapLoader.__init__(self)
        
        # self.clockSet(simClock) #an altered clock, only for simulations where the speed is faster/slower than normal
        #self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
        self.car = RC.realCar(self.clock, comPort='/dev/ttyUSB1')
        
        CC.coneConnecter.__init__(self)
        PF.pathFinder.__init__(self)
        PP.pathPlanner.__init__(self)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerData()



global debugThresh
debugThresh = 750 #only lidar data within this radius


def makeCone(blob):
    adjustedConeDiam = Map.Cone.coneDiam * 0.5 #TBD: calculate the diamter of the cone AT THE HEIGHT OF THE LIDAR (this does not have to be done dynamically, it can be constant)
    global sim1
    if(len(blob.origins) != len(blob.points)):
        print("less origins than points!:", len(blob.origins), len(blob.points))
    coneCenterPos = [[],[]]
    if(len(blob._lines)>0):
        perpAdd = np.pi/2 # you could assume (or calculate only once) the direction of the perpendicular angle, by knowing the rotation direction of the lidar
        #sim1.debugLines = []
        if(len(blob.origins)>0): #if there is an origin point available
            baseAngle = GF.get_norm_angle_between(blob.origins[int(len(blob.origins)/2)], blob.points[int(len(blob.points)/2)], 0.0)
            if(abs(GF.radDiff(blob.lineData(0)[1]-perpAdd, baseAngle)) < abs(GF.radDiff(blob.lineData(0)[1]+perpAdd, baseAngle))):
                perpAdd = -perpAdd
            ## alternatively, one could use the fact that the lidar always spins 1 way (CW), and therefore the points are always added in that order, so angles between them have a constant relation to the origin
            #sim1.debugLines.append([0, sim1.realToPixelPos(blob.origins[int(len(blob.origins)/2)]), sim1.realToPixelPos(blob.points[int(len(blob.points)/2)]), 2])
        for i in range(len(blob._lines)):
            #perpAngle = ((blob.lineData(i)[1]+perpAdd) if (abs(GF.radDiff(blob.lineData(i)[1]+perpAdd, baseAngle)) < abs(GF.radDiff(blob.lineData(i)[1]-perpAdd, baseAngle))) else (blob.lineData(i)[1]-perpAdd))
            perpAngle = blob.lineData(i)[1] + perpAdd
            conePos = GF.distAnglePosToPos(adjustedConeDiam/2, perpAngle, blob.points[i])
            coneCenterPos[0].append(conePos[0]);   coneCenterPos[1].append(conePos[1]) #format it as [xpositions, ypositions] lists, to make getting average() easier
            #sim1.debugLines.append([0, sim1.realToPixelPos(blob.points[i]), sim1.realToPixelPos(blob.points[i+1]), 0])
            #sim1.debugLines.append([0, sim1.realToPixelPos(blob.points[i]), sim1.realToPixelPos(conePos), 1])
    else:
        if(len(blob.origins)>0):
            conePos = GF.distAnglePosToPos(Map.Cone.coneDiam, GF.get_norm_angle_between(blob.origins[0], blob.points[0], 0.0), blob.points[0])
        else:
            print("1 point, no origin")
            conePos = blob.points[0]
        coneCenterPos[0].append(conePos[0]);   coneCenterPos[1].append(conePos[1])
    conePos = [GF.average(coneCenterPos[0]), GF.average(coneCenterPos[1])]
    overlaps, overlappingCone = sim1.overlapConeCheck(conePos)
    if(overlaps):
        ## SLAM code should replace this
        if(type(overlappingCone.slamData) is list):
            overlappingCone.slamData.append([conePos, sim1.clock(), [v for v in sim1.car.position], sim1.car.velocity, overlappingCone.slamData[-1][-1]]) #cone position, timestamp, car position, car velocity, totalSpotCount
            if(len(overlappingCone.slamData)>10): #limit number of sightings to remember
                overlappingCone.slamData.pop(0) #delete oldest data
            overlappingCone.position[0] = GF.average([item[0][0] for item in overlappingCone.slamData])
            overlappingCone.position[1] = GF.average([item[0][1] for item in overlappingCone.slamData])
            blob.extraData = overlappingCone
    else:
        ## SLAM code could replace this
        leftOrRight = (GF.get_norm_angle_between(sim1.car.position, conePos, sim1.car.angle) < 0.0) #if the angle relative to car is negative (CW), it's a right-side cone
        conePlaceSuccess, coneInList = Map.addCone(conePos, leftOrRight, False)
        #if(conePlaceSuccess) # overlap check already done earlier
        coneInList.slamData = [[conePos, sim1.clock(), [v for v in sim1.car.position], sim1.car.velocity, 1]]
        blob.extraData = coneInList
    #blob.uponDeletion = delCone
    #blob.uponExist = None #this is just to mark that the function has been called (alternatively, check extraData)

def callbackFunc(lidarSelf, newPacket, pointsAdded):
    if((newPacket is not None) and (pointsAdded > 0)):
        startAngle = newPacket.startAngle/CX.DEG_DIV
        angleDif = GF.degDiff(newPacket.startAngle/CX.DEG_DIV, newPacket.endAngle/CX.DEG_DIV) #(rollover proof) IMPORTANT: packet angles are still integers, divide by 64 to get degrees (0 to 360)
        angleStep = angleDif/7 #note: there are actually 8 datapointes per packet, but i'm assuming the endAngle is AT the 8th datapoint (and the startAngle is AT the first)
        global debugThresh, sim1
        #LB.checkBlobAge(sim1.clock)
        for i in range(len(newPacket.measurements)-pointsAdded, len(newPacket.measurements)): #if it's not a new packet, but an existing one with new points, this will skip the old ones
            if((newPacket.measurements[i] < debugThresh) and (newPacket.measurements[i] != CX.NO_MEASUREMENT)):
                angle = startAngle+angleStep*i
                ## correctedLidarAngle:  get 'middle' angle, convert to (-180,180) with derRoll, convert to radians, set 0-angle at car-rear (instead of front) with radInv and *-1 because lidar degrees are CW and Map rotation is CCW
                lidarAngleError = np.deg2rad(-15) #TBD save this somewhere else, maybe in camsense_X1
                correctedLidarAngle = (-1)*GF.radInv(np.deg2rad(angle)) + lidarAngleError #NOTE: radRoll is done in radInv, if you remove this please use degRoll or radRoll instead
                carPos = sim1.car.getRearAxlePos()
                pointPos = GF.distAnglePosToPos(newPacket.measurements[i]/1000, sim1.car.angle + correctedLidarAngle, carPos) #get map position from relative position (asuming lidar is mounter on car)
                isNewBlob, newBlob = LB.blobify(pointPos, carPos, sim1.clock)
                if(isNewBlob):
                    #newBlob.uponExist = lambda blobObj : print("exist:", len(blobObj.points), len(blobObj._lines))  #example
                    #newBlob.uponDeletion = lambda blobObj, reason : print("delete:", len(blobObj.points), len(blobObj._lines), blobObj.exists, reason)  #example
                    newBlob.uponExist = makeCone
                    newBlob.uponExistAppendTimeout = (60.0/lidarSelf.RPM())*0.5 #IMPORTANT: (seconds per rotation) * half(?), this timout also makes sure blobs are not appended to on subsequent rotations




global sim1
sim1 = pygamesimLocal()

if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
    print("found sys.argv[1], attempting to import:", sys.argv[1])
    sim1.load_map(sys.argv[1], sim1)

timeSinceLastUpdate = sim1.clock()

print("printing serial ports:")
[print(entry.name) for entry in RC.serial.tools.list_ports.comports()]
print("done printing ports.")
print()

try:
    lidar = CX.camsense_X1('/dev/ttyUSB0')
    lidar.postParseCallback = callbackFunc
    
    mapSender = MS.mapTransmitterSocket(host='', port=65432, objectWithMap=sim1)
    mapSender.mapSendInterval = 0.2 #start safe, you can bring this number down if the connection is good (FPS = 1/this)
    threadKeepRunning = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    autoMapSend = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    UIreceive = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    mapSockThread = thr.Thread(target=mapSender.runOnThread, name="mapSockThread", args=(threadKeepRunning, autoMapSend, UIreceive), daemon=True)
    mapSockThread.start()
    ##note: mapSender.manualSendBuffer is a list to which you can append things (any object) to be sent to the connected client (if any)
    
    while threadKeepRunning[0]: #stop evrything if mapSockThread stops
        rightNow = sim1.clock()
        dt = rightNow - timeSinceLastUpdate
        
        LB.checkBlobAge(sim1.clock)
        lidar.run() #needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
        if((sim1.car.pathFolData.auto) if (sim1.pathPlanningPresent and (sim1.car.pathFolData is not None)) else False):
            sim1.calcAutoDriving()
            sim1.car.sendSpeedAngle(sim1.car.desired_velocity, sim1.car.desired_steering) #(spam) send instruction (or simulate doing so)
        sim1.car.getFeedback() #run this to parse serial data (or simulate doing so)
        sim1.car.update(dt)
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        
        FPSrightNow = sim1.clock() #
        if((FPSrightNow-rightNow) < 0.003): #60FPS limiter (optional)
            time.sleep(0.0035-(FPSrightNow-rightNow))
        
# except KeyboardInterrupt:
#     print("main thread keyboard interrupt")
# except Exception as excep:
#     print("main thread exception:", excep)
finally:
    try:  #alternatively:  if(type(sim1.car) is RC.realCar):
        sim1.car.sendSpeedAngle(0.0, 0.0)
    except:
        print("failed to stop car")
    try:
        lidar._serialPort.close()
        print("closed lidar serial port")
    except:
        print("coudln't close lidar serial port")
    try:
        threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
        mapSockThread.join(2)
        print("mapSockThread still alive?:", mapSockThread.is_alive())
    except Exception as excep:
        print("couldn't stop thread?:", excep)
    try:  #alternatively:  if(type(sim1.car) is RC.realCar):
        sim1.car.disconnect()
    except Exception as excep:
        print("failed to run car.disconnect():", excep)
