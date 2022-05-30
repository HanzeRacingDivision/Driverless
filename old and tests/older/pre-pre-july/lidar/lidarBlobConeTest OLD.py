import camsense_X1 as CX
import lidarBlobs as LB
import time
import numpy as np

import generalFunctions as GF
from Map import Map
import coneConnecting as CC
#import pathFinding    as PF
#import pathPlanningTemp as PP
#import simulatedCar   as SC
import drawDriverless as DD


class pygamesimLocal(CC.coneConnecter, DD.pygameDrawer):
    def __init__(self, window, drawSize=(700,350), drawOffset=(0,0), carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        Map.__init__(self) #init map class
        
        # self.clockSet(simClock) #an altered clock, only for simulations where the speed is faster/slower than normal
        # self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
        CC.coneConnecter.__init__(self)
        # PF.pathFinder.__init__(self)
        # PP.pathPlanner.__init__(self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = False
        self.pathFinderPresent = False
        self.pathPlanningPresent = False
        self.SLAMPresent = False
        
        self.isRemote = False #tell the drawing class to apply UI elements locally
        
        self.carPolygonMode = True #if you dont want to use the car sprite, set this to true (but if the sprite wasnt loaded this will be used automatically)
        
        # if(self.pathPlanningPresent):
        #     self.car.pathFolData = PP.pathPlannerData()



global debugThresh
debugThresh = 750

# def delCone(blob, reason):
#     if(reason == LB.DEL_OLD):
#         global sim1
#         posToDelete = blob.extraData.position
#         overlaps, overlappingCone = sim1.overlapConeCheck(posToDelete)
#         if(overlaps):
#             print("deleting:", overlappingCone.ID, round((sim1.clock()-blob.timestamp)*1000,1))
#             for connectedCone in overlappingCone.connections:
#                 if(len(connectedCone.coneConData) > 0): #it's always a list, but an empty one if coneConnecter is not used
#                     connectedCone.coneConData.pop((0 if (connectedCone.connections[0].ID == overlappingCone.ID) else 1))
#                 connectedCone.connections.pop((0 if (connectedCone.connections[0].ID == overlappingCone.ID) else 1))
#             listToRemoveFrom = (sim1.right_cone_list if overlappingCone.LorR else sim1.left_cone_list)
#             listToRemoveFrom.pop(GF.findIndexByClassAttr(listToRemoveFrom, 'ID', overlappingCone.ID))
#         else:
#             print("HELP!, standed cone?:", [round(posToDelete[0],1), round(posToDelete[1],1)], round((sim1.clock()-blob.timestamp)*1000,1))
#         overlaps, overlappingCone = sim1.overlapConeCheck(posToDelete)
#         if(overlaps):
#             print("double overlap!")
#         print("len", len(sim1.left_cone_list))
#     elif(reason == LB.DEL_OVLP_BLB):
#         print("DEL_OVLP_BLB", len(sim1.left_cone_list), round((sim1.clock()-blob.timestamp)*1000,1))
#     # else:
#     #     print("else", reason, "ID", blob.extraData.ID, "len", len(sim1.left_cone_list))

def makeCone(blob):
    global sim1
    correctedLidarAngle = (-1)* GF.radInv(np.deg2rad(GF.degRoll(blob.startAngle+(GF.degDiff(blob.startAngle, blob.endAngle)/2))))
    ## correctedLidarAngle:  get 'middle' angle, convert to (-180,180) with derRoll, convert to radians, set 0-angle at car-rear (instead of front) with radInv and *-1 because lidar degrees are CW and Map rotation is CCW
    posToPlace = GF.distAnglePosToPos((blob.averagePoint/1000)+(Map.Cone.coneDiam/2), sim1.car.angle + correctedLidarAngle, sim1.car.position) #get map position from relative position (asuming lidar is mounter on car)
    overlaps, overlappingCone = sim1.overlapConeCheck(posToPlace)
    if(overlaps):
        overlappingCone.position[0] = posToPlace[0]
        overlappingCone.position[1] = posToPlace[1]
        blob.extraData = overlappingCone
    else:
        newConeID = GF.findMaxAttrIndex((sim1.right_cone_list + sim1.left_cone_list), 'ID')[1]
        aNewCone = Map.Cone(newConeID+1, posToPlace, False, False)
        sim1.left_cone_list.append(aNewCone)
        blob.extraData = aNewCone
    #blob.uponDeletion = delCone
    #blob.uponExist = None #this is just to mark that the function has been called (alternatively, check extraData)

def callbackFunc(lidarSelf, newPacket, pointsAdded):
    if((newPacket is not None) and (pointsAdded > 0)):
        startAngle = newPacket.startAngle/CX.DEG_DIV
        angleDif = LB.degDiff(newPacket.startAngle/CX.DEG_DIV, newPacket.endAngle/CX.DEG_DIV) #(rollover proof) IMPORTANT: packet angles are still integers, divide by 64 to get degrees (0 to 360)
        angleStep = angleDif/7 #note: there are actually 8 datapointes per packet, but i'm assuming the endAngle is AT the 8th datapoint (and the startAngle is AT the first)
        global debugThresh, sim1
        #LB.checkBlobAge(sim1.clock)
        #LB.checkBlobOverlap()
        for i in range(len(newPacket.measurements)-pointsAdded, len(newPacket.measurements)): #if it's not a new packet, but an existing one with new points, this will skip the old ones
            angle = startAngle+angleStep*i
            if((newPacket.measurements[i] < debugThresh) and (newPacket.measurements[i] != CX.NO_MEASUREMENT)):
                isNewBlob, newBlob = LB.blobify(angle, newPacket.measurements[i], sim1.clock)
                if(isNewBlob):
                    #newBlob.uponExist = lambda blobObj : print("blob:", len(blobObj.points), blobObj.startAngle, blobObj.averagePoint)
                    newBlob.uponExist = makeCone
                    newBlob.uponExistAppendTimeout = (60.0/lidarSelf.RPM())*0.5 #(seconds per rotation) * half(?)



resolution = [1200, 600]

DD.pygameInit(resolution)
global sim1
sim1 = pygamesimLocal(DD.window, resolution)

timeSinceLastUpdate = sim1.clock()

try:
    lidar = CX.camsense_X1('COM19')
    lidar.postParseCallback = callbackFunc
    
    while DD.windowKeepRunning:
        rightNow = sim1.clock()
        dt = rightNow - timeSinceLastUpdate
        DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        
        LB.checkBlobAge(sim1.clock)
        LB.checkBlobOverlap()
        lidar.run() #needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
        # if((sim1.car.pathFolData.auto) if (sim1.pathPlanningPresent and (sim1.car.pathFolData is not None)) else False):
        #     sim1.calcAutoDriving()
        #     sim1.car.sendSpeedAngle(sim1.car.desired_velocity, sim1.car.desired_steering) #(spam) send instruction (or simulate doing so)
        # sim1.car.getFeedback() #run this to parse serial data (or simulate doing so)
        sim1.car.update(dt)
        
        sim1.redraw()
        DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        
# except KeyboardInterrupt:
#     print("main thread keyboard interrupt")
# except Exception as excep:
#     print("main thread exception:", excep)
finally:
    try:
        lidar._serialPort.close()
        print("closed lidar serial port")
    except:
        print("coudln't close lidar serial port")
    DD.pygameEnd() #correctly shut down pygame window
