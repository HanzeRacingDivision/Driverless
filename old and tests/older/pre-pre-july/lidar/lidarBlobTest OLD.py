import camsense_X1 as CX
import lidarBlobs as LB
import time
import threading as thr

global debugThresh
debugThresh = 750
# global pointList, pointListSemphr, lastRotationCount
# pointList = [[],[]]
# pointListSemphr = thr.Semaphore()
# lastRotationCount = 0

def callbackFunc(lidarSelf, newPacket, pointsAdded):
    if((newPacket is not None) and (pointsAdded > 0)):
        startAngle = newPacket.startAngle/CX.DEG_DIV
        angleDif = LB.degDiff(newPacket.startAngle/CX.DEG_DIV, newPacket.endAngle/CX.DEG_DIV) #(rollover proof) IMPORTANT: packet angles are still integers, divide by 64 to get degrees (0 to 360)
        angleStep = angleDif/7 #note: there are actually 8 datapointes per packet, but i'm assuming the endAngle is AT the 8th datapoint (and the startAngle is AT the first)
        global debugThresh
        # global pointList, pointListSemphr, lastRotationCount
        # if(lidarSelf.rotationCount > lastRotationCount):
        #     lastRotationCount = lidarSelf.rotationCount
        #     with pointListSemphr:
        #         pointList[0] = [item for item in pointList[1]]
        #         pointList[1] = []
        LB.checkBlobAge()
        for i in range(len(newPacket.measurements)-pointsAdded, len(newPacket.measurements)): #if it's not a new packet, but an existing one with new points, this will skip the old ones
            angle = startAngle+angleStep*i
            #with pointListSemphr: #can't be bothered
            # if(newPacket.measurements[i] != camsense_X1.NO_MEASUREMENT):
            #     pointList[1].append([angle, newPacket.measurements[i]])
            if((newPacket.measurements[i] < debugThresh) and (newPacket.measurements[i] != CX.NO_MEASUREMENT)):
                isNewBlob, newBlob = LB.blobify(angle, newPacket.measurements[i])
                if(isNewBlob):
                    newBlob.uponExist = lambda blobObj : print("blob:", len(blobObj.points), blobObj.startAngle, blobObj.averagePoint)


printTimer = time.time()
speedTimer = []
try:
    lidar = CX.camsense_X1('COM19')
    lidar.postParseCallback = callbackFunc
    
    while(True):
        lidar.run() #needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
        
        rightNow = time.time()
        if((rightNow - printTimer) > 1.0):
            printTimer = rightNow
            
            #print(lidar.spinning, lidar.rotationCount, len(lidar.lidarData))
            # with pointListSemphr:
            #     distList = [item[1] for item in pointList[0]]
            #     distList.sort()
            #     print(distList)
            
            #blobPoints = [print(len(item.points), round(item.averagePoint,1)) for item in LB.blobList]
            #print()
            #print(blobPoints)

finally: #required for correct serial port closing (otherwise, you need to unplug and replug device every time)
    try:
        lidar._serialPort.close()
        print("closed lidar serial port")
    except:
        print("coudln't close lidar serial port")