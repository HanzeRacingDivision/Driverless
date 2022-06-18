from Map import Map  #used to get coneDiam
import GF.generalFunctions as GF
import numpy as np

#from numba import njit


global blobInProgress
blobInProgress = None
#if a new blob is made, the angle of that point is set as the starting angle
#whenever a point is added, new end angle = max(current end angle, new point angle)
#the average point is just the average value of the list of points

adjustedConeDiam = Map.Cone.coneLidarDiam

maxBlobPointCount = 12 #points, not angle
#maxBlobAverageGapSizeSqrd = 0.075**2 #if the average distance between points is larger than this, only applied if pointCount >= maxBlobAverageGapPoints
#maxBlobAverageGapPoints = 3 #number of points in blob required to activate maxBlobAverageGapSizeSqrd check
maxBlobSingleGap = adjustedConeDiam * 0.5 #if the distance between the new datapoint and the last blob-point is larger than this, don't append, (make a new blob)
## maxBlobSingleGap must equal adjustedConeDiam, because otherwise the np.arcsin() in blobToConePos() will fail to compute.

MIN_BLOB_CONE_LEN = 3 #used for blobToConePos

#import time

## a numpy dtype is fast/efficient, but not really user-fiendly (not very python-like). Good thing i don't like legible code anyways ;)
blobType =np.dtype([('timestamp', np.float64),
                    ('appendTimestamp', np.float64),
                    ('points', np.float64, (maxBlobPointCount,2)),
                    ('origins', np.float64, (maxBlobPointCount,2)),
                    ('lines', np.float64, (maxBlobPointCount-1,2)), #stores [dist, angle]
                    ('pointCount', np.uint8)])

#@njit
def blobCreate(point, origin, timestamp):
    newBlob = np.zeros(1, dtype=blobType)[0] #create a new blob object (filled with all 0's)
    newBlob['timestamp'] = timestamp
    newBlob['appendTimestamp'] = timestamp
    newBlob['points'][0] = point
    newBlob['origins'][0] = origin
    newBlob['pointCount'] = 1
    return(newBlob)

#@njit
def blobAppend(blob, point, origin, timestamp):
    """attempt to append a datapoint to the blob, return whether successful"""
    distAngle = GF.distAngleBetwPos(blob['points'][blob['pointCount']-1], point) #get gap size (and angle, while you're at it)
    if((distAngle[0] < maxBlobSingleGap)
       #and ((GF.average(np.array([entry[0] for entry in self._lines]+[gapSizeSqrd])) < maxBlobAverageGapSizeSqrd) if (len(self.points)>=maxBlobAverageGapPoints) else True)
       and (blob['pointCount'] < maxBlobPointCount)):
        
        blob['appendTimestamp'] = timestamp
        blob['points'][blob['pointCount']] = point
        blob['origins'][blob['pointCount']] = origin
        blob['lines'][blob['pointCount']-1] = distAngle
        blob['pointCount'] += 1
        return(True)
    else:
        return(False)

def checkBlobAge(timestamp, uponExist=None, uponExistArgs=None, uponExistAppendTimeout=-1):
    """if blobs were last appended to more than 'uponExistAppendTimeout' seconds ago, call the 'uponExist' callback"""
    global blobInProgress
    if(blobInProgress is not None):
        if(((timestamp - blobInProgress['appendTimestamp'])>uponExistAppendTimeout) if (uponExistAppendTimeout > 0) else False):
            if(callable(uponExist)):
                uponExist(blobInProgress, uponExistArgs)
            blobInProgress = None

def blobify(point, origin, timestamp, uponExist=None, uponExistArgs=None, uponExistAppendTimeout=-1):
    """deletes-, appends to- or creates blobs given a lidar measurement"""
    if(uponExistAppendTimeout > 0):
        checkBlobAge(timestamp, uponExist, uponExistArgs, uponExistAppendTimeout)
    global blobInProgress
    #checkDeleteBlobs(point, timestamp)
    makeNewBlob = True
    if(blobInProgress is not None):
        makeNewBlob = not blobAppend(blobInProgress, point, origin, timestamp) #try to add point to blob in progress
        if(makeNewBlob):
            if(callable(uponExist)):
                uponExist(blobInProgress, uponExistArgs)
    if(makeNewBlob):
        blobInProgress = blobCreate(point, origin, timestamp)
    return(makeNewBlob, blobInProgress)

#@njit
def blobToConePos(blob): #calculate the position the cone would have over here, to save some processing time on the main thread
    if(blob['pointCount'] < MIN_BLOB_CONE_LEN):
        #print("warning: blob too few points to make into cone")
        return(False, np.zeros(2, dtype=np.float64))
    coneCenterPos = np.empty((2,blob['pointCount']-1), dtype=np.float64) #TBD
    if(blob['pointCount'] > 1):
        perpAdd = np.pi/2 # we can assume (or calculate only once) the direction of the perpendicular angle, by knowing the rotation direction of the lidar
        # baseAngle = GF.get_norm_angle_between(blob['origins'][int(blob['pointCount']/2)], blob['points'][int(blob['pointCount']/2)], 0.0)
        # if(abs(GF.radDiff(blob['lines'][0][1]-perpAdd, baseAngle)) < abs(GF.radDiff(blob['lines'][0][1]+perpAdd, baseAngle))):
        #     perpAdd = -perpAdd
        #     print("warning: lidar measurement not simply CW?")
        for i in range(blob['pointCount']-1):
            ## the indirect way:
            # superAdjustedConeRadius = np.cos(np.arcsin((blob['lines'][i][0]/2) / (adjustedConeDiam/2))) * (adjustedConeDiam/2)
            # lineCenter = GF.distAnglePosToPos(blob['lines'][i][0]/2, blob['lines'][i][1], blob['points'][i])
            # conePos = GF.distAnglePosToPos(superAdjustedConeRadius, blob['lines'][i][1] + perpAdd, lineCenter)
            ## the direct way:
            conePos = GF.distAnglePosToPos((adjustedConeDiam/2), blob['lines'][i][1] + perpAdd - np.arcsin(blob['lines'][i][0] / adjustedConeDiam), blob['points'][i]) # alt version (untested)
            coneCenterPos[0][i] = conePos[0];   coneCenterPos[1][i] = conePos[1]
        conePos = np.array([GF.average(coneCenterPos[0]), GF.average(coneCenterPos[1])], dtype=np.float64)
        return(True, conePos)
    else:
        conePos = GF.distAnglePosToPos(adjustedConeDiam, GF.get_norm_angle_between(blob['origins'][0], blob['points'][0], 0.0), blob['points'][0])
        return(True, conePos)

# def compileAll(verbose=False):
#     ## precompile things by running them once
#     print("precompiling njit lidarBlobs...")
#     import time
#     compileStartTime = time.time()
    
#     aMap = Map()
#     aPoint = np.array([10.0, 10.0], dtype=np.float64)
#     #uponExistCallback = lambda blob, extraArgs : print(blobToConePos(blob), extraArgs)
#     blobify(aPoint, aMap.car.position, aMap.clock()) #runs blobCreate
#     aPoint[0] += 0.025;  aPoint[1] -= 0.045;
#     blobToConePos(blobify(aPoint, aMap.car.position, aMap.clock())[1])  #runs blobAppend and then blobToConePos
#     del(aMap);   global blobInProgress; blobInProgress = None
#     print("lidarBlobs njit compilation done! (took", round(time.time()-compileStartTime,1), "seconds)")
#     if(verbose):
#         def inspect(compiledFunction):
#             print(compiledFunction.__name__, compiledFunction.signatures, type(compiledFunction.signatures[0][0]))
#             compiledFunction.inspect_types(file)
#         ## verbose output of njit functions (to file)
#         print("verbose output of generalFunctions njit compilation:")
#         file = open("lidarBlobs njit verbose output.txt", "w+")
#         inspect(blobCreate)
#         inspect(blobAppend)
#         inspect(blobToConePos)
#         file.close()

# try:
#     compileAll()
# except:
#     print("precompiling njit lidarBlobs failed (is generalFunctions not compiled?)")
#     print("falling back to lidarBlobsNoNumba")
#     del(blobCreate);  del(blobAppend);  del(blobToConePos);  del(compileAll)
#     from lidarBlobsNoNumba import *


#if __name__ == "__main__":
    