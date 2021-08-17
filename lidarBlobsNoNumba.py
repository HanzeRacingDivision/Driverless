from Map import Map  #used to get coneDiam
import GF.generalFunctions as GF
import numpy as np


global blobInProgress
blobInProgress = None
#if a new blob is made, the angle of that point is set as the starting angle
#whenever a point is added, new end angle = max(current end angle, new point angle)
#the average point is just the average value of the list of points

adjustedConeDiam = Map.Cone.coneDiam * 0.5

maxBlobPointCount = 16 #points, not angle
#maxBlobAverageGapSizeSqrd = 0.075**2 #if the average distance between points is larger than this, only applied if pointCount >= maxBlobAverageGapPoints
#maxBlobAverageGapPoints = 3 #number of points in blob required to activate maxBlobAverageGapSizeSqrd check
maxBlobSingleGap = adjustedConeDiam #if the distance between the new datapoint and the last blob-point is larger than this, don't append, (make a new blob)
## maxBlobSingleGap must equal adjustedConeDiam, because otherwise the np.arcsin() in blobToConePos() will fail to compute.

MIN_BLOB_CONE_LEN = 3 #used for blobToConePos

#import time

blobType =np.dtype([('timestamp', np.float64),
                    ('appendTimestamp', np.float64),
                    ('points', np.float64, (maxBlobPointCount,2)),
                    ('origins', np.float64, (maxBlobPointCount,2)),
                    ('lines', np.float64, (maxBlobPointCount-1,2)), #stores [dist, angle]
                    ('pointCount', np.uint8)])

def blobCreate(point, origin, timestamp):
    newBlob = np.zeros(1, dtype=blobType)[0] #create a new blob object (filled with all 0's)
    newBlob['timestamp'] = timestamp
    newBlob['appendTimestamp'] = timestamp
    newBlob['points'][0] = point
    newBlob['origins'][0] = origin
    newBlob['pointCount'] = 1
    return(newBlob)

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

def checkBlobAge(timestamp, uponExist=None, uponExistArgs=None, uponExistAppendTimeout=-1): #check if any blobs are too old to still be considered real (and delete/move them) AND run uponExist, if the blob has a timeout value set for that
    """check whether blobs are too old (and need to be deleted)
        also checks whether blobs 'exist', if they have a timeout set
        should be run as often as possible"""
    global blobInProgress
    if(blobInProgress is not None):
        if(((timestamp - blobInProgress['appendTimestamp'])>uponExistAppendTimeout) if (uponExistAppendTimeout > 0) else False):
            if(callable(uponExist)):
                uponExist(blobInProgress, uponExistArgs)
            blobInProgress = None

def blobify(point, origin, timestamp, uponExist=None, uponExistArgs=None):
    """deletes-, appends to- or creates blobs given a lidar measurement (position, origin(optional))"""
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

def blobToConePos(blob): #calculate the position the cone would have over here, to save some processing time on the main thread
    if(blob['pointCount'] < MIN_BLOB_CONE_LEN):
        print("warning: blob too few points to make into cone")
        return(False, np.zeros(2, dtype=np.float64))
    coneCenterPos = np.empty((2,blob['pointCount']-1), dtype=np.float64) #TBD
    if(blob['pointCount'] > 1):
        perpAdd = np.pi/2 # we can assume (or calculate only once) the direction of the perpendicular angle, by knowing the rotation direction of the lidar
        # baseAngle = GF.get_norm_angle_between(blob['origins'][int(blob['pointCount']/2)], blob['points'][int(blob['pointCount']/2)], 0.0)
        # if(abs(GF.radDiff(blob['lines'][0][1]-perpAdd, baseAngle)) < abs(GF.radDiff(blob['lines'][0][1]+perpAdd, baseAngle))):
        #     perpAdd = -perpAdd
        #     print("warning: lidar measurement not simply CW?")
        for i in range(blob['pointCount']-1):
            superAdjustedConeRadius = np.cos(np.arcsin((blob['lines'][i][0]/2) / (adjustedConeDiam/2))) * (adjustedConeDiam/2)
            perpAngle = blob['lines'][i][1] + perpAdd
            lineCenter = GF.distAnglePosToPos(blob['lines'][i][0]/2, blob['lines'][i][1], blob['points'][i])
            conePos = GF.distAnglePosToPos(superAdjustedConeRadius, perpAngle, lineCenter)
            coneCenterPos[0][i] = conePos[0];   coneCenterPos[1][i] = conePos[1]
        conePos = np.array([GF.average(coneCenterPos[0]), GF.average(coneCenterPos[1])], dtype=np.float64)
        return(True, conePos)
    else:
        conePos = GF.distAnglePosToPos(adjustedConeDiam, GF.get_norm_angle_between(blob['origins'][0], blob['points'][0], 0.0), blob['points'][0])
        return(True, conePos)
