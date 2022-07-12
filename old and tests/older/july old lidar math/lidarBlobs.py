import GF.generalFunctions as GF
import numpy as np

global blobList
blobList = [] #list of blob class objects
#if a new blob is made, the angle of that point is set as the starting angle
#whenever a point is added, new end angle = max(current end angle, new point angle)
#the average point is just the average value of the list of points

maxBlobPointCount = 33 #points, not angle
## the unit for points needs to match with these values!, currently in: meters
maxBlobAverageGapSizeSqrd = 0.075**2 #if the average distance between points is larger than this, only applied if pointCount >= maxBlobAverageGapPoints
maxBlobAverageGapPoints = 3 #number of points in blob required to activate maxBlobAverageGapSizeSqrd check
maxBlobSingleGapSqrd = 0.1**2 #if the distance between the new datapoint and the last blob-point is larger than this, don't append, (make a new blob)
blobOverlapThreshSqrd = 0.1**2 #if the distance between any of the points of any 2 blobs are less than this, consider it overlapping and delete the older one
blobTimeout = 1.5 #time after which any blob is deleted/moved

import time

DEL_OVLP_NEW = 1
DEL_OVLP_BLB = 2
DEL_OLD      = 3


class blob:
    """a blob is a collection of adjacent (or nearly adjacent) lidar points
        when the blob is fully formed ('exists'), a callback can be called
        another callback can be called just before deletion
        blobs are deleted whenever they overlap, when they're too old,
        or when there is a new blob ABOUT TO BE placed there"""
    def __init__(self, point, origin=None, timestamp=None):
        if(timestamp is None):
            timestamp = time.time()
        self.points = [np.array(point)] #datapoints, locations that the lidar detected
        self.origins = [] #optional parameter, location(s) that the LIDAR itself had when it measured the points
        if(origin is not None): #only if parameter was filled in
            self.origins.append(np.array(origin))
        self._lines = [] #lines between the points, consists of [[distSqrd,angle,dist], ...] where angle and dist are optional, the length of this list is len(points)-1. use lineData(index) to retrieve data
        self.timestamp = timestamp
        self.exists = False #(bad name), set to True when appending fails (same time as oponExist is run (if callable()))
        self.uponExist = None    #a callback for when it fails to append (which means it will never be appended to again (single rotation direction), so it's done at that point). arguments: (self)
        self.uponExistAppendTimeout = -1 #an (optional) timeout, if it hasn't been appended to in this long, run uponExist (if callable()). Set to -1 to disable this timeout
        self.appendTimestamp = self.timestamp
        self.uponDeletion = None #a callback for just before the blob is deleted. arguments: (self, deletionReason)
        self.extraData = None #store any extra data in here (especially useful in the calback functions)
    
    def __repr__(self):
        return("blob(p:"+str(len(self.points))+\
               ",o:"+str(len(self.points))+\
               ",e?:"+str(self.exists)+\
               ((",UpEx:"+self.uponExist.__name__) if callable(self.uponExist) else "")+\
               ((",UpDel:"+self.uponDeletion.__name__) if callable(self.uponDeletion) else "")+\
               ((",exDat:"+str(self.extraData)) if (self.extraData is not None) else "")+")")
    
    def append(self, point, origin=None, timestamp=None):
        """attempt to append a datapoint to the blob, return whether successful
            if it fails to append, the current blob is considered to exist"""
        gapSizeSqrd = GF.distSqrdBetwPos(point, self.points[-1])
        if((gapSizeSqrd < maxBlobSingleGapSqrd) and ((GF.average(np.array([entry[0] for entry in self._lines]+[gapSizeSqrd])) < maxBlobAverageGapSizeSqrd) if (len(self.points)>=maxBlobAverageGapPoints) else True) and (len(self.points) < maxBlobPointCount) and (not self.exists)):
            if(timestamp is None):
                timestamp = time.time()
            self.appendTimestamp = timestamp
            self.points.append(np.array(point))
            if(origin is not None): # and (len(self.origins)>0)):
                self.origins.append(np.array(origin))
            self._lines.append([gapSizeSqrd, None, None]) #don't calculate angle and (non-sqrd) dist only once they're required
            #alternatively, you could set the average value to -1, and calculate it the next time it's needed by some function
            #that would save time here, and cost time later, but it might save time if many points are added quickly
            return(True)
        elif(not self.exists):
            self.exists = True
            if(callable(self.uponExist)):
                self.uponExist(self)
            return(False)
        else:
            return(False)
    
    def lineData(self, whichLine: int, calcAngle=True, calcDist=True):
        """use this instead of grabbing data from the ._lines list!
            this is to save calculations (and therefore general resources)
            specify whether you need the angle and/or distance"""
        #if((whichLine >= 0) and (whichLine < len(self._lines))): #extra safety check
        if((self._lines[whichLine][1] is None) and calcAngle):
            if((self._lines[whichLine][2] is None) and calcDist):
                self._lines[whichLine][2], self._lines[whichLine][1] = GF.distAngleBetwPos(self.points[whichLine], self.points[whichLine+1])
            else:
                self._lines[whichLine][1] = GF.get_norm_angle_between(self.points[whichLine], self.points[whichLine+1], 0.0)
        elif((self._lines[whichLine][2] is None) and calcDist):
            self._lines[whichLine][2] = GF.distAngleBetwPos(self.points[whichLine], self.points[whichLine+1])[0]
            #self._lines[whichLine][2] = self._lines[whichLine]**0.5 #calculating square roots is inefficient
        return(self._lines[whichLine])

def checkBlobOverlap(): #check if existing blobs overlap (and if they do, delete/move the older one)
    """check whether existing blobs overlap, shouldn't happen too often
        should only be run when blobs are appended/created, because it is inefficient
        (even then, running it every few changes (upon 'exist' calls?) instead of every time wouldn't hurt that much)"""
    global blobList
    listLength = len(blobList)
    if(listLength > 1):
        keepList = [True for i in range(listLength)]
        for i in range(listLength):
            for j in range(listLength):
                if((i != j) and keepList[i] and keepList[j]):
                    for Ipoint in blobList[i].points:     #this is what makes this function so inefficient,
                        for Jpoint in blobList[j].points: #running it for every point in every blob means doing lots and lots of loops
                            if(GF.distSqrdBetwPos(Ipoint, Jpoint) < blobOverlapThreshSqrd): #if any 2 points are close enough to satisfy the overlap threshold
                                if(blobList[i].timestamp < blobList[j].timestamp): #whichever one is older
                                    keepList[i] = False
                                else:
                                    keepList[j] = False
                                break #break out of Jpoint loop
                        if(not (keepList[i] and keepList[j])): #if either blob was marked for deletion
                            break #break out of Ipoint loop
        #now start deleting entries (or move them to some other list, i guess)
        popCount = 0
        for i in range(listLength):
            if(not keepList[i]):
                if(callable(blobList[i-popCount].uponDeletion)):
                    blobList[i-popCount].uponDeletion(blobList[i-popCount], DEL_OVLP_BLB)
                blobList.pop(i-popCount)
                popCount += 1
        # if(popCount > 0):
        #     print("deleted", popCount, "blobs for overlapping")

def checkBlobAge(timestamp=None): #check if any blobs are too old to still be considered real (and delete/move them) AND run uponExist, if the blob has a timeout value set for that
    """check whether blobs are too old (and need to be deleted)
        also checks whether blobs 'exist', if they have a timeout set
        should be run as often as possible"""
    global blobList
    listLength = len(blobList)
    keepList = [True for i in range(listLength)]
    if(timestamp is None):
        timestamp = time.time()
    for i in range(listLength):
        if((timestamp - blobList[i].timestamp) > blobTimeout):
            keepList[i] = False
        if(((timestamp - blobList[i].appendTimestamp)>blobList[i].uponExistAppendTimeout) if ((not blobList[i].exists) and (blobList[i].uponExistAppendTimeout > 0)) else False):
            blobList[i].exists = True
            if(callable(blobList[i].uponExist)):
                blobList[i].uponExist(blobList[i])
    #now start deleting entries (or move them to some other list, i guess)
    popCount = 0
    for i in range(listLength):
        if(not keepList[i]):
            if(callable(blobList[i-popCount].uponDeletion)):
                blobList[i-popCount].uponDeletion(blobList[i-popCount], DEL_OLD)
            blobList.pop(i-popCount)
            popCount += 1
    # if(popCount > 0):
    #     print("deleted", popCount, "blobs for being too old")

# def checkDeleteBlobs(point, clockFunc=time.time):
#     """run all blob-checks, and deletes any blobs that fall within the entered angle"""
#     global blobList
#     #if(len(blobList) > 0): #forloop already does this
#     checkBlobOverlap()
#     checkBlobAge(clockFunc)
#     for blob in blobList:
#         if(GF.degRange(angle, blob.startAngle, blob.endAngle) and blob.exists): #if angle falls within an existing blob
#             ## now either delete the (old) blob, or move it to some other list (or neither, in which case checkBlobOverlap() will do so later)
#             if(callable(blob.uponDeletion)):
#                 blob.uponDeletion(blob, DEL_OVLP_NEW)
#             blobList.remove(blob)
#             ## now, because checkBlobOverlap() has been run, we can stop searching here and just return()
#             ## (because if no blobs overlap, then 'angle' will not fall within any other blobs)
#             #print("angle fell within an existing blob")
#             return(True)
#     return(False)


def blobify(point, origin=None, timestamp=None):
    """deletes-, appends to- or creates blobs given a lidar measurement (position, origin(optional))"""
    global blobList
    #checkDeleteBlobs(point, timestamp)
    if(timestamp is None):
        timestamp = time.time()
    makeNewBlob = True
    if(len(blobList) > 0):
        makeNewBlob = not blobList[-1].append(point, origin, timestamp) #try to add point to latest blob
    if(makeNewBlob): #if the list is empty, or appending failed
        blobList.append(blob(point, origin, timestamp)) #add to total list
    checkBlobOverlap()
    checkBlobAge(timestamp)
    return(makeNewBlob, blobList[-1])
