import GF.generalFunctions as GF
import numpy as np


#this version is called "...NoList" because i realized that i didnt really use the blobList in the fastest applications (which i'm targeting).
# this version just forgets about the existance of all previous blobs, relying fully on the use of the .uponExist callback function to grab the fully formed blobs
# the .uponDelete callback has been removed, as it would be called right after .uponExist anyway

global blobInProgress
blobInProgress = None
#if a new blob is made, the angle of that point is set as the starting angle
#whenever a point is added, new end angle = max(current end angle, new point angle)
#the average point is just the average value of the list of points

maxBlobPointCount = 33 #points, not angle
## the unit for points needs to match with these values!, currently in: meters
maxBlobAverageGapSizeSqrd = 0.075**2 #if the average distance between points is larger than this, only applied if pointCount >= maxBlobAverageGapPoints
maxBlobAverageGapPoints = 3 #number of points in blob required to activate maxBlobAverageGapSizeSqrd check
maxBlobSingleGapSqrd = 0.1**2 #if the distance between the new datapoint and the last blob-point is larger than this, don't append, (make a new blob)

import time

class blob:
    """a blob is a collection of adjacent (or nearly adjacent) lidar points
        when the blob is fully formed ('exists'), a callback can be called"""
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
        self.extraData = None #store any extra data in here (especially useful in the calback functions)
    
    def __repr__(self):
        return("blob(p:"+str(len(self.points))+\
               ",o:"+str(len(self.points))+\
               ",e?:"+str(self.exists)+\
               ((",UpEx:"+self.uponExist.__name__) if callable(self.uponExist) else "")+\
               ((",exDat:"+str(self.extraData)) if (self.extraData is not None) else "")+")")
    
    def append(self, point, origin=None, timestamp=None):
        """attempt to append a datapoint to the blob, return whether successful
            if it fails to append, the current blob is considered to exist"""
        gapSizeSqrd = GF.distSqrdBetwPos(point, self.points[-1]) #using squared values means you can skip doing a (slow) square-root operation (or using sin()/cos()/tan() of course)
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
            print("warning: appending to an blob that 'exists' failed") #i dont think this should ever happen; so if i do see this, something may be very wrong
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

def checkBlobAge(timestamp=None): #check if any blobs are too old to still be considered real (and delete/move them) AND run uponExist, if the blob has a timeout value set for that
    """check whether blobs are too old (and need to be deleted)
        also checks whether blobs 'exist', if they have a timeout set
        should be run as often as possible"""
    global blobInProgress
    if(timestamp is None):
        timestamp = time.time()
    if(blobInProgress is not None):
        if(((timestamp - blobInProgress.appendTimestamp)>blobInProgress.uponExistAppendTimeout) if ((not blobInProgress.exists) and (blobInProgress.uponExistAppendTimeout > 0)) else False):
            blobInProgress.exists = True
            if(callable(blobInProgress.uponExist)):
                blobInProgress.uponExist(blobInProgress)
            blobInProgress = None
        # elif((timestamp - blobInProgress.timestamp) > blobTimeout):
        #     blobInProgress = None

def blobify(point, origin=None, timestamp=None):
    """deletes-, appends to- or creates blobs given a lidar measurement (position, origin(optional))"""
    global blobInProgress
    #checkDeleteBlobs(point, timestamp)
    if(timestamp is None):
        timestamp = time.time()
    makeNewBlob = True
    if(blobInProgress is not None):
        makeNewBlob = not blobInProgress.append(point, origin, timestamp) #try to add point to blob in progress
    if(makeNewBlob):
        blobInProgress = blob(point, origin, timestamp)
    return(makeNewBlob, blobInProgress)

