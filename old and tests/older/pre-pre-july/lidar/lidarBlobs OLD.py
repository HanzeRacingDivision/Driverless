global blobList
blobList = [] #list of blob class objects
#if a new blob is made, the angle of that point is set as the starting angle
#whenever a point is added, new end angle = max(current end angle, new point angle)
#the average point is just the average value of the list of points

maxBlobSize = 33 #points, not angle
maxBlobGapAngle = 5.1 #max degree gap between any 2 points in a blob (should be divided by distance, to make it circumference based, not angle based?)
#maxBlobTotalGapSize = 3.1 #max number of missing points in blob in total
maxBlobAppendDelta = 100 #possibly the most important constant; this determines whether it's 1 object or 2
blobTimeout = 1.5 #time after which any blob is deleted/moved

import time

DEL_OVLP_NEW = 1
DEL_OVLP_BLB = 2
DEL_OLD      = 3

#all of the following funcctions are needed for 1 if-statement in the checkBlobOverlap(), where degRange() is used.
# it's used because rollover is annoying, and the alternative is a list of 4+ "(A and B) or" statements
#some of my rollover-compatible angle macros (NOTE: these are -180 to 180, not 0 to 360)
degRollTwo = lambda angle : ((angle % -360) if (angle < 0) else (angle % 360))
degRollOne = lambda angle : ((angle % -180) if (angle > 180) else ((angle % 180) if (angle < -180) else angle))
degRoll = lambda angle : degRollOne(degRollTwo(angle))
degDiff = lambda angleOne, angleTwo : degRoll(angleTwo-angleOne)  #recommend using abs(degDiff()), but currently, it returns the value required to get from angleOne to angleTwo, so (90, -45) = -135
degMiddOne = lambda lowBound, upBound : (degRoll(lowBound) + (degDiff(lowBound, upBound)/2) + (180 if (degDiff(lowBound, upBound) < 0) else 0))
#degMidd = lambda lowBound, upBound : degRoll(degMiddOne(lowBound, upBound))
simpleRange = lambda angle, lowBound, upBound : ((lowBound <= angle) and (angle <= upBound))
degRangeOne = lambda angle, lowBound, upBound, offset : simpleRange(degRoll(angle-offset), degRoll(lowBound-offset), degRoll(upBound-offset)) #offset values to make lowBound a negative number between (-180, 0) and upBound between (0, 180) and offset input angle the same, to make the check simple
degRange = lambda angle, lowBound, upBound : degRangeOne(degRoll(angle), degRoll(lowBound), degRoll(upBound), degMiddOne(lowBound, upBound))

average = lambda oneDimArray : (sum(oneDimArray) / len(oneDimArray))

class blob:
    """a blob is a collection of adjacent (or nearly adjacent) lidar points
        they must share a similar distance measurement ('.delta').
        the distances are in millimeters, in accordance with the intended lidar (camsense-X1).
        the angles are from 0 to 360, in accordance with the intended lidar (camsense-X1).
        when the blob is fully formed ('exists'), a callback can be called
        another callback can be called just before deletion
        blobs are deleted whenever they overlap, when they're too old,
        or when there is a new blob ABOUT TO BE placed there"""
    def __init__(self, angle, delta, clockFunc=time.time):
        self.startAngle = angle
        self.endAngle = angle
        self.points = [delta]
        self.averagePoint = delta
        self.timestamp = clockFunc()
        self.exists = False #(bad name), set to True when appending fails (same time as oponExist is run (if callable()))
        self.uponExist = None    #a callback for when it fails to append (which means it will never be appended to again (single rotation direction), so it's done at that point). arguments: (self)
        self.uponExistAppendTimeout = -1 #an (optional) timeout, if it hasn't been appended to in this long, run uponExist (if callable()). Set to -1 to disable this timeout
        self.appendTimestamp = clockFunc()
        self.uponDeletion = None #a callback for just before the blob is deleted. arguments: (self, deletionReason)
        self.extraData = None #store any extra data in here (especially useful in the calback functions)
    
    def append(self, angle, delta, clockFunc=time.time):
        """attempt to append a datapoint to the blob, return whether successful
            if it fails to append, the current blob is considered to exist"""
        # if((((max(self.endAngle, endAngle)-self.startAngle) % 360) - len(self.points)) <= maxBlobTotalGapSize): #make sure the new entry doesnt violate the max total gap size
        angleDif = degDiff(self.endAngle, angle)
        distDif = abs(self.averagePoint - delta)
        if(((angleDif > 0) and (angleDif <= maxBlobGapAngle)) and (distDif < maxBlobAppendDelta) and (len(self.points) < maxBlobSize)):
            self.appendTimestamp = clockFunc()
            self.endAngle = angle
            self.points.append(delta)
            self.averagePoint = average(self.points)
            #alternatively, you could set the average value to -1, and calculate it the next time it's needed by some function
            #that would save time here, and cost time later, but it might save time if many points are added quickly
            return(True)
        else:
            #print("cant append blob, too many gaps:", angleDif, distDif)
            self.exists = True
            if(callable(self.uponExist)):
                self.uponExist(self)
            return(False)

def checkBlobOverlap(): #check if existing blobs overlap (and if they do, delete/move the older one)
    """check whether existing blobs overlap, shouldn't happen too often"""
    global blobList
    listLength = len(blobList)
    if(listLength > 1):
        keepList = [True for i in range(listLength)]
        for i in range(listLength):
            for j in range(listLength):
                if((i != j) and keepList[j] and keepList[i]):
                    if(degRange(blobList[i].startAngle, blobList[j].startAngle, blobList[j].endAngle) or degRange(blobList[j].startAngle, blobList[i].startAngle, blobList[i].endAngle)): #this checks for overlap (partial or total/encompassing)
                        if(blobList[i].timestamp < blobList[j].timestamp): #whichever one is older
                            keepList[i] = False
                        else:
                            keepList[j] = False
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

def checkBlobAge(clockFunc=time.time): #check if any blobs are too old to still be considered real (and delete/move them) AND run uponExist, if the blob has a timeout value set for that
    """check whether blobs are too old (and need to be deleted)
        also checks whether blobs 'exist', if they have a timeout set
        should be run as often as possible"""
    global blobList
    listLength = len(blobList)
    keepList = [True for i in range(listLength)]
    rightNow = clockFunc()
    for i in range(listLength):
        if((rightNow - blobList[i].timestamp) > blobTimeout):
            keepList[i] = False
        if(((rightNow - blobList[i].appendTimestamp)>blobList[i].uponExistAppendTimeout) if ((not blobList[i].exists) and (blobList[i].uponExistAppendTimeout > 0)) else False):
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

def checkDeleteBlobs(angle, clockFunc=time.time):
    """run all blob-checks, and deletes any blobs that fall within the entered angle"""
    global blobList
    #if(len(blobList) > 0): #forloop already does this
    checkBlobOverlap()
    checkBlobAge(clockFunc)
    for blob in blobList:
        if(degRange(angle, blob.startAngle, blob.endAngle)): #if angle falls within an existing blob
            ## now either delete the (old) blob, or move it to some other list (or neither, in which case checkBlobOverlap() will do so later)
            if(callable(blob.uponDeletion)):
                blob.uponDeletion(blob, DEL_OVLP_NEW)
            blobList.remove(blob)
            ## now, because checkBlobOverlap() has been run, we can stop searching here and just return()
            ## (because if no blobs overlap, then 'angle' will not fall within any other blobs)
            #print("angle fell within an existing blob")
            return(True)
    return(False)


def blobify(angle, delta, clockFunc=time.time):
    """deletes-, appends to- or creates blobs given a lidar measurement (angle, distance)"""
    global blobList
    checkDeleteBlobs(angle, clockFunc)
    if(len(blobList) > 0):
        lastBlob = blobList[-1] #recall last blob
        if(not lastBlob.append(angle, delta, clockFunc)): #try to add point to blob
            blobList.append(blob(angle, delta, clockFunc)) #add new blob to list
            return(True, blobList[-1])
        else:
            return(False, lastBlob)
    else:
        blobList.append(blob(angle, delta, clockFunc)) #add to total list
        return(True, blobList[-1])
        


if __name__ == '__main__':
    blobify(355, 100)
    blobify(357, 105)
    blobify(359, 120)
    blobify(1, 125)
    blobify(3, 130)
    
    [print(item.points) for item in blobList]