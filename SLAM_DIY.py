import numpy as np

import GF.generalFunctions as GF
import multilateration as multLat

MIN_LANDMARK_COUNT = 4
MAX_SLAM_CORRECTION = 1.0 #meters of distance
MAX_BLOB_HISTORY = 10


## TBD: improve error magnitudes:
##                  - based on time since last SLAM (plus a certain minimum value?)
##                  - based on the measured distance (to equalize error, instead of closer cones being more important (pythangorean is not linear))


class coneSlamData: #a class to go in Map.Cone.coneConData. This carries some extra data which is only used by the coneConnecter functions
    """some data to go in .slamData of Map.Cone objects"""
    def __init__(self, firstPosition, firstTimestamp, firstBLob=None):
        self.positions = [firstPosition, ] # several (but not all!, see MAX_BLOB_HISTORY) measured positions
        self.timestamps = [firstTimestamp, ] # timestamps for when this cone was spotted
        self.blobs = [firstBLob, ]   # lidar blobs
        self.counter = 1 # spot count
    def append(self, pos, timestamp, blob=None):
        self.positions.append(pos)
        self.timestamps.append(timestamp)
        self.blobs.append(blob)
        if(len(self.positions)>MAX_BLOB_HISTORY):
            self.positions.pop(1);   self.timestamps.pop(1);   self.blobs.pop(1) # delete the second extry (keep the first data point as a sort of anchor)
        self.counter += 1
    def getConePos(self, depth=-1): # calculate the cone position based on the positions[] array
        returnPos = np.zeros(2)
        depth = (min(depth, len(self.positions)) if (depth>0) else len(self.positions))
        for i in range(depth):
            returnPos[0] += self.positions[len(self.positions)-1-i][0] # add to sum
            returnPos[1] += self.positions[len(self.positions)-1-i][1] # add to sum
        returnPos[0] /= depth
        returnPos[1] /= depth
        # returnPos[0] = GF.average(np.array([self.positions[len(self.positions)-1-i][0] for i in range(depth)]))
        # returnPos[1] = GF.average(np.array([self.positions[len(self.positions)-1-i][1] for i in range(depth)]))
        return(returnPos)

def distBetw(posOne: np.ndarray, posTwo: np.ndarray):
    """just a quick macro to GF.distAngleBetwPos"""
    return(GF.distAngleBetwPos(posOne, posTwo)[0])
    #return(math.hypot(*(posTwo-posOne)))

def rotationShift(centerPos, posToShift, shiftAngle):
    """rotate a point around another point (not very efficient (yet?))"""
    dist, angle = GF.distAngleBetwPos(np.array(centerPos), np.array(posToShift))
    return(GF.distAnglePosToPos(dist, angle+shiftAngle, np.array(centerPos)))

def averageAngle(angles: np.ndarray):
    """calculates the average of a list of angles
        NOTE: this means converting to unit vectors, summing and recalculating an angle
            you can't just add the angles together if you want an actually good average"""
    # for i in range(len(angles)):
    #     angles[i] = GF.radRoll(angles[i])
    sumVector = np.zeros(2) #the easiest way to calculate a circular mean (mean that considers rollover), is to use the sum of (unit (or any real)) vectors
    zeroPos = np.zeros(2)
    for angle in angles:
        sumVector += GF.distAnglePosToPos(1.0, angle, zeroPos)
    #return(GF.get_norm_angle_between(zeroPos, sumVector, 0.0))
    return(GF.distAngleBetwPos(zeroPos, sumVector)[1])

def calculateOffsets(carPos: np.ndarray, knownCones: np.ndarray, measuredCones: np.ndarray, errorMagnitudes: np.ndarray):
    """calculates the positional error based on a list of known and measured cones
        uses 'multilateration' (see multilateration.py and google it, it's what GPS uses)"""
    #print("multilat: \n\n")
    calculatedLinearOffset, resultError = multLat.multilaterate(carPos, np.array(measuredCones), np.array([distBetw(carPos, knownCones[i]) for i in range(len(knownCones))]), errorMagnitudes)
    calculatedLinearOffset -= carPos #needed
    #print("\n\n multilat done")
    
    #print("calculated Linear Offset:", calculatedLinearOffset)
    unshiftedConesOnlyLinear = [measuredCones[i]-(calculatedLinearOffset*errorMagnitudes[i]) for i in range(len(measuredCones))]
    #print("unshiftedConesOnlyLinear:"); [print(item) for item in unshiftedConesOnlyLinear]
    
    angles = np.zeros(len(measuredCones))
    for i in range(len(measuredCones)):
        #angles[i] = get_norm_angle_between(carPos, measuredCones[i], get_norm_angle_between(carPos, knownCones[i]))
        angles[i] = GF.distAngleBetwPos(carPos, unshiftedConesOnlyLinear[i])[1] - GF.distAngleBetwPos(carPos, knownCones[i])[1]
        # if(linearTimeOffsets):
        #     angles[i] *= (len(measuredCones) / (i+1.0))
        angles[i] = angles[i] * (1.0 / errorMagnitudes[i])
    calculatedRotationalOffset = averageAngle(angles)
    #print("calculated Rot.Offset:", np.rad2deg(calculatedRotationalOffset))
    
    unshiftedCones = [rotationShift(carPos, unshiftedConesOnlyLinear[i], -calculatedRotationalOffset*errorMagnitudes[i]) for i in range(len(measuredCones))]
    #print("unshiftedCones:"); [print(item) for item in unshiftedCones]
    
    return(calculatedLinearOffset, calculatedRotationalOffset, unshiftedConesOnlyLinear, unshiftedCones)

def updateExistingCone(cone, newPos, timestamp, blob=None):
    """updates the .slamData on an existing cone"""
    if(cone.slamData is not None):
        cone.slamData.append(newPos, timestamp, blob)
        newConePos = cone.slamData.getConePos()
        cone.position[0] = newConePos[0];    cone.position[1] = newConePos[1]
    else:
        print("SLAM_DIY warning: adding 'slamData' to an existing cone")
        cone.slamData = coneSlamData(newPos, timestamp, blob)

def updatePosition(mapToUse, landmarkLists, trust=(1.0, 1.0), makeNewCones=True):
    """runs the whole DIY_SLAM!
        you give it measured landmarks (LiDAR & CompVis data) 
         and it works out which cones those corrospond with.
        if it encouters a measurement where there is no cone,
         it will place a cone there (IF makeNewCones==True)
        updates the car's positions based on the results"""
    # if(not mapToUse.SLAMPresent):
    #     print("warning, SLAM disabled but used anyway?!")
    rightNow = mapToUse.clock()
    lastSLAMtime = mapToUse.car.slamData
    passedTime = rightNow - lastSLAMtime
    lidarLandmarks = landmarkLists[0]
    cameraLandmarks = landmarkLists[1]
    
    ## first, find corrosponding landmarks
    #cones = [] # a list with format [ [existing_cone, [new measurements of that cone]], ...]
    conePointers = [];  knownCones = np.empty((len(lidarLandmarks), 2)); measuredCones = np.empty((len(lidarLandmarks), 2)); magnitudes = np.empty(len(lidarLandmarks))
    blobList = [] # may be filled with only None objects, depending on the contents of the landmarkLists
    newCones = []
    for i in range(len(lidarLandmarks)):
        measurementPos, blob = lidarLandmarks[i]
        overlapsCone, overlappingCone = mapToUse.overlapConeCheck(measurementPos)
        if(overlapsCone):
            
            # multipleOverlaps = -1 #if multiple measurements overlap 1 existing cone
            # for j in range(len(cones)):
            #     if(cones[j][0].ID == overlappingCone.ID):
            #         multipleOverlaps = j
            #         print("SLAM debug: multiple lidar measurements of same existing cone:", overlappingCone.ID, j)
            # if(multipleOverlaps >= 0):
            #     cones[multipleOverlaps][1].append((measurementPos, blob['appendTimestamp']))
            # else:
            #     cones.append([overlappingCone, [(measurementPos, blob['appendTimestamp']),]])
            
            conePointers.append(overlappingCone)
            knownCones[len(conePointers)-1] = np.array(overlappingCone.position) #np.array() call should be redundant
            measuredCones[len(conePointers)-1] = np.array(measurementPos) #np.array() call may be redundant
            magnitudes[len(conePointers)-1] = 1.0
            #magnitudes[coneCounteri] = abs(blob['appendTimestamp'] - lastSLAMtime) / passedTime #the abs() is only if the coneTimestamp is (for some strange reason) older than lastSLAMtime
            blobList.append(blob)
            
            #if(blob['appendTimestamp'] < lastSLAMtime):
            #    print("warning, blob older than lastSLAMtime:", lastSLAMtime - blob['appendTimestamp'])
        else:
            if(blob is not None):
                newCones.append((measurementPos, blob['appendTimestamp'], blob))
            else:
                newCones.append((measurementPos, mapToUse.clock(), blob))
    
    # ## at this point, we have a list of landmarks and matching measurements
    # print("cones:")
    # [print(item) for item in cones]
    
    calculatedLinearOffset = np.zeros(2);  calculatedRotationalOffset = 0.0 #init var
    if(len(conePointers) >= MIN_LANDMARK_COUNT):
        # knownCones = np.empty((coneCounter, 2)); measuredCones = np.empty((coneCounter, 2)); magnitudes = np.empty(coneCounter)
        # i = 0
        # for entry in cones:
        #     knownCone = entry[0].position
        #     for measuredCone, coneTimestamp in entry[1]:
        #         knownCones[i] = knownCone
        #         measuredCones[i] = measuredCone
        #         magnitudes[i] = 1.0
        #         #magnitudes[i] = abs(coneTimestamp - lastSLAMtime) / passedTime #the abs() is only if the coneTimestamp is (for some strange reason) older than lastSLAMtime
        #         i += 1
        knownCones = knownCones[:len(conePointers)]; measuredCones = measuredCones[:len(conePointers)]; magnitudes = magnitudes[:len(conePointers)]
        #print("knownCones: \n", knownCones, "\n measuredCones: \n", measuredCones, "\n magnitudes: \n", magnitudes)
        #print("SLAM list sizes:", len(conePointers), len(newCones))
        calculatedLinearOffset, calculatedRotationalOffset, unshiftedConesOnlyLinear, unshiftedCones = calculateOffsets(mapToUse.car.position, knownCones, measuredCones, magnitudes)
        
        # if((mapToUse.simVars.car is not None) if (mapToUse.simVars is not None) else False):
        #     print("true offsets:", mapToUse.car.position - mapToUse.simVars.car.position, mapToUse.car.angle - mapToUse.simVars.car.angle)
        
        if(GF.distAngleBetwPos(np.zeros(2), calculatedLinearOffset)[0] < MAX_SLAM_CORRECTION):
            for i in range(len(conePointers)):
                unshiftedPos = unshiftedCones[i].copy()
                for j in range(len(conePointers)):
                    if((i != j) and (conePointers[i].ID == conePointers[j].ID)):
                        #print("SLAM debug: multiple lidar measurements of same existing cone:", conePointers[i].ID, i, j)
                        if(i < j): #only do it once
                            #print("dealing with overlap:")
                            overlapCounter = 1 #needed to calculate average at the end
                            for k in range(i+1, len(conePointers)): #go through the list one more time (but only from i+1 onwards)
                                if(conePointers[i].ID == conePointers[j].ID):
                                    unshiftedPos += unshiftedCones[j] #sum up all the overlapping positions
                                    overlapCounter += 1
                            unshiftedPos = unshiftedPos / overlapCounter #get average position
                            #print("overlap average position:", unshiftedPos)
                            break #break out of (j) loop, becuase otherwise this process might happen multiple times
                updateExistingCone(conePointers[i], unshiftedPos, rightNow, blobList[i])
        else:
            print("SLAM overcorrected?:", calculatedLinearOffset, np.rad2deg(calculatedRotationalOffset))
        
        mapToUse.car.position -= (calculatedLinearOffset * trust[0])
        mapToUse.car.angle -= (calculatedRotationalOffset * trust[1])
        mapToUse.car.lastUpdateTimestamp = rightNow # save when the car position was last updated
    else: # if there's NOT enough landmarks to perform SLAM
        ## this i'm not sure about, but i guess just take the measured positions at face value and store them for now
        if(abs(mapToUse.car.velocity) < 0.01): # if the car is just standing still (presumably at the start line)
            for i in range(len(conePointers)):
                updateExistingCone(conePointers[i], measuredCones[i], rightNow, blobList[i])
        # else:
        #     print("too few (known) landmarks to do SLAM")
    
    
    for measuredCone, coneTimestamp, blob in newCones:
        magnitude = 1.0
        #magnitude = abs(coneTimestamp - lastSLAMtime) / passedTime
        unshiftedPos = rotationShift(mapToUse.car.position, measuredCone-(calculatedLinearOffset*magnitude), -calculatedRotationalOffset*magnitude)
        overlapsCone, overlappingCone = mapToUse.overlapConeCheck(unshiftedPos) #it should not matter if you use unshiftedPos or measuredCone for this, 
        if(overlapsCone):
            ## if this happens, it means a newCone was spotted multiple times (because it had no overlapping cone in the earlier check).
            ## alternatively, the lidar error was so bad that (due to the unshifting) it now suddenly overlaps a cone when it previously didn't.
            ## in either case, simply update the overlapping cone's position
            updateExistingCone(overlappingCone, unshiftedPos, rightNow, blob)
        elif(makeNewCones): # extra check to make sure SLAM is actually allowed to make new cones
            leftOrRight = (GF.get_norm_angle_between(mapToUse.car.position, unshiftedPos, mapToUse.car.angle) < 0.0) # (bad) lidar-only test fix: leftOrRight is taken very literally
            # cameraMatchSuccess = False;   leftOrRight = None # init vars
            # for cameraConePos, LorR in cameraLandmarks:
            #     if(mapToUse._overlapConeCheck(cameraConePos, unshiftedPos)):
            #         cameraMatchSuccess = True
            #         leftOrRight = LorR
            # if(cameraMatchSuccess):
            conePlaceSuccess, coneInList = mapToUse.addCone(unshiftedPos, leftOrRight, False)
            print("SLAM debug: adding new cone:", coneInList)
            coneInList.slamData = coneSlamData(unshiftedPos, rightNow, blob)
            # else:
            #     print("SLAM debug: can't add new lidar cone because there is no matching camera cone (to indicate the color)")
            #     ## coneLimbo.append(unshiftedPos) # add the lidar cone to some kind of temporary waiting list, untill the camera has found it and determined its color
    
    mapToUse.car.slamData = rightNow # save when SLAM was last run
    return() # i'm not sure what would be useful to return yet