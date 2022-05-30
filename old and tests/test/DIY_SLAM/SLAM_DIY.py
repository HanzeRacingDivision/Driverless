import numpy as np

import GF.generalFunctions as GF
import multilateration as multLat

MIN_LANDMARK_COUNT = 4
MAX_SLAM_CORRECTION = 1.0 #meters of distance
MAX_BLOB_HISTORY = 10


## TBD: improve error magnitudes:
##                  - based on time since last SLAM (plus a certain minimum value?)
##                  - based on the measured distance (to equalize error, instead of closer cones being more important (pythangorean is not linear))

def distBetw(posOne: np.ndarray, posTwo: np.ndarray):
    return(GF.distAngleBetwPos(posOne, posTwo)[0])
    #return(math.hypot(*(posTwo-posOne)))

def rotationShift(centerPos, posToShift, shiftAngle):
    dist, angle = GF.distAngleBetwPos(np.array(centerPos), np.array(posToShift))
    return(GF.distAnglePosToPos(dist, angle+shiftAngle, np.array(centerPos)))

def averageAngle(angles: np.ndarray):
    # for i in range(len(angles)):
    #     angles[i] = GF.radRoll(angles[i])
    sumVector = np.zeros(2) #the easiest way to calculate a circular mean (mean that considers rollover), is to use the sum of (unit (or any real)) vectors
    zeroPos = np.zeros(2)
    for angle in angles:
        sumVector += GF.distAnglePosToPos(1.0, angle, zeroPos)
    #return(GF.get_norm_angle_between(zeroPos, sumVector, 0.0))
    return(GF.distAngleBetwPos(zeroPos, sumVector)[1])

def calculateOffsets(carPos: np.ndarray, knownCones: np.ndarray, measuredCones: np.ndarray, errorMagnitudes: np.ndarray):
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
    if(type(cone.slamData) is list):
        if(blob is not None):
            cone.slamData.append((newPos, timestamp, blob, cone.slamData[-1][-1]+1)) #cone position, timestamp, blob, totalSpotCount
        else:
            cone.slamData.append((newPos, cone.slamData[-1][-1]+1)) #cone position, totalSpotCount
        if(len(cone.slamData)>MAX_BLOB_HISTORY): #limit number of sightings to remember
            cone.slamData.pop(1) #delete the 2nd oldest data (keep the very first sighting (for simulateLidar, if nothing else))
        cone.position[0] = GF.average(np.array([item[0][0] for item in cone.slamData]))
        cone.position[1] = GF.average(np.array([item[0][1] for item in cone.slamData]))
    else:
        print("SLAM_DIY warning: adding 'slamData' to an existing cone")
        if(blob is not None):
            cone.slamData = [(newPos, timestamp, blob, 1)] #i'd like to make a 0th entry using the cone's current position, but i dont have a matching blob*
        else:
            cone.slamData = [(cone.position.copy(), 0), (newPos, 1)]

def updatePosition(mapToUse, landmarkLists, trust=(1.0, 1.0), makeNewCones=True, storeBlob=False):
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
    blobList = [] #only used if storeBlob == True
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
            if(storeBlob):
                blobList.append(blob)
            
            if(blob['appendTimestamp'] < lastSLAMtime):
                print("warning, blob older than lastSLAMtime:", lastSLAMtime - blob['appendTimestamp'])
        else:
            newCones.append((measurementPos, blob['appendTimestamp'], blob))
    
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
        
        if(hasattr(mapToUse.car, 'simulationVariables')):
            if(type(mapToUse.car.simulationVariables) is list):
                print("true offsets:", mapToUse.car.position - mapToUse.car.simulationVariables[0], mapToUse.car.angle - mapToUse.car.simulationVariables[1])
            elif(mapToUse.car.simulationVariables is not None): #(type(initMap.car.simulationVariables) is SC.simCar):
                print("true offsets:", mapToUse.car.position - mapToUse.car.simulationVariables.position, mapToUse.car.angle - mapToUse.car.simulationVariables.angle)
        
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
                if(storeBlob):
                    updateExistingCone(conePointers[i], unshiftedPos, rightNow, blobList[i])
                else:
                    updateExistingCone(conePointers[i], unshiftedPos, rightNow)
        else:
            print("SLAM overcorrected?:", calculatedLinearOffset, np.rad2deg(calculatedRotationalOffset))
        
        mapToUse.car.position -= (calculatedLinearOffset * trust[0])
        mapToUse.car.angle -= (calculatedRotationalOffset * trust[1])
    # else:
    #     print("too few (known) landmarks to do SLAM")
    #     # ## this i'm not sure about, but i guess just take the measured positions at face value and store them for now
    #     # for i in range(len(conePointers)):
    #     #     if(storeBlob):
    #     #         updateExistingCone(conePointers[i], measuredCones[i], rightNow, blobList[i])
    #     #     else:
    #     #         updateExistingCone(conePointers[i], measuredCones[i], rightNow)
    
    
    for measuredCone, coneTimestamp, blob in newCones:
        magnitude = 1.0
        #magnitude = abs(coneTimestamp - lastSLAMtime) / passedTime
        unshiftedPos = rotationShift(mapToUse.car.position, measuredCone-(calculatedLinearOffset*magnitude), -calculatedRotationalOffset*magnitude)
        overlapsCone, overlappingCone = mapToUse.overlapConeCheck(unshiftedPos) #it should not matter if you use unshiftedPos or measuredCone for this, 
        if(overlapsCone):
            ## if this happens, it means a newCone was spotted multiple times (because it had no overlapping cone in the earlier check).
            ## alternatively, the lidar error was so bad that (due to the unshifting) it now suddenly overlaps a cone when it previously didn't.
            ## in either case, simply update the overlapping cone's position
            if(storeBlob):
                updateExistingCone(overlappingCone, unshiftedPos, rightNow, blob)
            else:
                updateExistingCone(overlappingCone, unshiftedPos, rightNow)
        elif(makeNewCones):
            leftOrRight = (GF.get_norm_angle_between(mapToUse.car.position, unshiftedPos, mapToUse.car.angle) < 0.0) #if the angle relative to car is negative (CW), it's a right-side cone
            
            conePlaceSuccess, coneInList = mapToUse.addCone(unshiftedPos, leftOrRight, False)
            print("SLAM debug: adding new cone based on lidar measurement:", coneInList)
            if(storeBlob):
                coneInList.slamData = [(unshiftedPos, rightNow, blob, 1)]
            else:
                coneInList.slamData = [(unshiftedPos, 1)] #less data, faster code
    
    mapToUse.car.slamData = rightNow
    return()