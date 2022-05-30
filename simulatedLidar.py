# there are 2 ways to simulate lidar data:
# first you take a cone's real (simulated) position and the car's real (simulated) position and you use the cars calculated position to determine where the cone would be measured
# then you can either just slap some noise onto that position, OR
# you can simulate the whole process of sampling the surface of the elipse, add noise to each measurement on tbe surface and re-combine the data to get a cone position with realistic sensor noise.
# one could even go so far as to model the movement of the car as the lidar samples (Which the lidar MCU may or may not account for, IDK yet)
# the first is obviously much easier, but i've already DONE the second one (in a previous version of this sim), in lidarBlobProcSimulated.py
import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use

SIMULATE_SAMPLING = False # simulate the whole process of sampling points on the surface of the cone (slightly more realistic data)

RANGE_LIMIT = 2.0 # (meters) lidar range limit

## for SIMULATE_SAMPLING == False
STD_DEV_LIDAR_POS_ERROR = 1.0/1000 # (in meters)
#STD_DEV_LIDAR_POS_ERROR_PER_METER = 0.01 # (in meters) std dev of positional error (directly added to (x,y)) at 1 meter. More distance means more error
## for SIMULATE_SAMPLING == True
STD_DEV_LIDAR_MEASUREMENT_ERROR = 1.0/1000 # (in meters) std dev of error in individual LiDAR distance measurements (get from datasheet?)
DATAPOINTS_PER_DISTANCE_NUMERATOR = 3.6 * (Map.Cone.coneLidarDiam / (Map.Cone.coneDiam/2))
if(SIMULATE_SAMPLING):
    import lidarBlobsNoNumba as LB

LIDAR_FULL_ROTATION_FREQ = 5 # 300 RPM == 5 RPS
## variables:
class simulatedLidarVariables: #a class to go in Map.simulationVariables (these are the only varibles in simulatedLidar.py, this seemed cleaner than using global variables)
    """some data to go in .simulationVariables of the Map"""
    def __init__(self):
        self.lidarSimulatedAngle = 0.0 # if the simulated lidar is sampled faster than LIDAR_FULL_ROTATION_FREQ, the lidar's current rotation angle determines what it has seens since last time
        self.lidarLastSampleTimestamp = 0.0


def calcLidarPos(mapToUse, lidarIndex=0, interpolation=False, carToUse=None): # 'carToUse' is specifically for simulations with positional drift
    if(carToUse is None):
        carToUse = mapToUse.car
    carPos = carToUse.position.copy()
    carAngle = carToUse.angle
    if(interpolation):
        print("calcLidarPos interpolation is TBD!")
        # dTime = mapToUse.clock() - carToUse.lastFeedbackTimestamp
        # interpolationDiff = carToUse.update(dTime, carToUse.velocity*dTime, False)
        # #carPos += interpolationDiff[0] # numpy array addition
        # carPos[0] = carPos[0] + interpolationDiff[0][0];   carPos[1] = carPos[1] + interpolationDiff[0][1]
        # carAngle = carAngle + interpolationDiff[1]
        ## alternatively, you could make a dummy Car class object, copy the relevant parameters and run .update() on that, but this seemed cleaner
    diagonalDist, angleToLidar = GF.distAngleBetwPos(np.zeros((2)), carToUse.lidarOffsets[lidarIndex]) # (a bit crude) get distance and angle to lidar from car center
    lidarPos = GF.distAnglePosToPos(diagonalDist, carAngle + angleToLidar, carPos)
    #print(carToUse.lidarOffsets[lidarIndex], GF.vectorProjectDist(carPos, lidarPos, carAngle)) # validity check: should print out the same thing twice
    return(lidarPos)

def getCones(mapToUse):
    rightNow = mapToUse.clock()
    returnList = []
    for lidarIndex in range(len(mapToUse.car.lidarOffsets)):
        simVariables = mapToUse.simulationVariables[lidarIndex]
        timeSinceLastSampling = rightNow - simVariables.lidarLastSampleTimestamp
        prevLidarAngle = simVariables.lidarSimulatedAngle
        simVariables.lidarSimulatedAngle = GF.radRoll((2*np.pi*LIDAR_FULL_ROTATION_FREQ)*rightNow) # absolute sinusoidal angle
        # simVariables.lidarSimulatedAngle += GF.radRoll((2*np.pi*LIDAR_FULL_ROTATION_FREQ)*timeSinceLastSampling) # iterative angle (needless, more floating point error)
        simVariables.lidarLastSampleTimestamp = rightNow
        
        # lidarPos = calcLidarPos(mapToUse, lidarIndex)
        # if((mapToUse.car.simulationVariables is not None) if hasattr(mapToUse.car, 'simulationVariables') else False): # should always be true, but just to be sure
        lidarPos = calcLidarPos(mapToUse, lidarIndex, carToUse=mapToUse.car.simulationVariables)
        ## i use distanceToCone() to easily filter the nearby cones, but the distAngle is incorrect (becuase the positions of the cones are incorrect)
        nearbyConeList = mapToUse.distanceToCone(lidarPos, None, sortBySomething='SORTBY_ANGL', simpleThreshold=RANGE_LIMIT, angleThreshRange=([] if (timeSinceLastSampling < (1/LIDAR_FULL_ROTATION_FREQ)) else [prevLidarAngle, simVariables.lidarSimulatedAngle]))
        returnList.append([])
        for i in range(len(nearbyConeList)): # a forloop to check for obscu
            cone, distAngle = nearbyConeList[i]
            # conePos = cone.position
            # if((mapToUse.car.simulationVariables is not None) if hasattr(mapToUse.car, 'simulationVariables') else False): # should always be true, but just to be sure
            realActualConePos = (cone.position if (cone.slamData is None) else cone.slamData[0][0]) # the simulated lidar must use the 'real' (unmoving) cone position to 'measure' from (but if the cone has no slamData yet, use .position)
            distAngle = GF.distAngleBetwPos(lidarPos, realActualConePos) # get dist and angle between true lidar and true cone positions to get the angle that should be measured
            distAngle[1] += mapToUse.car.angle - mapToUse.car.simulationVariables.angle # now distAngle holds the 'measured' distance and angle
            conePos = GF.distAnglePosToPos(distAngle[0], distAngle[1], mapToUse.car.position) # apply true (what the real lidar would measure) dist&angle data to the believed (drifted) car pos&angle.
            
            outerEdgeAngles = [distAngle[1] - np.arctan2(Map.Cone.coneDiam/2, distAngle[0]), distAngle[1] + np.arctan2(Map.Cone.coneDiam/2, distAngle[0])]
            nearbyConeList[i].append(outerEdgeAngles)
            obscured = False
            for j in range(i):
                #radRangeOne only works with GF_noNumba, but it could be slightly faster maybe
                if(GF.radRange(outerEdgeAngles[0], nearbyConeList[j][2][0], nearbyConeList[j][2][1]) or 
                    GF.radRange(outerEdgeAngles[1], nearbyConeList[j][2][0], nearbyConeList[j][2][1])):
                    if(distAngle[0] < nearbyConeList[j][1][0]): #if that cone is further away (and therefore obstructed by the current cone)
                        if(len(nearbyConeList[j]) > 3):# if a blob was made for that cone,
                            #print("(lidarBlobProcSim) popping blob:", nearbyConeList[j][3])
                            nearbyConeList[j].pop(3) #delete that (visually obstructed) blob
                    else: # if that cone is closer than the current one (and so the current one is visually obstructed)
                        obscured = True
                        break # break out of this secondary forloop
            
            if(not obscured): # if the cone would be visible to the lidar
                if(SIMULATE_SAMPLING):
                    # print("simulatedLidar.SIMULATE_SAMPLING is TBD!")
                    adjustedConeDiam = Map.Cone.coneLidarDiam # cone diameter (at the height of the lidar!)
                    lidarPoints = int(6) #init var
                    if(distAngle[0] > 0.01):#avoid divide by zero, and generally approaching unreasonable numbers
                        lidarPoints = max(int(DATAPOINTS_PER_DISTANCE_NUMERATOR / distAngle[0]), 2) #the number of datapoints decreases with distance
                    lidarPointsAngleRange = np.deg2rad(60)
                    invCarToConeAngle = GF.radInv(distAngle[1])
                    dataPointAngles = [((-lidarPointsAngleRange + ((lidarPointsAngleRange/(lidarPoints-1))*i*2))+invCarToConeAngle) for i in range(lidarPoints)]
                    newBlob = None;  appendSuccess=True; #init var
                    for j in range(len(dataPointAngles)):
                        if(not appendSuccess):
                            continue
                        randomMeasurementError = np.random.normal() * STD_DEV_LIDAR_MEASUREMENT_ERROR #add error in a normal distribution (you could scale with dist, but whatever)
                        pointPos = GF.distAnglePosToPos((adjustedConeDiam/2) + randomMeasurementError, dataPointAngles[j], conePos)
                        if(j == 0):
                            newBlob = LB.blobCreate(pointPos, mapToUse.car.position, mapToUse.clock())
                        else:
                            appendSuccess = LB.blobAppend(newBlob, pointPos, mapToUse.car.position, mapToUse.clock())
                            # if(not appendSuccess):
                            #     print("(lidarBlobProcSim) bad blob append?")
                    if(appendSuccess):
                        nearbyConeList[i].append(newBlob)
                else:
                    randomPosError = np.random.normal(0.0, STD_DEV_LIDAR_POS_ERROR, size=cone.position.shape)
                    #randomPosError += np.random.normal(0.0, STD_DEV_LIDAR_POS_ERROR_PER_METER, size=randomPosError.shape) * distAngle[0] # multiply with the distance (further measurements should have a little more error than close ones)
                    #conePos = conePos + randomPosError # numpy array addition
                    conePos[0] = conePos[0] + randomPosError[0];   conePos[1] = conePos[1] + randomPosError[1] # safe addition
                    returnList[lidarIndex].append((conePos, None)) # (note: store None instead of individual measurements (because SIMULATE_SAMPLING == False))
        if(SIMULATE_SAMPLING):
            invalidConePoses = 0
            for i in range(len(nearbyConeList)):
                if(len(nearbyConeList[i]) > 3):
                    posIsValid, conePos = LB.blobToConePos(nearbyConeList[i][3])
                    if(posIsValid): #only send if the pos was successfully calculated
                        returnList[lidarIndex].append((conePos, nearbyConeList[i][3]))
                    else:
                        #print("blob with invalid conePos:", nearbyConeList[i][3])
                        invalidConePoses += 1
            if(invalidConePoses > 0):
                print("invalidConePoses:", invalidConePoses)
    return(returnList) # returns an array (len = num_of_lidars), containing arrays (len = num_of_cones_spotted) containing a tuple (position, simulated_measurement_points)