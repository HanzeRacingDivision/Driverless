# there are 2 ways to simulate lidar data:
# first you take a cone's real (simulated) position and the car's real (simulated) position and you use the cars calculated position to determine where the cone would be measured
# then you can either just slap some noise onto that position, OR
# you can simulate the whole process of sampling the surface of the elipse, add noise to each measurement on tbe surface and re-combine the data to get a cone position with realistic sensor noise.
# one could even go so far as to model the movement of the car as the lidar samples (Which the lidar MCU may or may not account for, IDK yet)
# the first is obviously much easier, but i've already DONE the second one (in a previous version of this sim), in lidarBlobProcSimulated.py
import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use

SIMULATE_SAMPLING = True # simulate the whole process of sampling points on the surface of the cone (slightly more realistic data)

RANGE_LIMIT = 6.0 # (meters) lidar range limit
ANGLE_LIMITS = np.deg2rad(np.array([[-95, 95], [80, -80]])) # angle limits (for each lidar). The kart itself is in the way of a certain portion of the lidar's FOV

## for SIMULATE_SAMPLING == False
STD_DEV_LIDAR_POS_ERROR = 0.0/1000 # (in meters)
#STD_DEV_LIDAR_POS_ERROR_PER_METER = 0.01 # (in meters) std dev of positional error (directly added to (x,y)) at 1 meter. More distance means more error
## for SIMULATE_SAMPLING == True
STD_DEV_LIDAR_MEASUREMENT_ERROR = 0.0/1000 # (in meters) std dev of error in individual LiDAR distance measurements (get from datasheet?)

if(SIMULATE_SAMPLING):
    import lidarBlobsNoNumba as LB
    ## the accurate way: (see 'lidar resolution vs range calculator.xlsx' for explanatory diagram)
    LIDAR_SURFACE_REFLECTION_ARC = np.deg2rad(90) # at some point, the surface is insufficiently perpendicular to correctly reflect the laser
    LIDAR_SAMPLERATE = 7500 # (Hz) lidar samples per second
    LIDAR_ROTATIONS_PER_SECOND = 5 # (Hz) lidar full rotations per second (5 RPS == 300 RPM)
    LIDAR_ANGULAR_RESOLUTION = (LIDAR_ROTATIONS_PER_SECOND * 2 * np.pi) / LIDAR_SAMPLERATE # radians between samples
    LIDAR_REFLECTION_ARC_RATIO = lambda coneDiam : ((coneDiam/2) * np.sin(LIDAR_SURFACE_REFLECTION_ARC/2))
    LIDAR_SAMPLES_AT_DIST = lambda dist, coneDiam : int((2 * np.arcsin(LIDAR_REFLECTION_ARC_RATIO(coneDiam) / dist)) / LIDAR_ANGULAR_RESOLUTION)

    # RANGE_LIMIT = min(RANGE_LIMIT, LIDAR_REFLECTION_ARC_RATIO / np.sin(LB.MIN_BLOB_CONE_LEN*LIDAR_ANGULAR_RESOLUTION/2)) # recalculate true max range based on the parameters above. NOTE: comment this to overwrite
    # print("simulatedLidar RANGE_LIMIT:", RANGE_LIMIT)

LIDAR_FULL_ROTATION_FREQ = 5 # 300 RPM == 5 RPS
## variables:
class simulatedLidarVariables: #a class to go in Map.simVars (these are the only varibles in simulatedLidar.py, this seemed cleaner than using global variables)
    """some data to go in .simVars.lidarSimVars of the Map"""
    def __init__(self):
        self.lidarSimulatedAngle = 0.0 # if the simulated lidar is sampled faster than LIDAR_FULL_ROTATION_FREQ, the lidar's current rotation angle determines what it has seens since last time
        self.lidarLastSampleTimestamp = 0.0


def getCones(mapToUse: Map):
    """simulates LiDAR data and returns a list of cone positions (for all LiDARs)
        enable SIMULATE_SAMPLING to get blob data (and slightly more realistic error)"""
    rightNow = mapToUse.clock()
    returnList = []
    for lidarIndex in range(len(mapToUse.car.lidarOffsets)):
        returnList.append([]) # initialize an empy array (for each lidar)
        simVariables = mapToUse.simVars.lidarSimVars[lidarIndex]
        timeSinceLastSampling = rightNow - simVariables.lidarLastSampleTimestamp
        prevLidarAngle = simVariables.lidarSimulatedAngle
        simVariables.lidarSimulatedAngle = GF.radRoll((2*np.pi*LIDAR_FULL_ROTATION_FREQ)*rightNow) # absolute sinusoidal angle
        # simVariables.lidarSimulatedAngle += GF.radRoll((2*np.pi*LIDAR_FULL_ROTATION_FREQ)*timeSinceLastSampling) # iterative angle (needless, more floating point error)
        simVariables.lidarLastSampleTimestamp = rightNow

        ## i use distanceToCone() to easily filter the nearby cones, but the distAngle is incorrect (becuase the positions of the cones are incorrect)
        nearbyConeList = None;    simVarsLidarPos = None # init vars
        regularLidarPos = mapToUse.car.calcLidarPos(lidarIndex) # fallback value, in case simVars.car is None (and an initialzation of the variable)
        detectionAngleLimits = ANGLE_LIMITS[lidarIndex] + mapToUse.car.angle # (numpy matrix addition) fallback value, in case simVars.car is None (and an initialzation of the variable)
        coneDiam = Map.Cone.coneLidarDiam(mapToUse.car.lidarOffsets[lidarIndex][2]) # calculate cone diameter at the height of this lidar
        coneLists = mapToUse.cone_lists[False] + mapToUse.cone_lists[True] # fallback value, in case simVars is None (and an initialzation of the variable)
        if(mapToUse.simVars.undiscoveredCones or ((len(mapToUse.simVars.cone_lists[False]) + len(mapToUse.simVars.cone_lists[True])) > 0)): # undiscoveredCones is also used by UI stuff, so may not be entirely helpful
            coneLists = mapToUse.simVars.cone_lists[False] + mapToUse.simVars.cone_lists[True]
        if(mapToUse.simVars.car is not None): # should be true if simulatePositionalDrift is enabled (which is most of the time)
            simVarsLidarPos = mapToUse.simVars.car.calcLidarPos(lidarIndex) 
            detectionAngleLimits = ANGLE_LIMITS[lidarIndex] + mapToUse.simVars.car.angle # (numpy matrix addition)
        if(timeSinceLastSampling < (1/LIDAR_FULL_ROTATION_FREQ)):
            lidarMeasuredAngleRange = np.array([prevLidarAngle, simVariables.lidarSimulatedAngle])
            if(GF.radRange(lidarMeasuredAngleRange[0], *detectionAngleLimits) and GF.radRange(lidarMeasuredAngleRange[1], *detectionAngleLimits)): # if the whole measurement angle range lies within the limits
                detectionAngleLimits = lidarMeasuredAngleRange
            elif(GF.radRange(lidarMeasuredAngleRange[0], *detectionAngleLimits)): # if the lower limit lies within the limit (and therefore the upper does not yet)
                detectionAngleLimits[0] = lidarMeasuredAngleRange[0] # overwrite lower limit, but leave upper limit at the detection limit
            elif(GF.radRange(lidarMeasuredAngleRange[1], *detectionAngleLimits)): # if the upper limit lies within the limit (and therefore the lower does not yet)
                detectionAngleLimits[1] = lidarMeasuredAngleRange[1] # overwrite upper limit, but leave lower limit at the detection limit
            elif(not (GF.radRange(detectionAngleLimits[0], *lidarMeasuredAngleRange) and GF.radRange(detectionAngleLimits[1], *lidarMeasuredAngleRange))): # if the measurement encompasses the limit area, then just do nothing
                # if the whole measurement range lies outside the limits
                # print("debug, ass", np.round(detectionAngleLimits, 3), np.round(lidarMeasuredAngleRange, 3), GF.radRange(lidarMeasuredAngleRange[0], *detectionAngleLimits), GF.radRange(lidarMeasuredAngleRange[1], *detectionAngleLimits))
                continue # skip this lidar's update entirely
        nearbyConeList = mapToUse.distanceToCone((simVarsLidarPos if (mapToUse.simVars.car is not None) else regularLidarPos), coneLists, sortBySomething='SORTBY_ANGL', simpleThreshold=RANGE_LIMIT, angleThreshRange=detectionAngleLimits)
        
        for i in range(len(nearbyConeList)):
            cone, _ = nearbyConeList[i] # just makes things more legible

            conePos = cone.position # fallback value (simulatePositionalDrift must be False or something)
            realActualConePos = (cone.position if (cone.slamData is None) else cone.slamData.positions[0]) # an absolutely JANKY hack, which used to make sense, but now barely does. If you loaded in the mapfile WITHOUT undiscoveredCones, this will still work
            distAngle = GF.distAngleBetwPos((simVarsLidarPos if (mapToUse.simVars.car is not None) else regularLidarPos), realActualConePos) # get dist and angle between true lidar and true cone positions to get the angle that should be measured
            if(mapToUse.simVars.car is not None): # should always be true, but just to be sure
                distAngle[1] += mapToUse.car.angle - mapToUse.simVars.car.angle # now distAngle holds the 'measured' distance and angle
                conePos = GF.distAnglePosToPos(distAngle[0], distAngle[1], regularLidarPos) # apply true (what the real lidar would measure) dist&angle data to the believed (drifted) car pos&angle.
            
            outerEdgeAngleOffset = np.arctan2(coneDiam/2, distAngle[0]) # (draw it out, it's not complicated.) the angle between the center and outer edge of the cone at this distance
            outerEdgeAngles = [distAngle[1] - outerEdgeAngleOffset, distAngle[1] + outerEdgeAngleOffset]
            nearbyConeList[i].append(outerEdgeAngles)
            obscured = False
            for j in range(i):
                #radRangeOne only works with GF_noNumba, but it could be slightly faster maybe
                if(GF.radRange(outerEdgeAngles[0], nearbyConeList[j][2][0], nearbyConeList[j][2][1]) or 
                    GF.radRange(outerEdgeAngles[1], nearbyConeList[j][2][0], nearbyConeList[j][2][1])): # check if the current outerEdgeAngles overlap with existing (closer) outerEdgeAngles
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
                    lidarPoints = LB.MIN_BLOB_CONE_LEN #init var
                    if(distAngle[0] > 0.1): # avoid divide by zero, and generally approaching unreasonable numbers
                        lidarPoints = max(LIDAR_SAMPLES_AT_DIST(distAngle[0], coneDiam), LB.MIN_BLOB_CONE_LEN) #the number of datapoints decreases with distance
                    invCarToConeAngle = GF.radInv(distAngle[1])
                    dataPointAngles = [((-LIDAR_SURFACE_REFLECTION_ARC + ((LIDAR_SURFACE_REFLECTION_ARC/(lidarPoints-1))*i*2))+invCarToConeAngle) for i in range(lidarPoints)]
                    newBlob = None;  appendSuccess=True; #init var
                    for j in range(len(dataPointAngles)):
                        randomMeasurementError = np.random.normal() * STD_DEV_LIDAR_MEASUREMENT_ERROR #add error in a normal distribution (you could scale with dist, but whatever)
                        pointPos = GF.distAnglePosToPos((coneDiam/2) + randomMeasurementError, dataPointAngles[j], conePos)
                        if(j == 0):
                            newBlob = LB.blobCreate(pointPos, mapToUse.car.position, mapToUse.clock())
                        else:
                            appendSuccess = LB.blobAppend(newBlob, pointPos, mapToUse.car.position, mapToUse.clock())
                            if(not appendSuccess):
                                # print("(lidarBlobProcSim) bad blob append?")
                                break;
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
                    posIsValid, conePos = LB.blobToConePos(nearbyConeList[i][3], coneDiam)
                    if(posIsValid): #only send if the pos was successfully calculated
                        returnList[lidarIndex].append((conePos, nearbyConeList[i][3]))
                    else:
                        #print("blob with invalid conePos:", nearbyConeList[i][3])
                        invalidConePoses += 1
            if(invalidConePoses > 0):
                print("invalidConePoses:", invalidConePoses)
    return(returnList) # returns an array (len = num_of_lidars), containing arrays (len = num_of_cones_spotted) containing a tuple (position, simulated_measurement_points)