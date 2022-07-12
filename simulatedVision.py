# the computerVision is relatively easy to simulate,
# as long as you don't want super-entirely-accurate simulation.
# using the 3D drawer to render the camera frames is really not needed, but i mean... it's fancy so... fuck it, right
import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use

RENDER_FRAMES = False # whether to render a fake camera frame

RANGE_LIMIT = 6.0 # (meters) camera range limit (based mostly on resolution and how well the AI model is working, value is TBD!)

STD_DEV_CAMERA_POS_ERROR = 0.0 # (in meters)
#STD_DEV_CAMERA_POS_ERROR_PER_METER = 0.01 # (in meters) std dev of positional error (directly added to (x,y)) at 1 meter. More distance means more error

## TODO: use drawer3D to render a simulated camera frame
# import drawDriverless as DD
# DD.pygameDrawer3D(masterMap, DD.window, resolution)


def getCones(mapToUse: Map):
    """simulates computer-vision data and returns a list of cone positions
        enable RENDER_FRAMES to get a fun visual representation of what the simulated camera 'saw' """
    rightNow = mapToUse.clock()
    returnList = []

    ## i use distanceToCone() to easily filter the nearby cones, but the distAngle is incorrect (becuase the positions of the cones are incorrect)
    nearbyConeList = None;    simVarsCameraPos = None # init vars
    regularCameraPos = mapToUse.car.calcCameraPos() # fallback value, in case simVars.car is None (and an initialzation of the variable)
    FOVangleThresholds = [mapToUse.car.angle - (mapToUse.car.cameraFOV[0]/2), mapToUse.car.angle + (mapToUse.car.cameraFOV[0]/2)] # fallback value, in case simVars is None (and an initialzation of the variable)
    coneLists = mapToUse.cone_lists[False] + mapToUse.cone_lists[True] # fallback value, in case simVars is None (and an initialzation of the variable)
    if(mapToUse.simVars.undiscoveredCones or ((len(mapToUse.simVars.cone_lists[False]) + len(mapToUse.simVars.cone_lists[True])) > 0)): # undiscoveredCones is also used by UI stuff, so may not be entirely helpful
        coneLists = mapToUse.simVars.cone_lists[False] + mapToUse.simVars.cone_lists[True]
    if(mapToUse.simVars.car is not None): # should be true if simulatePositionalDrift is enabled (which is most of the time)
        simVarsCameraPos = mapToUse.simVars.car.calcCameraPos() 
        FOVangleThresholds = [mapToUse.simVars.car.angle - (mapToUse.simVars.car.cameraFOV[0]/2), mapToUse.simVars.car.angle + (mapToUse.simVars.car.cameraFOV[0]/2)] # the angle range of what the camera sees
    nearbyConeList = mapToUse.distanceToCone((simVarsCameraPos if (mapToUse.simVars.car is not None) else regularCameraPos), coneLists, sortBySomething='SORTBY_ANGL', simpleThreshold=RANGE_LIMIT, angleThreshRange=FOVangleThresholds)
    
    for i in range(len(nearbyConeList)):
        cone, _ = nearbyConeList[i] # just makes things more legible

        conePos = cone.position # fallback value (simulatePositionalDrift must be False or something)
        realActualConePos = (cone.position if (cone.slamData is None) else cone.slamData.positions[0]) # an absolutely JANKY hack, which used to make sense, but now barely does. If you loaded in the mapfile WITHOUT undiscoveredCones, this will still work
        distAngle = GF.distAngleBetwPos((simVarsCameraPos if (mapToUse.simVars.car is not None) else regularCameraPos), realActualConePos) # get dist and angle between true camera and true cone positions to get the angle that should be measured
        if(mapToUse.simVars.car is not None): # should always be true, but just to be sure
            distAngle[1] += mapToUse.car.angle - mapToUse.simVars.car.angle # now distAngle holds the 'measured' distance and angle
            conePos = GF.distAnglePosToPos(distAngle[0], distAngle[1], regularCameraPos) # apply true (what the real camera would measure) dist&angle data to the believed (drifted) car pos&angle.
        
        outerEdgeAngleOffset = np.arctan2(Map.Cone.coneDiam/2, distAngle[0]) # (i recommend drawing this, it's not actually complicated) the angle between the center and outer edge of the cone at this distance
        outerEdgeAngles = [distAngle[1] - outerEdgeAngleOffset, distAngle[1] + outerEdgeAngleOffset]
        nearbyConeList[i].append(outerEdgeAngles)
        obscured = False
        for j in range(i):
            #radRangeOne only works with GF_noNumba, but it could be slightly faster maybe
            if(GF.radRange(outerEdgeAngles[0], nearbyConeList[j][2][0], nearbyConeList[j][2][1]) or 
                GF.radRange(outerEdgeAngles[1], nearbyConeList[j][2][0], nearbyConeList[j][2][1])): # check if the current outerEdgeAngles overlap with existing (closer) outerEdgeAngles
                if(distAngle[0] < nearbyConeList[j][1][0]): #if that cone is further away (and therefore obstructed by the current cone)
                    if(len(nearbyConeList[j]) > 3):# if a blob was made for that cone,
                        nearbyConeList[j].pop(3) #delete that (visually obstructed) blob
                else: # if that cone is closer than the current one (and so the current one is visually obstructed)
                    obscured = True
                    break # break out of this secondary forloop
        
        if(not obscured): # if the cone would be visible to the camera
            randomPosError = np.random.normal(0.0, STD_DEV_CAMERA_POS_ERROR, size=cone.position.shape)
            #randomPosError += np.random.normal(0.0, STD_DEV_CAMERA_POS_ERROR_PER_METER, size=randomPosError.shape) * distAngle[0] # multiply with the distance (further measurements should have a little more error than close ones)
            #conePos = conePos + randomPosError # numpy array addition
            conePos[0] = conePos[0] + randomPosError[0];   conePos[1] = conePos[1] + randomPosError[1] # safe addition
            returnList.append((conePos, cone.LorR))
    return(returnList)