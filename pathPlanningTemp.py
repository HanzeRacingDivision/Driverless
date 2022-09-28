#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import numpy as np  #general math library
from scipy.interpolate import splprep, splev

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use

IDEAL_VELOCITY = 1.0 # (m/s) the ideal velocity (at which the pathfollowing works best)
MINIMUM_VELOCITY = 0.5 # (m/s) the minimum velocity at which the motor can run

## DEV NOTES:
# i added spline target driving (bit of a hack), but i only added it for post-first-lap use.
# to use splines before the track is complete would require a lot of tricky hacking in the current system,
#  so if you really want to do that, it might be best to just redesign this whole thing.

## TODO: 
# drive along cubic spline instead of direct target points (almost done)
# tune spline generator(s)
# resolve spline overlap/chain-loop
# start driving along spline targets as soon as the target_list goes full-circle
# partial spline targets (needs lots of tuning and some significant hacks) (may not result in improved performance in first lap at all)
# make target positions update based on the constantly shifting cone positions ??? (doesn't really seem necessary)


class pathPlannerCarData: #a class to go in Map.Car.coneConData or Map.Car.pathFolData, if Alex approves
    """some data to go in .pathFolData of the car"""
    def __init__(self):
        self.auto = False
        self.nextTarget: Map.Target # type hints to help the IDE syntax coloring and stuff. Not required, and should not complain about discrepencies, really just a type HINT.
        self.nextTarget = None # a target object to make some things easier (if you only had an index, you'd also need access to the right list (which may change))
        self.nextTargetIndex = -1 # the index of the current target in the list
        self._useSplineTargets = False # set to true when the first lap is completed
        # self.pathSplineProgressIndex = -1 # indicates how many Map.target_list entries have already been included in the path spline 
        
        self.laps = 0
        
        self.targetVelocity = IDEAL_VELOCITY # this is a variable (instead of a contant) so it can be altered by other parties (like pygameUI)

class pathPlannerMapData: #a class to go in Map.pathFolData (with these variables seperate (instead of inside pathPlanner class), pathPlanner is now fully static)
    """some data to go in .pathFolData of the Map"""
    def __init__(self):
        self.boundrySplines = {False : [], True : []}
        self.targetSpline: list[splineTarget] # type hints to help the IDE syntax coloring and stuff. Not required, and should not complain about discrepencies, really just a type HINT.
        self.targetSpline = []

class splineTarget(Map.Target):
    """to drive along splines instead of regular targets"""
    # idk exactly how i want to do this, as it's a bit of a dirty hack op top of the original system.
    # i can't think of anything that a spline target would need that a regular target wouldn't,
    #  and since coneConData exists (initialized to None) in the Target class by default, this should make things relatively interchangable...
    pass

########################### some (static!!!) functions ####################################

def getNextTarget(mapToUse: Map, targetListToUse: list[Map.Target], currentTarget: Map.Target, currentTargetIndex: int, panic=True):
    """returns the next target (obj) and the index of that target (simple rollover)"""
    if((currentTarget is not None) or (currentTargetIndex >= 0)):
        if(((currentTarget is None) and (currentTargetIndex >= 0)) or ((currentTarget is not None) and (currentTargetIndex < 0))): # basic safety check
            if(panic):
                print("PANIC, currentTarget does not match the currentTargetIndex", currentTarget, currentTargetIndex)
        # elif(currentTarget == targetListToUse[currentTargetIndex]): # extra safety check (does not seem to work properly?? (maybe these dirty pointer comparisons are not so good after all...))
        #     if(panic):
        #         print("PANIC, currentTarget is not at the currentTargetIndex", currentTarget, currentTargetIndex, targetListToUse[currentTargetIndex])
    if(currentTargetIndex<(len(targetListToUse)-1)):
        return(targetListToUse[currentTargetIndex+1], currentTargetIndex+1)
    elif(mapToUse.targets_full_circle):
        #print("target rollover")
        return(targetListToUse[0], 0)
    else:
        if(panic):
            print("PANIC, ran out of targets")
        return(currentTarget, currentTargetIndex)

def targetUpdate(mapToUse: Map, targetListToUse: list[Map.Target], currentTarget: Map.Target, currentTargetIndex: int, distToTarget, angleToTarget, velocity, printDebug=True):
    """returns the next target if the current one is reached/passed/missed, returns the current target if the car is still on track to reach it"""
    if(currentTarget is None): #if the first target hasn't been selected yet
        if(printDebug):
            print("selecting totally new target (TBD!)")
        if(len(targetListToUse) > 0):
            return(targetListToUse[0], 0)
        else:
            if(printDebug):
                print("there are no targets, how am i supposed to update it >:(")
            return(currentTarget, currentTargetIndex)
    else:
        if(distToTarget < (pathPlanner.targetReachedThreshold + pathPlanner.targetDistSpeedMargin(velocity))):
            #print("target reached normally", distToTarget, np.rad2deg(angleToTarget))
            #currentTarget.passed += 1
            return(getNextTarget(mapToUse, targetListToUse, currentTarget, currentTargetIndex))
        elif((distToTarget < (pathPlanner.targetPassedDistThreshold + pathPlanner.targetDistSpeedMargin(velocity))) and (abs(angleToTarget) > pathPlanner.targetPassedAngleThreshold)):
            if(printDebug):
                print("target passed kinda wonky", distToTarget, np.rad2deg(angleToTarget))
            #currentTarget.passed += 1
            return(getNextTarget(mapToUse, targetListToUse, currentTarget, currentTargetIndex))
        elif(abs(angleToTarget) > pathPlanner.targetMissedAngleThreshold):
            print("TARGET MISSED!(?)", distToTarget, np.rad2deg(angleToTarget))
            return(getNextTarget(mapToUse, targetListToUse, currentTarget, currentTargetIndex))
        else: #just keep heading towards current target
            return(currentTarget, currentTargetIndex)
    

def calcAutoDriving(mapToUse: Map, saveOutput=True, printDebug=True):
    """calculate the steering angle (and speed) required to reach the current target
        (default) if input parameter True the output will be saved to the Car
        returns (desired_velocity, desired_steering, nextTarget, nextTargetIndex)"""
    if(mapToUse.car.pathFolData is None):
        mapToUse.car.pathFolData = pathPlannerCarData()
    nextTarget, nextTargetIndex = None, -1 # init var
    debugVar = False
    if((not mapToUse.car.pathFolData._useSplineTargets) and (mapToUse.targets_full_circle) and (len(mapToUse.pathFolData.targetSpline)<=0)): # should only run once (unless makePathSpline fails for some reason)
        print("generating spline targets", len(mapToUse.target_list), mapToUse.car.pathFolData.nextTargetIndex) # debug
        mapToUse.pathFolData.targetSpline = makePathSpline(mapToUse, saveOutput)
        ## NOTE: the spline target list may be generated, but it starts being used when the finish line is crossed.
        ##       if you just find the nearest (suitable) spline target point in here, you can switch over right away
    targetListToUse = mapToUse.target_list # init var
    if(mapToUse.car.pathFolData._useSplineTargets):
        targetListToUse: list[splineTarget]=mapToUse.pathFolData.targetSpline # switch to spline targets (also give type hint)
    if(len(targetListToUse) == 0):
        if(printDebug):
            print("can't autodrive, there are no targets")
        mapToUse.car.pathFolData.auto = False
        return(0.0, 0.0, None, -1)
    if((mapToUse.car.pathFolData.nextTarget is None) and saveOutput):
        mapToUse.car.pathFolData.nextTarget, mapToUse.car.pathFolData.nextTargetIndex = targetUpdate(mapToUse, targetListToUse, None, -1, 0.0, 0.0, 0.0, printDebug)
    if(mapToUse.car.pathFolData.nextTarget is None): #if saveOutput hasn't prevented the first target from being set
        return(0.0, 0.0, *targetUpdate(mapToUse, targetListToUse, None, -1, 0.0, 0.0, 0.0, printDebug))
    else: # if this is not the first target (normal situation)
        prevTarget = mapToUse.car.pathFolData.nextTarget;   prevTargetIndex = mapToUse.car.pathFolData.nextTargetIndex
        dist, angle = GF.distAngleBetwPos(mapToUse.car.position, prevTarget.position)
        angle = GF.radRoll(angle-mapToUse.car.angle)
        nextTarget, nextTargetIndex = targetUpdate(mapToUse, targetListToUse, prevTarget, prevTargetIndex, dist, angle, mapToUse.car.velocity, printDebug) #check whether the target should be updated (if prevTarget reached/passed/missed), and return next/current target
        if(nextTarget is None): # should never happen
            print("target went from valid to None:", prevTarget, nextTarget, type(prevTarget), mapToUse.car.pathFolData._useSplineTargets)
        if(prevTarget != nextTarget): # (slightly dirty pointer comparison) if it didnt change there's no need to spend time recalculating dist & angle
            dist, angle = GF.distAngleBetwPos(mapToUse.car.position, nextTarget.position)
            angle = GF.radRoll(angle-mapToUse.car.angle)
        
        ## velocity target TBD
        desired_velocity = mapToUse.car.pathFolData.targetVelocity

        ## steering target is still very simple
        desired_steering = min(max(angle,-mapToUse.car.maxSteeringAngle),mapToUse.car.maxSteeringAngle) #simply the constrained angle
        lastPredictTarget = mapToUse.car.pathFolData.nextTarget;   lastPredictTargetIndex = mapToUse.car.pathFolData.nextTargetIndex
        for i in range(pathPlanner.steeringPredictDepth):
            predictTarget, predictTargetIndex = getNextTarget(mapToUse, targetListToUse, lastPredictTarget, lastPredictTargetIndex, False)
            if(predictTarget != lastPredictTarget): # slightly shitty pointer comparison
                lastPredictDist, lastPredictAngle = GF.distAngleBetwPos(mapToUse.car.position, lastPredictTarget.position)
                predictDist, predictAngle = GF.distAngleBetwPos(mapToUse.car.position, predictTarget.position)
                ## this is for cutting corners (angling towards the next target early)
                predictAngle = GF.radRoll(predictAngle-mapToUse.car.angle)
                #print("predicting:", desired_steering, pathPlanner.steeringPredictDepthCurve(i+1), pathPlanner.steeringPredictDistCurve(lastPredictDist, desired_velocity), predictAngle, pathPlanner.steeringPredictDepthCurve(i+1) * pathPlanner.steeringPredictDistCurve(lastPredictDist, desired_velocity) * predictAngle)
                desired_steering += pathPlanner.steeringPredictDepthCurve(i+1) * \
                                    pathPlanner.steeringPredictDistCurve(lastPredictDist, desired_velocity) * \
                                    predictAngle
                #print("post-predict:", desired_steering)
                
                ## this is for setting up for sharp corners (going towards the outside corner, so you have the distance you need to actually make the turn)
                ## note: this code is pretty fiddly (and should be replaced by AI as soon as possible), but it does help sometimes
                ##TBD!
                # prevPredictTarget = getPrevTarget(mapToUse, targetListToUse, lastPredictTarget, False)
                # if(prevPredictTarget != lastPredictTarget):
                #     prevAngle = GF.distAngleBetwPos(prevPredictTarget.position, lastPredictTarget.position)[1]
                #     nextAngle = GF.distAngleBetwPos(lastPredictTarget.position, predictTarget.position)[1]
                #     angleDiff = GF.radDiff(prevAngle, nextAngle) #use this angle difference to determine how sharp the turn is
                #     #if(abs(angleDiff) > pathPlanner.steeringPredictCounterThresh):
                #     #    desired_steering += pathPlanner.steeringPredictDepthCurve(i+1) * \
                #     #                        pathPlanner.steeringPredictDistCurve(predictDist, desired_velocity) * \
                #     #                        pathPlanner.steeringPredictCounterCurve(angleDiff) #calculate how aggressively to countersteer
                # #else:
                # #    print("cant do the thing with the countersteer or whatever")

                lastPredictTarget = predictTarget;  lastPredictTargetIndex = predictTargetIndex
            else:
                if(printDebug):
                    print("cant predict target (for steering in early)???", i, nextTarget, lastPredictTarget, predictTarget)
                desired_velocity = MINIMUM_VELOCITY
                break;
        desired_steering = min(max(desired_steering,-mapToUse.car.maxSteeringAngle),mapToUse.car.maxSteeringAngle) #constrain one more time

        if(saveOutput):
            mapToUse.car.pathFolData.nextTarget = nextTarget
            mapToUse.car.pathFolData.nextTargetIndex = nextTargetIndex
            mapToUse.car.desired_steering = desired_steering
            mapToUse.car.desired_velocity = desired_velocity
            if(debugVar):
                print("NOW 1:", mapToUse.car.pathFolData.nextTargetIndex)
            if(prevTarget != nextTarget):
                prevTarget.passed += 1
                if(nextTarget == targetListToUse[0]):
                    mapToUse.car.pathFolData.laps += 1
                    if(printDebug):
                        print("target rollover, laps done:", mapToUse.car.pathFolData.laps)
                    if(not mapToUse.car.pathFolData._useSplineTargets): # when the first lap is completed, we can switch to driving along the spline instead
                        mapToUse.car.pathFolData._useSplineTargets = True
                        print("about to crash:", len(mapToUse.pathFolData.targetSpline), mapToUse.car.pathFolData.nextTarget, type(mapToUse.car.pathFolData.nextTarget))
                        mapToUse.car.pathFolData.nextTarget = mapToUse.pathFolData.targetSpline[0] # switch target object for one from the splinelist (note: index stays 0)
                        print("switching to spline target list!") # debug
                        ## note: the desired steering and velocity are still pointed towards the regular target list, but they'll be updated to the new target next time around (very soon), so it's fine
        return(desired_velocity, desired_steering, nextTarget, nextTargetIndex)

## cubic spline 
def makeBoundrySpline(mapToUse: Map, inputConeList: list[Map.Cone]):
    """calculate and return a qubic spline for a cone_list (left or right)"""
    returnList = []
    if(len(inputConeList) > 1):
        startingCone = inputConeList[0] #start at any random cone
        finishCone = mapToUse.find_finish_cones(inputConeList[0].LorR)[inputConeList[0].LorR]
        if(finishCone is not None):
            startingCone = finishCone #use finish line cone as start of chain
        itt = 0
        while((len(startingCone.connections) < 1) and (itt < len(inputConeList))): #try to find a cone that has a connection
            startingCone = inputConeList[itt] #try the next entry
            itt += 1
        if(len(startingCone.connections) < 1): #check if it found a cone with connections (if not, itt == len(inputConeList))
           #print("couldn't make boundry spline, no connected cones in list")
           return([])
        chainLen, startingCone = mapToUse.getConeChainLen(startingCone, ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)) #go to the end of the cone connection chain
        fullCircleLoop = False
        if(chainLen < 1): #should NEVER happen
            print("no splines here, chainlen:", chainLen)
            return([])
        elif((len(startingCone.connections) >= 2) and (chainLen >= len(inputConeList))): #if the chain is full-circle (looping)
            fullCircleLoop = True
        else: #if it's NOT a full-circle (looping) cone chain
            #the starting point has been found, now check how long the actual chain is (INEFFICIENT, but makes later forLoop easier)
            chainLen, startingCone = mapToUse.getConeChainLen(startingCone, ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)) #go to the end of the cone connection chain
        prevCone = ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)
        xValues = [];  yValues = []
        for i in range(chainLen): #you could do this with a while-loop, sequentially, or like this, and i think this is the simplest/shortest
            xValues.append(startingCone.position[0]);  yValues.append(startingCone.position[1]) #save position
            if(len(startingCone.connections)>1): #the case for all cones except the ones at the very end
                if(startingCone.connections[0].ID == prevCone.ID):
                    prevCone = startingCone
                    startingCone = startingCone.connections[1]
                else:
                    prevCone = startingCone
                    startingCone = startingCone.connections[0]
            else:
                prevCone = startingCone
                startingCone = startingCone.connections[0]
        if(len(xValues) < 2): #should never happen
            print("len(xValues) < 2: ", chainLen)
            return([])
        if(fullCircleLoop): #if cone chain is a full circle
            xValues.append(startingCone.position[0]);  yValues.append(startingCone.position[1]) #save position
        #print("making spline with length", len(xValues), chainLen)
        tck, u = splprep([xValues, yValues], s=0, k = (1 if (len(xValues)<3) else 2)) #(thijs) I DONT KNOW IF 's' SHOULD BE 0 OR 1, i dont know what it does
        unew = np.arange(0, 1.01, pathPlanner.boundrySplineSamplingDividend/(len(xValues)**pathPlanner.boundrySplineSamplingPower)) #more cones  = less final var
        returnList = splev(unew, tck)
        returnList = list(zip(*returnList)) # turn list from [[x,], [y,]] into [[x,y],]
    return(returnList)

def makeBoundrySplines(mapToUse: Map, saveOutput=True):
    """make and store qubic splines from both cone_lists"""
    returnLists = {False : [], True : []}
    for LorR in returnLists: # iterates over keys!
        returnLists[LorR] = makeBoundrySpline(mapToUse, mapToUse.cone_lists[LorR])
    if(saveOutput):
        mapToUse.pathFolData.boundrySplines = returnLists
    return(returnLists)

def _makePathSpline(mapToUse: Map):
    """calculate and return a qubic spline from the target_list"""
    returnList = []
    if(len(mapToUse.target_list) > 1):
        targetPositions = [[target.position[i] for target in mapToUse.target_list] for i in range(2)]
        if(mapToUse.targets_full_circle): #close the gap if the list goes full circle (STARTING AND ENDING SPLINE LINES DO NOT MATCH/FLOW-OVER)
            targetPositions[0].append(mapToUse.target_list[0].position[0])
            targetPositions[1].append(mapToUse.target_list[0].position[1])
        tck, u = splprep(targetPositions, s=pathPlanner.pathSplineSmoothing, k = (1 if (len(mapToUse.target_list)<3) else 2)) #(thijs) I DONT KNOW IF 's' SHOULD BE 0 OR 1, i dont know what it does
        unew = np.arange(0, 1.01, pathPlanner.pathSplineSamplingDividend/(len(mapToUse.target_list)**pathPlanner.pathSplineSamplingPower)) #more cones  = less final var
        returnList = splev(unew, tck)
        returnList = list(zip(*returnList))
    return(returnList)

def makePathSpline(mapToUse: Map, saveOutput=True):
    """calculate and return a list of cubic-spline-based targets to drive along"""
    splinePositions = _makePathSpline(mapToUse)
    splineTargets = [splineTarget(pos) for pos in splinePositions]
    if(saveOutput):
        mapToUse.pathFolData.targetSpline = splineTargets
    return(splineTargets)

# def appendPathSpline(mapToUse): # TBD???: splines for incomplete tracks (by building onto existing partial splines)

class pathPlanner():
    """a static class with constants and (pointers to) functions for (rudimentary) auto-driving and cubic-spline creation.
        The variables that the functions make use of are stored in the Map object in Map.pathFolData and Map.Car.pathFolData"""
    boundrySplineSamplingDividend = 0.35 # trial and error coefficients about how dense/strict the splines are generated
    boundrySplineSamplingPower = 1.2
    pathSplineSamplingDividend = 0.35
    pathSplineSamplingPower = 1.2
    pathSplineSmoothing = 0.075 #0 means it MUST cross all points, 1 is too much for the scale car, so just pick whatever looks decent
    
    
    targetDistSpeedMultiplier = 0.15 #at high speed, the tolerance for targer reached distance should increase (because turning radius increases and accuracy does not). To disable this, set it to 0
    targetReachedThreshold = Map.Car.wheelbase*0.5 #if distance to target (regardless of orientation) is less than this, target is reached (larger means less likely to spin off, smaller means more accurate)
    targetDistSpeedMargin = lambda velocity : (pathPlanner.targetDistSpeedMultiplier*pathPlanner.targetReachedThreshold*(velocity-IDEAL_VELOCITY)) #note: IDEAL_VELOCITY is the value for which targetReachedThreshold is intended
    ##if the target can't be reached, due to turning radius limitations (TBD: slowing down), then passing is enough
    targetPassedDistThreshold = targetReachedThreshold*2.0 #if it passes a target (without hitting it perfectly), the distance to target should still be less than this
    targetPassedAngleThreshold = np.deg2rad(75) #if it passes a target (without hitting it perfectly), the angle to target will probably be more than this
    targetMissedAngleThreshold = np.deg2rad(140) #if the angle to target is larger than this, (panic, and) move on to the next target (or something)

    steeringPredictDepth = 1 #how many targets to predictively steer towards
    steeringPredictDepthStrength = 1.6 #(higher = stronger)
    steeringPredictDepthCurve = lambda depth : (pathPlanner.steeringPredictDepthStrength / (max(depth, 1)**2)) #inverse exponential, starting at a depth of 1 (0 depth would be the current target)
    ##a distance-based steering power curve may be useful (especially for predictive steering)
    steeringPredictDistStrength = 1.7 #(higher = stronger) how sharp/strong the inverse exponential curve of distance over prediction_relevance (plot 1/((x+1)^2) in a graphing calculator)
    steeringPredictDistExpon = 4.0 #(higher = more exponential (sharper)) how sharp/strong the inverse exponential curve of distance over prediction_relevance (plot 1/((x+1)^2) in a graphing calculator)
    steeringPredictDistCurve = lambda dist, velocity=1.0 : min((pathPlanner.steeringPredictDistStrength / (((max(dist, 0.0) * pathPlanner.steeringPredictDistExpon) + (1.0 - ((pathPlanner.targetReachedThreshold + pathPlanner.targetDistSpeedMargin(velocity)) * pathPlanner.steeringPredictDistExpon)))**2)), pathPlanner.steeringPredictDistStrength)
    ## still TODO:
    ##in some situations the car should (besides slowing down), start at the outer edge of the corner, because the turn would otherwise be too tight
    #steeringPredictCounterThresh = np.deg2rad(30) #only if the angle is sharper than this, do we need to perform this pre-emptive counter-steer
    #steeringPredictCounterCurve = lambda turnAngle : TBD  (also note, maybe this should use the same distance function, as this countersteer needs to happen a little further away maybe)
    ##TBD: consider the distance to the predicted target as well (if it's far away, there's less need to cut corners, there will be time to react normally)


# if __name__ == "__main__":
#     