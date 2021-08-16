#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import numpy as np  #general math library
from scipy.interpolate import splprep, splev

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use


class pathPlannerCarData: #a class to go in Map.Car.coneConData or Map.Car.pathFolData, if Alex approves
    """some data to go in .pathFolData of the car"""
    def __init__(self):
        self.auto = False
        self.nextTarget = None
        
        self.laps = 0
        
        self.targetVelocity = 1.0

class pathPlannerMapData: #a class to go in Map.pathFolData (with these variables seperate (instead of inside pathPlanner class), pathPlanner is now fully static)
    """some data to go in .pathFolData of the Map"""
    def __init__(self):
        self.left_spline = [[], []]
        self.right_spline = [[], []]
        self.path_midpoints_spline = [[], []]


########################### some (static!!!) functions ####################################

def nextTarget(mapToUse, currentTarget):
    """returns the next target in the target_list"""
    for i in range(len(mapToUse.target_list)):
        if(currentTarget == mapToUse.target_list[i]): #slightly risky pointer comparison
            if(i<(len(mapToUse.target_list)-1)):
                return(mapToUse.target_list[i+1])
            elif(mapToUse.targets_full_circle):
                #print("target rollover")
                return(mapToUse.target_list[0])
            else:
                print("PANIC, ran out of targets")
                return(currentTarget)

def targetUpdate(mapToUse, currentTarget, distToTarget, angleToTarget, velocity):
    """returns the next target if the current one is reached/passed/missed, returns the current target if the car is still on track to reach it"""
    if(currentTarget is None): #if the first target hasn't been selected yet
        print("selecting totally new target (TBD!)")
        if(len(mapToUse.target_list) > 0):
            return(mapToUse.target_list[0])
        else:
            print("there are no targets, how am i supposed to update it >:(")
            return(currentTarget)
    else:
        if(distToTarget < (pathPlanner.targetReachedThreshold + (pathPlanner.targetDistSpeedMultiplier*pathPlanner.targetReachedThreshold*(velocity-1.0)))):
            #print("target reached normally", distToTarget, np.rad2deg(angleToTarget))
            #currentTarget.passed += 1
            return(nextTarget(mapToUse,currentTarget))
        elif((distToTarget < (pathPlanner.targetPassedDistThreshold + (pathPlanner.targetDistSpeedMultiplier*pathPlanner.targetPassedDistThreshold*(velocity-1.0)))) and (abs(angleToTarget) > pathPlanner.targetPassedAngleThreshold)):
            print("target passed kinda wonky", distToTarget, np.rad2deg(angleToTarget))
            #currentTarget.passed += 1
            return(nextTarget(mapToUse,currentTarget))
        elif(abs(angleToTarget) > pathPlanner.targetMissedAngleThreshold):
            print("TARGET MISSED!(?)", distToTarget, np.rad2deg(angleToTarget))
            return(nextTarget(mapToUse,currentTarget))
        else: #just keep heading towards current target
            return(currentTarget)
    

def calcAutoDriving(mapToUse, saveOutput=True):
    """calculate the steering angle (and speed) required to reach the current target
        (default) if input parameter True the output will be saved to the Car"""
    if(mapToUse.car.pathFolData is None):
        mapToUse.car.pathFolData = pathPlannerCarData()
    if(len(mapToUse.target_list) == 0):
        print("can't autodrive, there are no targets")
        mapToUse.car.pathFolData.auto = False
        return(0.0, 0.0, None)
    if((mapToUse.car.pathFolData.nextTarget is None) and saveOutput):
        mapToUse.car.pathFolData.nextTarget = targetUpdate(mapToUse, None, 0, 0, 0)
    if(mapToUse.car.pathFolData.nextTarget is not None): #if saveOutput hasn't prevented the first target from being set
        prevTarget = mapToUse.car.pathFolData.nextTarget
        dist, angle = GF.distAngleBetwPos(mapToUse.car.position, prevTarget.position)
        angle = GF.radRoll(angle-mapToUse.car.angle)
        nextTarget = targetUpdate(mapToUse, prevTarget, dist, angle, mapToUse.car.velocity) #check whether the target should be updated (if prevTarget reached/passed/missed), and return next/current target
        if(prevTarget != nextTarget): #if it didnt change there's no need to spend time recalculating dist & angle
            dist, angle = GF.distAngleBetwPos(mapToUse.car.position, nextTarget.position)
            angle = GF.radRoll(angle-mapToUse.car.angle)
        
        #desired_steering = min(max(np.arctan(angle/(dist**pathPlanner.turning_sharpness)), -25), 25)
        desired_steering = min(max(angle,-mapToUse.car.maxSteeringAngle),mapToUse.car.maxSteeringAngle)
        desired_velocity = mapToUse.car.pathFolData.targetVelocity
        if(saveOutput):
            mapToUse.car.pathFolData.nextTarget = nextTarget
            mapToUse.car.desired_steering = desired_steering
            mapToUse.car.desired_velocity = desired_velocity
            if(prevTarget != nextTarget):
                prevTarget.passed += 1
                if(nextTarget == mapToUse.target_list[0]):
                    mapToUse.car.pathFolData.laps += 1
                    print("target rollover, laps done:", mapToUse.car.pathFolData.laps)
        return(desired_velocity, desired_steering, nextTarget)
    else: #if no first target was selected (saveOutput=False)
        return(0.0, 0.0, targetUpdate(mapToUse, None, 0, 0, 0))

## cubic spline 
def makeBoundrySpline(mapToUse, inputConeList):
    """calculate and return a qubic spline for a cone_list (left or right)"""
    returnList = [[], []]
    if(len(inputConeList) > 1):
        startingCone = inputConeList[0] #start at any random cone
        if(len(mapToUse.finish_line_cones) > 0):
            for finishCone in mapToUse.finish_line_cones:
                if(inputConeList[0].LorR == finishCone.LorR):
                    startingCone = finishCone #use finish line cone as start of chain
        itt = 0
        while((len(startingCone.connections) < 1) and (itt < len(inputConeList))): #try to find a cone that has a connection
            startingCone = inputConeList[itt] #try the next entry
            itt += 1
        if(len(startingCone.connections) < 1): #check if it found a cone with connections (if not, itt == len(inputConeList))
           #print("couldn't make boundry spline, no connected cones in list")
           return([[], []])
        chainLen, startingCone = mapToUse.getConeChainLen(startingCone, ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)) #go to the end of the cone connection chain
        fullCircleLoop = False
        if(chainLen < 1): #should NEVER happen
            print("no splines here, chainlen:", chainLen)
            return([[], []])
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
            return([[], []])
        if(fullCircleLoop): #if cone chain is a full circle
            xValues.append(startingCone.position[0]);  yValues.append(startingCone.position[1]) #save position
        #print("making spline with length", len(xValues), chainLen)
        tck, u = splprep([xValues, yValues], s=0, k = (1 if (len(xValues)<3) else 2)) #(thijs) I DONT KNOW IF 's' SHOULD BE 0 OR 1, i dont know what it does
        unew = np.arange(0, 1.01, pathPlanner.boundrySplineSamplingDividend/(len(xValues)**pathPlanner.boundrySplineSamplingPower)) #more cones  = less final var
        returnList = splev(unew, tck)
    return(returnList)

def makeBoundrySplines(mapToUse):
    """make and store qubic splines for both left_ and right_cone_list"""
    mapToUse.pathFolData.left_spline = makeBoundrySpline(mapToUse, mapToUse.left_cone_list)
    mapToUse.pathFolData.right_spline = makeBoundrySpline(mapToUse, mapToUse.right_cone_list)

def makePathSpline(mapToUse):
    """calculate and return a qubic spline for the target_list"""
    mapToUse.pathFolData.path_midpoints_spline = [[], []]
    if(len(mapToUse.target_list) > 1):
        targetPositions = [[target.position[i] for target in mapToUse.target_list] for i in range(2)]
        if(mapToUse.targets_full_circle): #close the gap if the list goes full circle (STARTING AND ENDING SPLINE LINES DO NOT MATCH/FLOW-OVER)
            targetPositions[0].append(mapToUse.target_list[0].position[0])
            targetPositions[1].append(mapToUse.target_list[0].position[1])
        tck, u = splprep(targetPositions, s=pathPlanner.pathSplineSmoothing, k = (1 if (len(mapToUse.target_list)<3) else 2)) #(thijs) I DONT KNOW IF 's' SHOULD BE 0 OR 1, i dont know what it does
        unew = np.arange(0, 1.01, pathPlanner.pathSplineSamplingDividend/(len(mapToUse.target_list)**pathPlanner.pathSplineSamplingPower)) #more cones  = less final var
        mapToUse.pathFolData.path_midpoints_spline = splev(unew, tck)

class pathPlanner():
    """a static class with constants and (pointers to) functions for (rudimentary) auto-driving and cubic-spline creation.
        The variables that the functions make use of are stored in the Map object in Map.pathFolData and Map.Car.pathFolData"""
    boundrySplineSamplingDividend = 0.35
    boundrySplineSamplingPower = 1.2
    pathSplineSamplingDividend = 0.35
    pathSplineSamplingPower = 1.2
    pathSplineSmoothing = 0.075 #0 means it MUST cross all points, 1 is too much for the scale car, so just pick whatever looks decent
    
    #turning_sharpness = 1.8
    
    targetDistSpeedMultiplier = 0.25 #at high speed, the tolerance for targer reached distance should increase (because turning radius increases and accuracy does not). To disable this, set it to 0
    targetReachedThreshold = 0.3 #if distance to target (regardless of orientation) is less than this, target is reached (larger means less likely to spin off, smaller means more accurate)
    ##if the target can't be reached, due to turning radius limitations (TBD: slowing down), then passing is enough
    targetPassedDistThreshold = targetReachedThreshold*2.0 #if it passes a target (without hitting it perfectly), the distance to target should still be less than this
    targetPassedAngleThreshold = np.deg2rad(75) #if it passes a target (without hitting it perfectly), the angle to target will probably be more than this
    targetMissedAngleThreshold = np.deg2rad(140) #if the angle to target is larger than this, (panic, and) move on to the next target (or something)
    
    #this is mostly to keep compatibility with my older versions (where the pathPlanner class is inherited into the map object). I can't recommend that, as the map object is often transmitted to other processes/PCs
    @staticmethod
    def nextTarget(mapToUse, currentTarget):
        return(nextTarget(mapToUse, currentTarget))
    
    @staticmethod
    def targetUpdate(mapToUse, currentTarget, distToTarget, angleToTarget, velocity):
        return(targetUpdate(mapToUse, currentTarget, distToTarget, angleToTarget, velocity))
    
    @staticmethod
    def calcAutoDriving(mapToUse, saveOutput=True):
        return(calcAutoDriving(mapToUse, saveOutput))
    
    @staticmethod
    def makeBoundrySpline(mapToUse, inputConeList):
        return(makeBoundrySpline(mapToUse, inputConeList))
    
    @staticmethod
    def makeBoundrySplines(mapToUse):
        return(makeBoundrySplines(mapToUse))
    
    @staticmethod
    def makePathSpline(mapToUse):
        return(makePathSpline(mapToUse))