#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import numpy as np  #general math library
from scipy.interpolate import splprep, splev

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use


class pathPlannerData: #a class to go in Map.Car.coneConData or Map.Car.pathFolData, if Alex approves
    """some data to go in .pathFolData of the car"""
    def __init__(self):
        self.auto = False
        self.nextTarget = None
        
        self.laps = 0
        
        self.targetVelocity = 1.0
        


class pathPlanner():
    """some functions (& constants) to follow (steer towards- and select next) Targets.
        (could potentially be copied along with Map objects (but does not HAVE to be))"""
    def __init__(self, mapToUse=None):
        
        self.mapToUse = self
        if(mapToUse is not None):
            self.mapToUse = mapToUse
        
        self.left_spline = [[], []]
        self.right_spline = [[], []]
        self.path_midpoints_spline = [[], []]
        
        self.boundrySplineSamplingDividend = 0.35
        self.boundrySplineSamplingPower = 1.2
        self.pathSplineSamplingDividend = 0.35
        self.pathSplineSamplingPower = 1.2
        self.pathSplineSmoothing = 0.075 #0 means it MUST cross all points, 1 is too much for the scale car, so just pick whatever looks decent
        
        #self.turning_sharpness = 1.8
        
        self.targetDistSpeedMultiplier = 0.25 #at high speed, the tolerance for targer reached distance should increase (because turning radius increases and accuracy does not). To disable this, set it to 0
        self.targetReachedThreshold = 0.3 #if distance to target (regardless of orientation) is less than this, target is reached (larger means less likely to spin off, smaller means more accurate)
        ##if the target can't be reached, due to turning radius limitations (TBD: slowing down), then passing is enough
        self.targetPassedDistThreshold = self.targetReachedThreshold*2.0 #if it passes a target (without hitting it perfectly), the distance to target should still be less than this
        self.targetPassedAngleThreshold = np.deg2rad(75) #if it passes a target (without hitting it perfectly), the angle to target will probably be more than this
        self.targetMissedAngleThreshold = np.deg2rad(140) #if the angle to target is larger than this, (panic, and) move on to the next target (or something)
        
    def nextTarget(self, currentTarget):
        """returns the next target in the target_list"""
        for i in range(len(self.mapToUse.target_list)):
            if(currentTarget == self.mapToUse.target_list[i]): #slightly risky pointer comparison
                if(i<(len(self.mapToUse.target_list)-1)):
                    return(self.mapToUse.target_list[i+1])
                elif(self.mapToUse.targets_full_circle):
                    #print("target rollover")
                    return(self.mapToUse.target_list[0])
                else:
                    print("PANIC, ran out of targets")
                    return(currentTarget)
    
    def targetUpdate(self, currentTarget, distToTarget, angleToTarget, velocity):
        """returns the next target if the current one is reached/passed/missed, returns the current target if the car is still on track to reach it"""
        if(currentTarget is None): #if the first target hasn't been selected yet
            print("selecting totally new target (TBD!)")
            if(len(self.mapToUse.target_list) > 0):
                return(self.mapToUse.target_list[0])
            else:
                print("there are no targets, how am i supposed to update it >:(")
                return(currentTarget)
        else:
            if(distToTarget < (self.targetReachedThreshold + (self.targetDistSpeedMultiplier*self.targetReachedThreshold*(velocity-1.0)))):
                #print("target reached normally", distToTarget, np.rad2deg(angleToTarget))
                #currentTarget.passed += 1
                return(self.nextTarget(currentTarget))
            elif((distToTarget < (self.targetPassedDistThreshold + (self.targetDistSpeedMultiplier*self.targetPassedDistThreshold*(velocity-1.0)))) and (abs(angleToTarget) > self.targetPassedAngleThreshold)):
                print("target passed kinda wonky", distToTarget, np.rad2deg(angleToTarget))
                #currentTarget.passed += 1
                return(self.nextTarget(currentTarget))
            elif(abs(angleToTarget) > self.targetMissedAngleThreshold):
                print("TARGET MISSED!(?)", distToTarget, np.rad2deg(angleToTarget))
                return(self.nextTarget(currentTarget))
            else: #just keep heading towards current target
                return(currentTarget)
        
    
    def calcAutoDriving(self, saveOutput=True):
        """calculate the steering angle (and speed) required to reach the current target
            (default) if input parameter True the output will be saved to the Car"""
        if(self.mapToUse.car.pathFolData is None):
            self.mapToUse.car.pathFolData = pathPlannerData()
        if(len(self.mapToUse.target_list) == 0):
            print("can't autodrive, there are no targets")
            self.mapToUse.car.pathFolData.auto = False
            return(0.0, 0.0, None)
        if((self.mapToUse.car.pathFolData.nextTarget is None) and saveOutput):
            self.mapToUse.car.pathFolData.nextTarget = self.targetUpdate(None, 0, 0, 0)
        if(self.mapToUse.car.pathFolData.nextTarget is not None): #if saveOutput hasn't prevented the first target from being set
            prevTarget = self.mapToUse.car.pathFolData.nextTarget
            dist, angle = GF.distAngleBetwPos(self.mapToUse.car.position, prevTarget.position)
            angle = GF.radRoll(angle-self.mapToUse.car.angle)
            nextTarget = self.targetUpdate(prevTarget, dist, angle, self.mapToUse.car.velocity) #check whether the target should be updated (if prevTarget reached/passed/missed), and return next/current target
            if(prevTarget != nextTarget): #if it didnt change there's no need to spend time recalculating dist & angle
                dist, angle = GF.distAngleBetwPos(self.mapToUse.car.position, nextTarget.position)
                angle = GF.radRoll(angle-self.mapToUse.car.angle)
            
            #desired_steering = min(max(np.arctan(angle/(dist**self.turning_sharpness)), -25), 25)
            desired_steering = min(max(angle,-self.mapToUse.car.maxSteeringAngle),self.mapToUse.car.maxSteeringAngle)
            desired_velocity = self.mapToUse.car.pathFolData.targetVelocity
            if(saveOutput):
                self.mapToUse.car.pathFolData.nextTarget = nextTarget
                self.mapToUse.car.desired_steering = desired_steering
                self.mapToUse.car.desired_velocity = desired_velocity
                if(prevTarget != nextTarget):
                    prevTarget.passed += 1
                    if(nextTarget == self.mapToUse.target_list[0]):
                        self.mapToUse.car.pathFolData.laps += 1
                        print("target rollover, laps done:", self.mapToUse.car.pathFolData.laps)
            return(desired_velocity, desired_steering, nextTarget)
        else: #if no first target was selected (saveOutput=False)
            return(0.0, 0.0, self.targetUpdate(None, 0, 0, 0))
    
    ## cubic spline 
    def makeBoundrySpline(self, inputConeList):
        """calculate and return a qubic spline for a cone_list (left or right)"""
        returnList = [[], []]
        if(len(inputConeList) > 1):
            startingCone = inputConeList[0] #start at any random cone
            if(len(self.mapToUse.finish_line_cones) > 0):
                for finishCone in self.mapToUse.finish_line_cones:
                    if(inputConeList[0].LorR == finishCone.LorR):
                        startingCone = finishCone #use finish line cone as start of chain
            itt = 0
            while((len(startingCone.connections) < 1) and (itt < len(inputConeList))): #try to find a cone that has a connection
                startingCone = inputConeList[itt] #try the next entry
                itt += 1
            if(len(startingCone.connections) < 1): #check if it found a cone with connections (if not, itt == len(inputConeList))
               #print("couldn't make boundry spline, no connected cones in list")
               return([[], []])
            chainLen, startingCone = self.mapToUse.getConeChainLen(startingCone, ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)) #go to the end of the cone connection chain
            fullCircleLoop = False
            if(chainLen < 1): #should NEVER happen
                print("no splines here, chainlen:", chainLen)
                return([[], []])
            elif((len(startingCone.connections) >= 2) and (chainLen >= len(inputConeList))): #if the chain is full-circle (looping)
                fullCircleLoop = True
            else: #if it's NOT a full-circle (looping) cone chain
                #the starting point has been found, now check how long the actual chain is (INEFFICIENT, but makes later forLoop easier)
                chainLen, startingCone = self.mapToUse.getConeChainLen(startingCone, ((startingCone.connections[0]) if (len(startingCone.connections)>1) else None)) #go to the end of the cone connection chain
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
            unew = np.arange(0, 1.01, self.boundrySplineSamplingDividend/(len(xValues)**self.boundrySplineSamplingPower)) #more cones  = less final var
            returnList = splev(unew, tck)
        return(returnList)
    
    def makeBoundrySplines(self):
        """make and store qubic splines for both left_ and right_cone_list"""
        self.left_spline = self.makeBoundrySpline(self.mapToUse.left_cone_list)
        self.right_spline = self.makeBoundrySpline(self.mapToUse.right_cone_list)
    
    def makePathSpline(self):
        """calculate and return a qubic spline for the target_list"""
        self.path_midpoints_spline = [[], []]
        if(len(self.mapToUse.target_list) > 1):
            targetPositions = [[target.position[i] for target in self.mapToUse.target_list] for i in range(2)]
            if(self.mapToUse.targets_full_circle): #close the gap if the list goes full circle (STARTING AND ENDING SPLINE LINES DO NOT MATCH/FLOW-OVER)
                targetPositions[0].append(self.mapToUse.target_list[0].position[0])
                targetPositions[1].append(self.mapToUse.target_list[0].position[1])
            tck, u = splprep(targetPositions, s=self.pathSplineSmoothing, k = (1 if (len(self.mapToUse.target_list)<3) else 2)) #(thijs) I DONT KNOW IF 's' SHOULD BE 0 OR 1, i dont know what it does
            unew = np.arange(0, 1.01, self.pathSplineSamplingDividend/(len(self.mapToUse.target_list)**self.pathSplineSamplingPower)) #more cones  = less final var
            self.path_midpoints_spline = splev(unew, tck)


