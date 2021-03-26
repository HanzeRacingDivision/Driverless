import numpy as np
import generalFunctions as GF

#constants for distanceToConeSquared, can be replaced with strings, but this is faster
global DONT_SORT, SORTBY_DIST, SORTBY_ANGL, SORTBY_ANGL_DELT, SORTBY_ANGL_DELT_ABS
DONT_SORT = 0
SORTBY_DIST = 1
SORTBY_ANGL = 2
SORTBY_ANGL_DELT = 3
SORTBY_ANGL_DELT_ABS = 4
global NO_CONN_EXCL, EXCL_UNCONN, EXCL_SING_CONN, EXCL_DUBL_CONN
NO_CONN_EXCL = 0
EXCL_UNCONN = 1
EXCL_SING_CONN = 2
EXCL_DUBL_CONN = 3
EXCL_ANY_CONN = 4


class Map:
    """ A Parent Map Class that SLAM, PathPlanning and other simulations inherit from """
    def __init__(self, carStartPos=[0,0]):  # variables here that define the scenario/map
        self.car = self.Car([carStartPos[0], carStartPos[1]])
        self.left_cone_list = []
        self.right_cone_list = []
        self.finish_line_cones = [] #holds 2 Cone objects, 1 left and 1 right (redundant, becuase Cone.isFinish attribute, but this will save a lot of list-searching time)
        
        self.target_list = [] #list of Target objects, the order of the list is the order in which they need to be driven
        
        #self.newConeID = 0 #add 1 after adding a cone #TO BE REPLACED BY PANDAS INDEXING

    class Car:
        def __init__(self, pos=[0,0], angle=0):
            self.position = np.array([pos[0], pos[1]])
            self.angle = angle #car orientation in radians
            self.velocity = 0.0 #measured and filtered car 'forward' (wheel) speed in meters/second (used to update position)
            self.steering = 0.0 #measured and filtered steering angle in radians (used to update position)
            self.length = 2 #meters
            self.width = 1 #meters (mostly used for drawing)
            
            self.desired_velocity = 0.0 #desired velocity in m/s (sent to car MCU)
            self.desired_steering = 0.0 #desired steering angle in radians (sent to car MCU)
            self.auto = False #(thijs) could do with a clearer name like 'driving' or 'self_driving_active' or something
            self.max_velocity = 5 #in m/s
            self.max_acceleration = 4.0 #in m/s^2
            self.max_steering = np.radians(30) #max angle (in either direction) in radians
            
            #self.rearAxlePos = GF.distAnglePosToPos(self.length/2, GF.radInv(self.angle), self.position) #updates in Car.update()
            ## there is no need for rearAxlePos to be stored, but it would make Car.update() slightly faster IF (and only if) Car.update() is the ONLY function in which the position is altered
            
            #self.acceleration = 0.0 #acceleration in meters/second^2
            #self.fov_range = 60  #(thijs) this is the actual (camera) field of view variable, but it's only useful for the simulation, so delete this?
            
            #self.lastUpdateTime = time.time() #TBD, timestamp for 
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM

        def update(self, dt):
            # if(dt < 0.0001):
            #     timeRightNow = time.time() #python is not the fastest language, using time.time() at different points in this function will give different values, this won't
            #     dt = time.time() - lastUpdateTime
            
            #self.velocity += self.acceleration * dt #only for keyboard driving, unless carMCU control system changes
            #self.velocity = max(-self.max_velocity, min(self.velocity, self.max_velocity)) #constraining velocity should not be done here!
            
            #turning math
            if((abs(self.steering) > 0.001) and (abs(self.velocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                rearAxlePos = GF.distAnglePosToPos(self.length/2, GF.radInv(self.angle), self.position)
                turning_radius = self.length/np.tan(self.steering)
                angular_velocity = self.velocity/turning_radius
                arcMov = angular_velocity * dt
                
                #one way to do it
                # turning_center = GF.distAnglePosToPos(turning_radius, carAngle+(np.pi/2), rearAxlePos) #get point around which car turns
                # rearAxlePos = GF.distAnglePosToPos(turning_radius, carAngle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long
                
                #another way of doing it
                forwardMov = np.sin(arcMov)*turning_radius #sin(arc)*turning radius = movement paralel with (old) carAngle
                lateralMov = turning_radius - (np.cos(arcMov)*turning_radius) #sin(arc)*turning radius = movement perpendicular to (old) carAngle
                movAngle = np.arctan2(lateralMov, forwardMov) #
                diagonalMov = forwardMov/np.cos(movAngle) #the length of a line between the start of the arc and the end of the arc
                rearAxlePos = GF.distAnglePosToPos(diagonalMov, self.angle+movAngle, rearAxlePos)
                # rearAxlePos[0] = rearAxlePos[0] + diagonalMov * np.cos(self.angle+movAngle) #same as using distAnglePosToPos
                # rearAxlePos[1] = rearAxlePos[1] + diagonalMov * np.sin(self.angle+movAngle)
                
                #update position
                self.angle += arcMov
                self.position = GF.distAnglePosToPos(self.length/2, self.angle, rearAxlePos)
            else:
                self.position[0] += dt * self.velocity * np.cos(self.angle)
                self.position[1] += dt * self.velocity * np.sin(self.angle)
        
        def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the NEAREST surface of the car
            translatedDistance = GF.vectorProjectDist(self.position, pos, self.angle)
            #simSelf.debugLines.append([0, simSelf.realToPixelPos(self.pos), simSelf.realToPixelPos(pos), 0])
            if((abs(translatedDistance[0]) < (self.length/2)) and (abs(translatedDistance[1]) < (self.width/2))): #if pos lies within the car
                return(True, 0) #return(yes pos overlaps car, 0 distance to car)
            else:
                returnDistance = 0
                if(abs(translatedDistance[0]) < (self.length/2)): #if it's beside the car
                    returnDistance = abs(translatedDistance[1]) - (self.width/2) #shortest dist is a line perp. from the side of the car
                    # debugAngle = (radInv(self.orient) if (translatedDistance[0] < 0) else self.orient)
                    # simSelf.debugLines.append([1, simSelf.realToPixelPos(pos), [returnDistance, distAngleBetwPos(pos, [self.pos[0] + abs(translatedDistance[0])*np.cos(debugAngle), self.pos[1] + abs(translatedDistance[0])*np.sin(debugAngle)])[1]], 1])
                elif(abs(translatedDistance[1]) < (self.width/2)): #if it's in front of or behind the car
                    returnDistance = abs(translatedDistance[0]) - (self.length/2) #shortest dist is a line perp. from the front/back of the car
                    # debugAngle = self.orient + ((-np.pi/2) if (translatedDistance[1] < 0) else (np.pi/2))
                    # simSelf.debugLines.append([1, simSelf.realToPixelPos(pos), [returnDistance, distAngleBetwPos(pos, [self.pos[0] + abs(translatedDistance[1])*np.cos(debugAngle), self.pos[1] + abs(translatedDistance[1])*np.sin(debugAngle)])[1]], 1])
                else:
                    #returnDistance = ((abs(translatedDistance[0]) - (self.length/2))**2 + (abs(translatedDistance[1]) - (self.width/2))**2)**0.5  #pythagoras
                    tempAngle = np.arctan2(abs(translatedDistance[1]) - (self.width/2), abs(translatedDistance[0]) - (self.length/2))
                    returnDistance = abs(translatedDistance[1]/np.sin(tempAngle))  #soh  #note: this should never divide by 0, but if it does, change the '<' from previous two if-statements to a '<='
                    #returnDistance = translatedDistance[0]/np.cos(tempAngle)  #cah
                    ## debug:
                    # debugOffsets = [[np.cos(self.pointAngle+self.orient) * self.pointRadius, np.sin(self.pointAngle+self.orient) * self.pointRadius],
                    #         [np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius, np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius]]
                    # debugPos = [(self.pos[i] + (-1 if (translatedDistance[1]<0) else 1)*debugOffsets[(0 if (((translatedDistance[0]<0) and (translatedDistance[1]<0)) or ((translatedDistance[0]>0) and (translatedDistance[1]>0))) else 1)][i])  for i in range(2)]
                    # simSelf.debugLines.append([0, simSelf.realToPixelPos(pos), simSelf.realToPixelPos(debugPos), 2])
                return(False, returnDistance) #pos doesnt overlap car

    class Cone:
        """ Class for storing the coordinates and visibility of each cone """
        coneDiam = 0.2 #cone diameter in meters (constant)
        def __init__(self, coneID=-1, pos=[0,0], leftOrRight=False, isFinish=False):
            self.ID = coneID  #TO BE REPLACED BY PANDAS INDEXING
            self.position = np.array([pos[0], pos[1]])
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is. True=right, False=left
            self.isFinish = isFinish
            
            self.connections = [] #appendable list, max 2 entries. will contain pointers to cones if succesfully connected
            
            self.coneConData = [] #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM

    class Target:
        """ 'Target' child class only used in PathPlanning """
        def __init__(self, pos=[0,0]):
            #the targets should probably be just be ordered in the list to match their order in the track (shouldnt take much/any shuffling to get done), but if not: use pandas indexing?
            self.position = np.array([pos[0], pos[1]])
            
            #self.passed = 0 #counts the number of times it's been passed (from path-planning)
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
    
    
    def getConeChainLen(self, currentCone, prevCone=None, lengthMem=1): #(itteratively) determine the lenght of a sequence of connected cones (note: uses Cone.ID)
        if(lengthMem >= len((self.right_cone_list if currentCone.LorR else self.left_cone_list))): #a crude way of checking if the chain loops
            print(("right" if currentCone.LorR else "left"), "cone chain is full circle")
            ## alternatively, you could just set a (Map class) boolean to indicate that a full cone circle chain has been reached (and skip all this itteration)
            ## or, you could store (pass on as argument) the start of the chain, and just check if currentCone==startingCone
            return(lengthMem)
        connectionCount = len(currentCone.connections)
        if((connectionCount > 1) and (prevCone is None)):
            print("incorrect usage of getConeChainLen()")
            return(-1)
        if(connectionCount == 0):
            return(0) #you could just return lengthMem, but it should never arrive here sequentially
        elif(connectionCount == 1):
            if(prevCone is None): #start of a sequence
                return(self.getConeChainLen(currentCone.connections[0], currentCone, lengthMem+1)) #start sequence
            else:
                if(currentCone.connections[0].ID == prevCone.ID): #safety check (if prevCone is not None, its ID MUST be equal to the only connection on currentCone)
                    return(lengthMem) #end reached
                else:
                    print("serious error in getConeChainLen(). bad data in Cone.connections?:", currentCone.connections, prevCone.connections)
                    return(-1)
        else: #technically, this does allow more that 2 connections per cone, but what maniac would do that
            return(self.getConeChainLen(currentCone.connections[(1 if (currentCone.connections[0].ID == prevCone.ID) else 0)], currentCone, lengthMem+1)) #continue sequence
        
    
    
    def distanceToConeSquared(self, pos, conelist=None, sortByDistance=False, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, coneConnectionExclusions=NO_CONN_EXCL, ignoreLinkedConeIDs=[]):
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.left_cone_list + self.right_cone_list #then search both lists
        returnList = []  #[[conePointer, squaredDist], ]
        for cone in conelist:
            #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
            ignoreCone = False
            if(cone.ID in ignoreConeIDs):
                ignoreCone = True
            coneConnectionCount = len(cone.connections)
            if(((coneConnectionExclusions == EXCL_UNCONN) and (coneConnectionCount == 0)) or \
               ((coneConnectionExclusions == EXCL_SING_CONN) and (coneConnectionCount == 1)) or \
               ((coneConnectionExclusions == EXCL_DUBL_CONN) and (coneConnectionCount == 2)) or \
               ((coneConnectionExclusions == EXCL_ANY_CONN) and (coneConnectionCount > 0))):
                ignoreCone = True
            elif(coneConnectionCount > 0):
                for coneIDtoIgnore in ignoreLinkedConeIDs: #check if the cone needs to be ignored because of its connections
                    for connectedCone in cone.connections:
                        if(connectedCone.ID == coneIDtoIgnore):
                            ignoreCone = True
            if(not ignoreCone):
                squaredDistance = (pos[0]-cone.position[0])**2 + (pos[1]-cone.position[1])**2  #A^2 + B^2 = C^2
                if((simpleSquaredThreshold < 0) or ((simpleSquaredThreshold > 0) and (squaredDistance < simpleSquaredThreshold))): #simpleSquaredThreshold is less than 0 if no threshold should be used
                    ## returnCone structure: [squaredDistance, coneIndex, cone] (where 'cone' is just a simple pointer to the cone)
                    if(sortByDistance):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((squaredDistance < returnList[i][1]) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, squaredDistance])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, squaredDistance])
                    else:
                        returnList.append([cone, squaredDistance])
        return(returnList)
    
    def distanceToCone(self, pos, conelist=None, sortBySomething=DONT_SORT, ignoreConeIDs=[], simpleThreshold=-1.0, coneConnectionExclusions=NO_CONN_EXCL, ignoreLinkedConeIDs=[], angleDeltaTarget=0.0, angleThreshRange=[]): #note: angleThreshRange is [lowBound, upBound]
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.left_cone_list + self.right_cone_list #then search both lists
        returnList = []  #[[conePointer, [dist, angle]], ]
        hasAngleThreshRange = (len(angleThreshRange) == 2) #init var
        for cone in conelist:
            ignoreCone = False
            if(cone.ID in ignoreConeIDs):
                ignoreCone = True
            coneConnectionCount = len(cone.connections)
            if(((coneConnectionExclusions == EXCL_UNCONN) and (coneConnectionCount == 0)) or \
               ((coneConnectionExclusions == EXCL_SING_CONN) and (coneConnectionCount == 1)) or \
               ((coneConnectionExclusions == EXCL_DUBL_CONN) and (coneConnectionCount == 2)) or \
               ((coneConnectionExclusions == EXCL_ANY_CONN) and (coneConnectionCount > 0))):
                ignoreCone = True
            elif(coneConnectionCount > 0):
                for coneIDtoIgnore in ignoreLinkedConeIDs: #check if the cone needs to be ignored because of its connections
                    for connectedCone in cone.connections:
                        if(connectedCone.ID == coneIDtoIgnore):
                            ignoreCone = True
            if(not ignoreCone):
                distance, angle = GF.distAngleBetwPos(pos, cone.position) #math obove was moved to a handy function
                if(((distance < simpleThreshold) if (simpleThreshold > 0) else True) and ((GF.radRange(angle, angleThreshRange[0], angleThreshRange[1])) if (hasAngleThreshRange) else True)): #note: (method if boolean else method) used to make sure angleThreshRange isnt used if it's empty
                    #if it gets here, the cone fits the requirements and needs to be placed in returnList
                    if(sortBySomething == SORTBY_DIST):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((distance < returnList[i][1][0]) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == SORTBY_ANGL):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((angle < returnList[i][1][1]) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == SORTBY_ANGL_DELT):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((GF.radDiff(angle, angleDeltaTarget) < GF.radDiff(returnList[i][1][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == SORTBY_ANGL_DELT_ABS):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((abs(GF.radDiff(angle, angleDeltaTarget)) < abs(GF.radDiff(returnList[i][1][1], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    else:
                        returnList.append([cone, [distance, angle]])
        return(returnList)
    
    def overlapConeCheck(self, posToCheck):
        boolAnswer = False;   coneListPointer=None #boolAnswer MUST default to False, the other variables dont matter as much
        coneDistToler = self.Cone.coneDiam*2 #overlap tolerance  NOTE: area is square, not round
        combinedConeList = (self.right_cone_list + self.left_cone_list)
        for cone in combinedConeList:
            if((posToCheck[0] > (cone.position[0]-coneDistToler)) and (posToCheck[0] < (cone.position[0]+coneDistToler)) and (posToCheck[1] > (cone.position[1]-coneDistToler)) and (posToCheck[1] < (cone.position[1]+coneDistToler))):
                if(boolAnswer): #if an overlapping cone was already found
                    print("multiple cones overlap!?")
                else:
                    boolAnswer = True
                    coneListPointer = cone
        return(boolAnswer, coneListPointer) #coneListPointer is None if it doesnt overlap
    
    # def addCone(self, pos, leftOrRight=False, isFinish=False):
    #     if((len(self.finish_line_cones) >= 2) and isFinish):
    #         print("cant set finish cone, there's already 2 of them")
    #         isFinish = False
    #     (self.right_cone_list if leftOrRight else self.left_cone_list).append(self.Cone(self.newConeID, pos, leftOrRight, isFinish))
    #     self.newConeID += 1

