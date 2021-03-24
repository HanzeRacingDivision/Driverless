import numpy as np
import generalFunctions as FF

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
        
        #self.newConeID = 0 #add 1 after adding a cone #TO BE REPLACED BY PANDAS INDEXING

    class Car:
        def __init__(self, pos=[0,0], angle=0):
            self.position = np.array([pos[0], pos[1]])
            self.angle = angle #car orientation in radians
            self.length = 2 #meters
            self.width = 1 #meters (mostly used for drawing)
            self.velocity = 0.0 #measured and filtered car 'forward' (wheel) speed in meters/second (used to update position)
            self.steering = 0.0 #measured and filtered steering angle in radians (used to update position)
            
            self.desired_velocity = 0.0 #desired velocity in m/s (sent to car MCU)
            self.desired_steering = 0.0 #desired steering angle in radians (sent to car MCU)
            self.auto = False #(thijs) could do with a clearer name like 'driving' or 'self_driving_active' or something
            self.max_velocity = 5 #in m/s
            self.max_acceleration = 4.0 #in m/s^2
            self.max_steering = np.radians(30) #max angle (in either direction) in radians
            
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
            
            ## TO BE FIXED (by math that considers the center of the turning point correctly, ask Thijs to hurry up if you need it now)
            angular_velocity = 0 #init local variable
            if(abs(self.velocity) > 0.001): #avoid divide by 0 error
                angular_velocity = self.velocity * (np.tan(self.steering)/self.length)
            
            if self.steering:
                turning_radius = self.length / np.sin(self.steering)
                angular_velocity = self.velocity / turning_radius
            else:
                angular_velocity = 0

            self.position[0] += dt * self.velocity * np.cos(self.angle)
            self.position[1] += dt * self.velocity * np.sin(self.angle)
            self.angle += angular_velocity * dt
        
        def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the NEAREST surface of the car
            translatedDistance = FF.vectorProjectDist(self.position, pos, self.angle)
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
            self.coneID = coneID  #TO BE REPLACED BY PANDAS INDEXING
            self.position = np.array([pos[0], pos[1]])
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is
            self.isFinish = isFinish
            
            self.connections = [] #appendable list, max 2 entries. will contain pointers to cones if succesfully connected
            
            self.coneConData = None #extra data specifically for cone-connection
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
    
    
    def distanceToConeSquared(self, pos, conelist=None, sortByDistance=False, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, coneConnectionExclusions=NO_CONN_EXCL, ignoreLinkedConeIDs=[]):
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.left_cone_list + self.right_cone_list #then search both lists
        returnList = []  #[[conePointer, squaredDist], ]
        for cone in conelist:
            #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
            ignoreCone = False
            if(cone.coneID in ignoreConeIDs):
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
                        if(connectedCone.coneID == coneIDtoIgnore):
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
            if(cone.coneID in ignoreConeIDs):
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
                        if(connectedCone.coneID == coneIDtoIgnore):
                            ignoreCone = True
            if(not ignoreCone):
                distance, angle = FF.distAngleBetwPos(pos, cone.position) #math obove was moved to a handy function
                if(((distance < simpleThreshold) if (simpleThreshold > 0) else True) and ((FF.radRange(angle, angleThreshRange[0], angleThreshRange[1])) if (hasAngleThreshRange) else True)): #note: (method if boolean else method) used to make sure angleThreshRange isnt used if it's empty
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
                            if((FF.radDiff(angle, angleDeltaTarget) < FF.radDiff(returnList[i][1][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == SORTBY_ANGL_DELT_ABS):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((abs(FF.radDiff(angle, angleDeltaTarget)) < abs(FF.radDiff(returnList[i][1][1], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
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
        combinedConeList = (self.rightConeList + self.leftConeList)
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

