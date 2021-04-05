import numpy as np
import time
import generalFunctions as GF





class Map:
    """ A parent map class that holds all variables that make up a (basic) track """
    def __init__(self, carStartPos=[0,0]):  # variables here that define the scenario/map
        self.car = self.Car([carStartPos[0], carStartPos[1]])
        self.left_cone_list = []
        self.right_cone_list = []
        self.finish_line_cones = [] #holds 2 Cone objects, 1 left and 1 right (redundant, becuase Cone.isFinish attribute, but this will save a lot of list-searching time)
        
        self.target_list = [] #list of Target objects, the order of the list is the order in which they need to be driven
        self.targets_full_circle = False #if the target list loops around
        
        #self.newConeID = 0 #add 1 after adding a cone #TO BE REPLACED BY PANDAS INDEXING
        
        self.clockStart = time.time()
        self.clock = self.clockGet #pointer to clock function (can be changed to a simulated one)
    
    
    class Car:
        """ A (parent) car class that holds all the variables that make up a (basic) car """
        def __init__(self, pos=[0,0], angle=0.0):
            self.position = np.array([float(pos[0]), float(pos[1])])
            self.angle = angle #car orientation in radians
            self.velocity = 0.0 #measured and filtered car 'forward' (wheel) speed in meters/second (used to update position)
            self.steering = 0.0 #measured and filtered steering angle in radians (used to update position)
            self.wheelbase = 0.25 #meters
            self.chassis_length = 0.37 #meters
            self.chassis_width = 0.20 #meters
            self.chassis_length_offset = 0.03 #(meters) car center + this = chassis center
            
            self.desired_velocity = 0.0 #desired velocity in m/s (sent to car MCU)
            self.desired_steering = 0.0 #desired steering angle in radians (sent to car MCU)
            
            self.maxSteeringAngle = np.deg2rad(25) #the car can't steer harder than this, (and will not accept serial commands outside this range)
            
            ## moved to pathPlanningTemp (and some renamed), can be found in self.pathFolData
            #self.auto = False #(thijs) could do with a clearer name like 'driving' or 'self_driving_active' or something
            #self.max_velocity = 5 #in m/s
            #self.max_acceleration = 4.0 #in m/s^2
            #self.max_steering = np.radians(25) #max angle (in either direction) in radians
            
            #self.rearAxlePos = GF.distAnglePosToPos(self.wheelbase/2, GF.radInv(self.angle), self.position) #updates in Car.update()
            ## there is no need for rearAxlePos to be stored, but it would make Car.update() slightly faster IF (and only if) Car.update() is the ONLY function in which the position is altered
            
            #self.acceleration = 0.0 #acceleration in meters/second^2
            #self.fov_range = 60  #(thijs) this is the actual (camera) field of view variable, but it's only useful for the simulation, so delete this?
            
            #self.lastUpdateTime = time.time() #TBD, timestamp for 
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def update(self, dt):
            """ update the position of the car, based on velocity, steering and time-passage """
            # if(dt < 0.0001):
            #     timeRightNow = time.time() #python is not the fastest language, using time.time() at different points in this function will give different values, this won't
            #     dt = time.time() - lastUpdateTime
            
            #self.velocity += self.acceleration * dt #only for keyboard driving, unless carMCU control system changes
            #self.velocity = max(-self.max_velocity, min(self.velocity, self.max_velocity)) #constraining velocity should not be done here!
            
            #turning math
            if((abs(self.steering) > 0.001) and (abs(self.velocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                rearAxlePos = GF.distAnglePosToPos(self.wheelbase/2, GF.radInv(self.angle), self.position)
                turning_radius = self.wheelbase/np.tan(self.steering)
                angular_velocity = self.velocity/turning_radius
                arcMov = angular_velocity * dt
                
                #one way to do it
                # turning_center = GF.distAnglePosToPos(turning_radius, self.angle+(np.pi/2), rearAxlePos) #get point around which car turns
                # rearAxlePos = GF.distAnglePosToPos(turning_radius, self.angle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long
                
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
                newPosition = GF.distAnglePosToPos(self.wheelbase/2, self.angle, rearAxlePos)
                self.position[0] = newPosition[0] #keep the numpy array object the same (instead of replacing it with a whole new array every time)
                self.position[1] = newPosition[1]
            else:
                self.position[0] = self.position[0] + (dt * self.velocity * np.cos(self.angle)) #for some reason += doesnt work at the start
                self.position[1] = self.position[1] + (dt * self.velocity * np.sin(self.angle))
        
        def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the NEAREST surface of the car
            """ returns the shortest possible distance to the car chassis and whether or not the car overlaps the entered pos """
            translatedDistance = GF.vectorProjectDist(self.position, pos, self.angle)
            #self.debugLines.append([0, self.realToPixelPos(self.position), self.realToPixelPos(pos), 0])
            chassisBackLength = (-self.chassis_length/2)+self.chassis_length_offset #(negative number) length from middle of car to back-end of chassis (bumper)
            chassisFrontLength = (self.chassis_length/2)+self.chassis_length_offset #(positive number) length from middle of car to front-end of chassis (bumper)
            if(((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)) and (abs(translatedDistance[1]) < (self.chassis_width/2))): #if pos lies within the car
                return(True, 0) #return(yes pos overlaps car, 0 distance to car)
            else:
                returnDistance = 0
                if((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)): #if it's beside the car
                    returnDistance = abs(translatedDistance[1]) - (self.chassis_width/2) #shortest dist is a line perp. from the side of the car
                    # debugAngle = (GF.radInv(self.angle) if (translatedDistance[0] < 0) else self.angle)
                    # self.debugLines.append([1, self.realToPixelPos(pos), [returnDistance, GF.distAngleBetwPos(pos, [self.position[0] + abs(translatedDistance[0])*np.cos(debugAngle), self.position[1] + abs(translatedDistance[0])*np.sin(debugAngle)])[1]], 1])
                elif(abs(translatedDistance[1]) < (self.chassis_width/2)): #if it's in front of or behind the car
                    if(translatedDistance[0] > 0): #in front of the car
                        returnDistance = translatedDistance[0] - chassisFrontLength #shortest dist is a line perp. from the front of the car
                    else: #behind the car
                        returnDistance = abs(translatedDistance[0] - chassisBackLength) #shortest dist is a line perp. from the front of the car
                    # debugAngle = self.angle + ((-np.pi/2) if (translatedDistance[1] < 0) else (np.pi/2))
                    # self.debugLines.append([1, self.realToPixelPos(pos), [returnDistance, GF.distAngleBetwPos(pos, [self.position[0] + abs(translatedDistance[1])*np.cos(debugAngle), self.position[1] + abs(translatedDistance[1])*np.sin(debugAngle)])[1]], 1])
                else:
                    tempAngle = np.arctan2(abs(translatedDistance[1]) - (self.chassis_width/2), abs((translatedDistance[0] - chassisBackLength) if (translatedDistance[0]<0) else (translatedDistance[0] - chassisFrontLength)))
                    returnDistance = abs(translatedDistance[1]/np.sin(tempAngle))  #soh  #note: this should never divide by 0, but if it does, change the '<' from previous two if-statements to a '<='
                    #returnDistance = translatedDistance[0]/np.cos(tempAngle)  #cah
                    ## debug:
                    # debugOffsets = [[np.cos(self.pointAngle+self.angle) * self.pointRadius, np.sin(self.pointAngle+self.angle) * self.pointRadius],
                    #         [np.cos(np.pi-self.pointAngle+self.angle) * self.pointRadius, np.sin(np.pi-self.pointAngle+self.angle) * self.pointRadius]]
                    # debugPos = [(self.position[i] + (-1 if (translatedDistance[1]<0) else 1)*debugOffsets[(0 if (((translatedDistance[0]<0) and (translatedDistance[1]<0)) or ((translatedDistance[0]>0) and (translatedDistance[1]>0))) else 1)][i])  for i in range(2)]
                    # self.debugLines.append([0, self.realToPixelPos(pos), self.realToPixelPos(debugPos), 2])
                return(False, returnDistance) #pos doesnt overlap car

    class Cone:
        """ a small class to hold all pertinent information about boundry cones (like position, left-or-right-ness, whether it's part of the finish line, etc) """
        coneDiam = 0.14 #cone diameter in meters (constant)
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
        """ a single point in the path of the car (a target to aim for) """
        def __init__(self, pos=[0,0], isFinish=False):
            #the targets should probably be just be ordered in the list to match their order in the track (shouldnt take much/any shuffling to get done), but if not: use pandas indexing?
            self.position = np.array([pos[0], pos[1]])
            self.isFinish = isFinish
            
            #self.passed = 0 #counts the number of times it's been passed (from path-planning)
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
    
    def clockGet(self): #a function that is passed to self.clock by default
        return(time.time() - self.clockStart)
    
    def clockSet(self, clockFunction): #set a different (simulation) function for self.clock, and insert a pointer to the self as a default argument (only used if function HAS an argument, if this is an issue, avoid usage of setClock() and edit self.clock manually)
        self.clock = clockFunction
        self.clock.__defaults__ = (self, ) #insert this map object by default (little hacky, may be removed later)
    
    def getConeChainLen(self, currentCone: Cone, prevCone=None, lengthMem=1): #(itteratively) determine the lenght of a sequence of connected cones (note: uses Cone.ID)
        """ get the length of the 'chain' of cones that the input cone is connected to """
        connectionCount = len(currentCone.connections)
        if((connectionCount > 1) and (prevCone is None)):
            print("incorrect usage of getConeChainLen()")
            return(-1, currentCone)
        if(connectionCount == 0):
            return(0, currentCone) #you could just return lengthMem, but it should never arrive here sequentially
        elif(connectionCount == 1):
            if(prevCone is None): #start of a sequence
                return(self.getConeChainLen(currentCone.connections[0], currentCone, lengthMem+1)) #start sequence
            else:
                if(currentCone.connections[0].ID == prevCone.ID): #safety check (if prevCone is not None, its ID MUST be equal to the only connection on currentCone)
                    return(lengthMem, currentCone) #end reached
                else:
                    print("serious error in getConeChainLen(). bad data in Cone.connections?:", currentCone.connections, prevCone.connections)
                    return(-1, currentCone)
        else: #technically, this does allow more that 2 connections per cone, but what maniac would do that
            if(lengthMem >= len((self.right_cone_list if currentCone.LorR else self.left_cone_list))): #a crude way of checking if the chain loops
                #print(("right" if currentCone.LorR else "left"), "cone chain is full circle")
                ## alternatively, you could just set a (Map class) boolean to indicate that a full cone circle chain has been reached (and skip all this itteration)
                ## or, you could store (pass on as argument) the start of the chain, and just check if currentCone==startingCone
                return(lengthMem, currentCone)
            else:
                return(self.getConeChainLen(currentCone.connections[(1 if (currentCone.connections[0].ID == prevCone.ID) else 0)], currentCone, lengthMem+1)) #continue sequence
        
    
    
    def distanceToConeSquared(self, pos, conelist=None, sortByDistance=False, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, coneConnectionExclusions='NO_CONN_EXCL', ignoreLinkedConeIDs=[]):
        """ returns a list with squared distances to cones from a given position, 
            allows (optional) filtering based on cone.ID, max-distance, cone-connected-ness 
            and (optional) sorting by distance """   
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.left_cone_list + self.right_cone_list #then search both lists
        returnList = []  #[[conePointer, squaredDist], ]
        for cone in conelist:
            #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
            ignoreCone = False
            if(cone.ID in ignoreConeIDs):
                ignoreCone = True
            coneConnectionCount = len(cone.connections)
            if(((coneConnectionExclusions == 'EXCL_UNCONN') and (coneConnectionCount == 0)) or \
               ((coneConnectionExclusions == 'EXCL_SING_CONN') and (coneConnectionCount == 1)) or \
               ((coneConnectionExclusions == 'EXCL_DUBL_CONN') and (coneConnectionCount == 2)) or \
               ((coneConnectionExclusions == 'EXCL_ANY_CONN') and (coneConnectionCount > 0))):
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
    
    def distanceToCone(self, pos, conelist=None, sortBySomething='DONT_SORT', ignoreConeIDs=[], simpleThreshold=-1.0, coneConnectionExclusions='NO_CONN_EXCL', ignoreLinkedConeIDs=[], angleDeltaTarget=0.0, angleThreshRange=[]): #note: angleThreshRange is [lowBound, upBound]
        """ returns a list with distances and angles to cones from a given position, 
            allows (optional) filtering based on cone.ID, max-distance, cone-connected-ness and angle thresholds
            and (optional) sorting by distance, angle or angle-delta """
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.left_cone_list + self.right_cone_list #then search both lists
        returnList = []  #[[conePointer, [dist, angle]], ]
        hasAngleThreshRange = (len(angleThreshRange) == 2) #init var
        for cone in conelist:
            ignoreCone = False
            if(cone.ID in ignoreConeIDs):
                ignoreCone = True
            coneConnectionCount = len(cone.connections)
            if(((coneConnectionExclusions == 'EXCL_UNCONN') and (coneConnectionCount == 0)) or \
               ((coneConnectionExclusions == 'EXCL_SING_CONN') and (coneConnectionCount == 1)) or \
               ((coneConnectionExclusions == 'EXCL_DUBL_CONN') and (coneConnectionCount == 2)) or \
               ((coneConnectionExclusions == 'EXCL_ANY_CONN') and (coneConnectionCount > 0))):
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
                    if(sortBySomething == 'SORTBY_DIST'):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((distance < returnList[i][1][0]) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == 'SORTBY_ANGL'):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((angle < returnList[i][1][1]) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == 'SORTBY_ANGL_DELT'):
                        insertionDone = False
                        for i in range(len(returnList)):
                            if((GF.radDiff(angle, angleDeltaTarget) < GF.radDiff(returnList[i][1][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                returnList.insert(i, [cone, [distance, angle]])
                                insertionDone = True
                        if(not insertionDone): #if the new entry is larger than the last entry, append it
                            returnList.append([cone, [distance, angle]])
                    elif(sortBySomething == 'SORTBY_ANGL_DELT_ABS'):
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
        """ return whether or not a given position overlaps an existing cone and the cone which it overlaps (if any) """
        boolAnswer = False;   coneListPointer=None #boolAnswer MUST default to False, the other variables dont matter as much
        coneDistToler = self.Cone.coneDiam*1 #overlap tolerance  NOTE: area is square, not round
        combinedConeList = (self.right_cone_list + self.left_cone_list)
        for cone in combinedConeList:
            if((posToCheck[0] > (cone.position[0]-coneDistToler)) and (posToCheck[0] < (cone.position[0]+coneDistToler)) and (posToCheck[1] > (cone.position[1]-coneDistToler)) and (posToCheck[1] < (cone.position[1]+coneDistToler))):
                # if(boolAnswer): #if an overlapping cone was already found
                #     print("multiple cones overlap!?") #OR two cones are very close (but not overlapping), and the posToCheck overlaps with both
                # else:
                boolAnswer = True
                coneListPointer = cone
        return(boolAnswer, coneListPointer) #coneListPointer is None if it doesnt overlap
    
    # def addCone(self, pos, leftOrRight=False, isFinish=False):
    #     if((len(self.finish_line_cones) >= 2) and isFinish):
    #         print("cant set finish cone, there's already 2 of them")
    #         isFinish = False
    #     (self.right_cone_list if leftOrRight else self.left_cone_list).append(self.Cone(self.newConeID, pos, leftOrRight, isFinish))
    #     self.newConeID += 1

