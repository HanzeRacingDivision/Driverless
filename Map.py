from turtle import left
import numpy as np
import time
import GF.generalFunctions as GF

## IMPORTANT: clock requires major overhaul, as this just isnt working. Whenever a new map object is created (anywhere), it resets the clock, (i know why)


class Map:
    """ A parent map class that holds all variables that make up a (basic) track """
    def __init__(self):  # variables here that define the scenario/map
        self.clockStart = time.time()
        self.clock = self.internalClock # defaults to internalClock, but can be anything you want
        
        self.car = self.Car()
        self.cone_lists = {False: [], True: []}
        #self.coneLimbo = [] # TBD: all (recent?) cone detections which lack sufficient data to (confidently) put them into cone_lists. (e.g. lidar cones with no color)
        
        self.target_list = [] #list of Target objects, the order of the list is the order in which they need to be driven
        self.targets_full_circle = False #if the target list loops around
        
        #self.maxConeID = 0
        
        self.pathFolData = None #holds variables for pathPlanning (e.g. spline data), as pathPlannerMapData object
        
        self.simVars = None # holds simulatedLidar variables (and/or other: ...)
    
    def __repr__(self): #print map objects in a legible fashion
        return("Map(typ="+(self.__class__.__name__)+",LEFT:"+str(len(self.cone_lists[False]))+",RIGHT:"+str(len(self.cone_lists[True]))+",TARGETS:"+str(len(self.target_list))+(",TARG_FULL_CIRC,"if self.targets_full_circle else ",")+str(self.car)+")")
    
    class Car:
        """ A (parent) car class that holds all the variables that make up a (basic) car """
        ## some static constants:
        maxSteeringAngle = np.deg2rad(23.115) #the car can't steer harder than this, (and will not accept HW commands outside this range)
        wheelbase = 1.03 # (meters) distance between front and rear axle
        # axleWidth = 0.71 # (meters) distance between the centers of the wheels (a.k.a. 'track')
        chassis_length = 1.56 # (meters) distance bumper to bumper (for drawing/colision-detection)
        chassis_width = 1.06 # (meters) car chassis width ('skirt to skirt', one might say). NOT distance between wheel centers
        chassis_center_offset = (wheelbase/2) + 0.0 # (meters) car pos (rear axle center) + this = chassis center (mostly used for drawing)
        lidarOffsets = (np.array([1.26, 0.0, 0.115]), np.array([-0.27, 0.0, 0.10])) # the positions of the lidars, as ((forward offset, perpendicular offset, height), for all lidars) from the car position (not chassis center)
        cameraOffset = {"pos" : np.array([0.12, 0.0, 1.1]), "tilt" : 0.0} # the position of the camera where pos=(forward offset, perpendicular offset, height), positive tilt means looking upwards
        cameraFOV = np.deg2rad(np.array([69, 55])) # camera Field-Of-View as (horizontal, vertical) in radians
        
        def __init__(self):
            self.position = np.array([0.0, 0.0], dtype=np.float64)
            self.angle = 0.0 #car orientation in radians
            self.velocity = 0.0 #measured and filtered car 'forward' (wheel) speed in meters/second (used to update position)
            self.steering = 0.0 #measured and filtered steering angle in radians (used to update position)
            
            self.desired_velocity = 0.0 #desired velocity in m/s (sent to car MCU)
            self.desired_steering = 0.0 #desired steering angle in radians (sent to car MCU)
            
            self.lastFeedbackTimestamp = 0.0
            self.lastUpdateTimestamp = 0.0
            
            ## moved to pathPlanningTemp (and some renamed), can be found in self.pathFolData
            #self.auto = False #(thijs) could do with a clearer name like 'driving' or 'self_driving_active' or something
            #self.max_velocity = 5 #in m/s
            #self.max_acceleration = 4.0 #in m/s^2
            #self.max_steering = np.radians(25) #max angle (in either direction) in radians
            
            #self.acceleration = 0.0 #acceleration in meters/second^2
            #self.fov_range = 60  #(thijs) this is the actual (camera) field of view variable, but it's only useful for the simulation, so delete this?
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def __repr__(self): #print car objects in a legible fashion
            return("Car(typ="+(self.__class__.__name__)+",pos=["+str(round(self.position[0],2))+','+str(round(self.position[1],2))+"],angle="+str(round(np.rad2deg(self.angle),1))+")")
        
        def _calcOffsetPos(self, offset: np.ndarray, interpolationDt=0.0):
            """calculates the position of (one of) the lidar(s) mounted on the car.
                (optional) use interpolationDt for a more accurate position (DeltaTime since 'Car.lastUpdateTimestamp')"""
            carPos = self.position.copy()
            carAngle = self.angle
            if(abs(interpolationDt) > 0.001): # a little crude, but a very small interpolation is (usualy) insignificant anyway
                print("calcLidarPos interpolation is TBD!")
                # interpolationDiff = self.update(interpolationDt, self.velocity*interpolationDt, False)
                # #carPos += interpolationDiff[0] # numpy array addition
                # carPos[0] = carPos[0] + interpolationDiff[0][0];   carPos[1] = carPos[1] + interpolationDiff[0][1]
                # carAngle = carAngle + interpolationDiff[1]
                ## alternatively, you could make a dummy Car class object, copy the relevant parameters and run .update() on that, but this seemed cleaner
            ## a somewhat crude way:
            diagonalDist, angleToOffsetPos = GF.distAngleBetwPos(np.zeros((2)), offset[0:2]) # (a bit crude) get distance and angle to lidar from car center
            offsetPos = GF.distAnglePosToPos(diagonalDist, carAngle + angleToOffsetPos, carPos)
            ## the better way: TBD(?)
            ## debug:
            # print(np.round(offset[0:2], 2), np.round(GF.vectorProjectDist(carPos, offsetPos, carAngle), 2)) # validity check: should print out the same thing twice
            return(offsetPos)
        
        def getChassisCenterPos(self):
            """calculates the position of the center of chassis (instead of the car's position, which is at the center of rotation)
                used for drawing (and colision detection)
                please ensure chassis_center_offset is set correctly"""
            #return(self._calcOffsetPos(np.array([self.chassis_center_offset, 0.0])))
            return(GF.distAnglePosToPos(float(self.chassis_center_offset), float(self.angle), np.array(self.position))) # a little faster / more direct

        def calcLidarPos(self, lidarIndex=0, interpolationDt=0.0):
            """calculates the position of (one of) the lidar(s) mounted on the car.
                (optional) use interpolationDt for a more accurate position (DeltaTime since 'Car.lastUpdateTimestamp')
                note: this is a macro for _calcOffsetPos()"""
            return(self._calcOffsetPos(self.lidarOffsets[lidarIndex], interpolationDt))
            
        def calcCameraPos(self, interpolationDt=0.0):
            """calculates the position of (one of) the lidar(s) mounted on the car.
                (optional) use interpolationDt for a more accurate position (DeltaTime since 'Car.lastUpdateTimestamp')
                note: this is a macro for _calcOffsetPos()"""
            return(self._calcOffsetPos(self.cameraOffset['pos'], interpolationDt))
        
        #def update() was moved to simCar and realCar, because it should be replaced by SLAM (and/or only used as a quick update between (slower) SLAM updates
        
        def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the NEAREST surface of the car
            """ returns the shortest possible distance to the car chassis and whether or not the car overlaps the entered pos """
            translatedDistance = GF.vectorProjectDist(self.getChassisCenterPos(), pos, self.angle)
            chassisBackLength = (-self.chassis_length/2) #(negative number) length from middle of car to back-end of chassis (bumper)
            chassisFrontLength = (self.chassis_length/2) #(positive number) length from middle of car to front-end of chassis (bumper)
            if(((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)) and (abs(translatedDistance[1]) < (self.chassis_width/2))): #if pos lies within the car
                return(True, 0) #return(yes pos overlaps car, 0 distance to car)
            else:
                returnDistance = 0
                if((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)): #if it's beside the car
                    returnDistance = abs(translatedDistance[1]) - (self.chassis_width/2) #shortest dist is a line perp. from the side of the car
                elif(abs(translatedDistance[1]) < (self.chassis_width/2)): #if it's in front of or behind the car
                    if(translatedDistance[0] > 0): #in front of the car
                        returnDistance = translatedDistance[0] - chassisFrontLength #shortest dist is a line perp. from the front of the car
                    else: #behind the car
                        returnDistance = abs(translatedDistance[0] - chassisBackLength) #shortest dist is a line perp. from the front of the car
                else:
                    tempAngle = np.arctan2(abs(translatedDistance[1]) - (self.chassis_width/2), abs((translatedDistance[0] - chassisBackLength) if (translatedDistance[0]<0) else (translatedDistance[0] - chassisFrontLength)))
                    returnDistance = abs(translatedDistance[1]/np.sin(tempAngle))  #soh  #note: this should never divide by 0, but if it does, change the '<' from previous two if-statements to a '<='
                    #returnDistance = translatedDistance[0]/np.cos(tempAngle)  #cah
                return(False, returnDistance) #pos doesnt overlap car

    class Cone:
        """ a small class to hold all pertinent information about boundry cones (like position, left-or-right-ness, whether it's part of the finish line, etc) """
        coneDiam = 0.2 #cone diameter in meters (constant) (NOTE: no longer used by lidar math, mostly for drawing)
        conePeakHeight = (0.135*0.220)/0.124 + 0.024 # NOTE: not the height of the cone, but rather the height the cone WOULD HAVE reached, were it's peak actually sharp
        coneLidarDiam = lambda height : ((-0.124/0.220)*(height-0.024) + 0.135)
        defaultOverlapTolerance = coneDiam * 2.0 # multiple of coneDiam (radius)
        ## cone connection spacing is set in coneConnecting.py
        def __init__(self, coneID=-1, pos=[0,0], leftOrRight=False, isFinish=False):
            self.ID = coneID  #TO BE REPLACED BY PANDAS INDEXING
            self.position = np.array([pos[0], pos[1]], dtype=np.float64)
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is. True=right, False=left
            self.isFinish = isFinish
            
            self.connections = [] #appendable list, max 2 entries. will contain pointers to cones if succesfully connected
            
            self.coneConData = [] #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def __repr__(self): #print cone objects in a legible fashion
            return("Cone(ID="+str(self.ID)+","+("RIGHT" if self.LorR else "LEFT")+",pos=["+str(round(self.position[0],2))+','+str(round(self.position[1],2))+"],{"+(",".join([str(conn.ID) for conn in self.connections]))+("},FINISH)" if self.isFinish else "})"))

    # class limboCone:
    #     """ when cone measurements are not ready to be turned into full-blown cones, but you don't want to throw away data, use this class"""
    #     def __init__(self, pos=None, leftOrRight=None):
    #         self.position = pos
    #         self.leftOrRight = leftOrRight

    class Target:
        """ a single point in the path of the car (a target to aim for) """
        def __init__(self, pos=[0,0]):
            #the targets should probably be just be ordered in the list to match their order in the track (shouldnt take much/any shuffling to get done), but if not: use pandas indexing?
            self.position = np.array([pos[0], pos[1]], dtype=np.float64)
            #self.isFinish = isFinish #i added this in a previous version, but never actually made use of it (since the 0th target in the list is usually also the finish), but this might be reinstated later on
            
            self.passed = 0 #counts the number of times it's been passed (from path-planning)
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def __repr__(self): #print target objects in a legible fashion
            return("Target(pos=["+str(round(self.position[0],2))+','+str(round(self.position[1],2))+"],passed="+str(self.passed)+")")
    
    def internalClock(self): #a function that is passed to Map.clock by default
        """default clock() function"""
        return(time.time() - self.clockStart)
    
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
                    print("serious error in getConeChainLen(). bad data in Cone.connections?:", currentCone, prevCone) #currentCone.connections, prevCone.connections)
                    return(-1, currentCone)
        else: #technically, this does allow more that 2 connections per cone, but what maniac would do that
            if(lengthMem >= len(self.cone_lists[currentCone.LorR])): #a crude way of checking if the chain loops
                #print(("right" if currentCone.LorR else "left"), "cone chain is full circle")
                ## alternatively, you could just set a (Map class) boolean to indicate that a full cone circle chain has been reached (and skip all this itteration)
                ## or, you could store (pass on as argument) the start of the chain, and just check if currentCone==startingCone
                return(lengthMem, currentCone)
            else:
                return(self.getConeChainLen(currentCone.connections[(1 if (currentCone.connections[0].ID == prevCone.ID) else 0)], currentCone, lengthMem+1)) #continue sequence
        
    def find_finish_cones(self, onlyOneSide=None):
        returnList = {False : None, True :None}
        for LorR in self.cone_lists:
            if((LorR != onlyOneSide) if (onlyOneSide is not None) else False):
                continue # skip this loop if the user specified onlyOneSide (and it wasn't this side)
            for cone in self.cone_lists[LorR]:
                if(cone.isFinish):
                    if(returnList[LorR] is not None):
                        print("ERROR: more than 1 finish cone of the same color exist:", returnList[LorR], cone)
                    returnList[LorR] = cone
        return(returnList)
    
    def distanceToConeSquared(self, pos, conelist=None, sortByDistance=False, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, coneConnectionExclusions='NO_CONN_EXCL', ignoreLinkedConeIDs=[]):
        """ returns a list with squared distances to cones from a given position, 
            allows (optional) filtering based on cone.ID, max-distance, cone-connected-ness 
            and (optional) sorting by distance """   
        if(conelist is None): #if no conelist was entered (probably should do this)
            conelist = self.cone_lists[False] + self.cone_lists[True] #then search both lists
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
            conelist = self.cone_lists[False] + self.cone_lists[True] #then search both lists
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
    
    def _overlapConeCheck(self, posToCheck, conePosToUse, coneDistToler=None):
        """ return whether or not posToCheck overlaps a cone at conePosToUse"""
        if(coneDistToler is None):
            coneDistToler = self.Cone.defaultOverlapTolerance  #overlap tolerance  NOTE: area is square, not round
        return(GF.distSqrdBetwPos(posToCheck, conePosToUse) < (coneDistToler**2))

    def overlapConeCheck(self, posToCheck):
        """ return whether or not a given position overlaps an existing cone and the cone which it overlaps (if any) """
        boolAnswer = False;   coneListPointer=None #boolAnswer MUST default to False, the other variables dont matter as much
        combinedConeList = self.cone_lists[True] + self.cone_lists[False]
        for cone in combinedConeList:
            if(self._overlapConeCheck(posToCheck, cone.position)):
                # if(boolAnswer): #if an overlapping cone was already found
                #     print("multiple cones overlap!?") #OR two cones are very close (but not overlapping), and the posToCheck overlaps with both
                # else:
                boolAnswer = True
                coneListPointer = cone
        return(boolAnswer, coneListPointer) #coneListPointer is None if it doesnt overlap
    
    #@property
    def maxConeID(self):
        """searches both cone_lists to find the largest ID value
            returns 0 if the list was empty"""
        return(int(GF.findMaxAttrIndex((self.cone_lists[True] + self.cone_lists[False]), 'ID')[1]))
    
    def addCone(self, pos, leftOrRight: bool, isFinish=False):
        """add a new cone to the map"""
        overlaps, overlappingCone = self.overlapConeCheck(pos)
        if(overlaps):
            print("cant addConeObj(), there's already a cone there with ID:", overlappingCone.ID)
            return(False, overlappingCone)
        if((len(self.find_finish_cones()) >= 2) and isFinish):
            print("addCone warning: there's already 2 finish cones. adding as non-finish cone...")
            isFinish = False
        aNewCone = self.Cone(self.maxConeID()+1, pos, leftOrRight, bool(isFinish)) #bool is just to make sure it's not a 0/1 int
        self.cone_lists[leftOrRight].append(aNewCone)
        return(True, aNewCone)
    
    def addConeObj(self, coneObj: Cone, checkMaxConeID=True):
        """add a new cone (object) to the map"""
        overlaps, overlappingCone = self.overlapConeCheck(coneObj.position)
        if(overlaps):
            print("cant addConeObj(), there's already a cone there with ID:", overlappingCone.ID)
            return(False, overlappingCone)
        if(checkMaxConeID):
            maxConeID = self.maxConeID()
            if((coneObj.ID <= maxConeID) and ((len(self.cone_lists[False])+len(self.cone_lists[True]))>0)):
                print("addConeObj warning, ID (", coneObj.ID ,") too low! changing it to:", maxConeID+1)
                coneObj.ID = maxConeID+1
        if(coneObj.isFinish and (self.find_finish_cones(coneObj.LorR) is None)):
            print("addConeObj warning: there's already a finish cone. adding as non-finish cone...")
            coneObj.isFinish = False
        self.cone_lists[coneObj.LorR].append(coneObj)
        return(True, coneObj)
    
    def removeCone(self, ID: int, LorRhint=None): #LorRhint is either a boolean (indicating which conelist to search) or None, at which point both lists will be searched
        """remove a cone, given the cone's ID"""
        listToSearch = []; coneIndexInList=-1 #init vars (not needed, but it's clean)
        if((LorRhint is not None) and (type(LorRhint) is bool)):
            listToSearch = self.cone_lists[LorRhint]
        else:
            listToSearch = (self.cone_lists[False] + self.cone_lists[True])
        coneIndexInList = GF.findIndexByClassAttr(listToSearch,'ID', ID)
        if(coneIndexInList < 0):
            print("removeCone failed, couldn't find ID in list (", (("right cone list" if LorRhint else "left cone list") if ((LorRhint is not None) and (type(LorRhint) is bool)) else "both"),")")
            return(False)
        coneToDelete = listToSearch[coneIndexInList]
        if((LorRhint != coneToDelete.LorR) if ((LorRhint is not None) and (type(LorRhint) is bool)) else ((coneIndexInList >= len(self.cone_lists[False])) != coneToDelete.LorR)): #if the cone.LorR value doesnt match the list in which it was found, something went very wrong earlier!
            print("removeCone WARNING: LorR list mismatch:", coneToDelete.LorR)
        
        if(not ((LorRhint is not None) and (type(LorRhint) is bool))): #if both lists were searched, select the list (to delete from)
            listToSearch = (self.cone_lists[True] if (coneIndexInList >= len(self.cone_lists[False])) else self.cone_lists[False])
            coneIndexInList = ((coneIndexInList-len(self.cone_lists[False])) if (coneIndexInList >= len(self.cone_lists[False])) else coneIndexInList)
        ##first, cleanly sever all ties to other things in the system
        for connectedCone in coneToDelete.connections:
            if(len(connectedCone.coneConData) > 0): #it's always a list, but an empty one if coneConnecter is not used
                connectedCone.coneConData.pop((0 if (connectedCone.connections[0].ID == coneToDelete.ID) else 1))
            connectedCone.connections.pop((0 if (connectedCone.connections[0].ID == coneToDelete.ID) else 1)) #this only works because .connections holds pointers, not copies
        ## TBD: delete anything else?, pathFolData?, slamData?
        listToSearch.pop(coneIndexInList) #finally, delete the cone
        return(True)
    
    def removeConeObj(self, coneObj: Cone):
        """remove a cone, given that cone object
            note: this just calls removeCone(ID), for simplicity"""
        # overlaps, overlappingCone = self.overlapConeCheck(coneObj.position)
        # if(overlaps):
        return(self.removeCone(coneObj.ID, coneObj.LorR))

class mapSimVarClass(Map):
    """(to go in Map.simVars) a map object which holds:
        undiscovered cone lists,
        the true car position (if simulatePositionalDrift is enabled)
        and any other simulation-specific variables like 'lidarSimVars'"""
    def __init__(self):
        Map.__init__(self) #init map class
        self.car = None # put a simCar in this Map-like object only if positionalDrift is enabled
        self.lidarSimVars = [] # store simulatedLidarVariables here
        self.undiscoveredCones = True # whether newly added cones should be 'discovered' by (simulated) sensors (True), or instantly added (False)