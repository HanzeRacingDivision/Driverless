import numpy as np
import time
import GF.generalFunctions as GF

## IMPORTANT: clock requires major overhaul, as this just isnt working. Whenever a new map object is created (anywhere), it resets the clock, (i know why)


class Map:
    """ A parent map class that holds all variables that make up a (basic) track """
    def __init__(self):  # variables here that define the scenario/map
        self.clockStart = time.time()
        self.clock = self.internalClock #default to internalClock
        
        self.car = self.Car()
        self.left_cone_list = []
        self.right_cone_list = []
        self.finish_line_cones = [] #holds 2 Cone objects, 1 left and 1 right (redundant, becuase Cone.isFinish attribute, but this will save a lot of list-searching time)
        
        self.target_list = [] #list of Target objects, the order of the list is the order in which they need to be driven
        self.targets_full_circle = False #if the target list loops around
        
        #self.maxConeID = 0
        
        self.pathFolData = None #holds variables for pathPlanning (e.g. spline data), as pathPlannerMapData object
        
        #(safe) defaults
        self.coneConnecterPresent = False
        self.pathFinderPresent = False
        self.pathPlanningPresent = False
        self.SLAMPresent = False
    
    def __repr__(self): #print map objects in a legible fashion
        return("Map(typ="+(self.__class__.__name__)+",LEFT:"+str(len(self.left_cone_list))+",RIGHT:"+str(len(self.right_cone_list))+",FINISH:"+str(len(self.finish_line_cones))+",TARGETS:"+str(len(self.target_list))+(",TARG_FULL_CIRC,"if self.targets_full_circle else ",")+str(self.car)+")")
    
    class Car:
        """ A (parent) car class that holds all the variables that make up a (basic) car """
        ## some static constants:
        wheelbase = 0.25 #meters
        chassis_length = 0.37 #meters
        chassis_width = 0.20 #meters
        chassis_length_offset = 0.03 #(meters) car center + this = chassis center
        maxSteeringAngle = np.deg2rad(25) #the car can't steer harder than this, (and will not accept serial commands outside this range)
        
        def __init__(self):
            self.position = np.array([0.0, 0.0], dtype=np.float32)
            self.angle = 0.0 #car orientation in radians
            self.velocity = 0.0 #measured and filtered car 'forward' (wheel) speed in meters/second (used to update position)
            self.steering = 0.0 #measured and filtered steering angle in radians (used to update position)
            
            self.desired_velocity = 0.0 #desired velocity in m/s (sent to car MCU)
            self.desired_steering = 0.0 #desired steering angle in radians (sent to car MCU)
            
            self.totalDistTraveled = 0.0
            self.lastFeedbackTimestamp = 0.0
            
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
        
        # def getRearAxlePos(self): #i changed the car's main position to be at the rear-axle, so this function is obsolete. Use getChassisCenterPos() if you still want the chassis center (old way)
        #     return(GF.distAnglePosToPos(self.wheelbase/2, GF.radInv(self.angle), self.position))
        
        def getChassisCenterPos(self):
            return(GF.distAnglePosToPos((self.wheelbase/2) + self.chassis_length_offset, self.angle, self.position))
        
        #def update() was moved to simCar and realCar, because it should be replaced by SLAM (and/or only used as a quick update between (slower) SLAM updates
        
        ##TBD, i changed the origin to be at rear-axle, so this function needs to be slightly fixed
        # def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the NEAREST surface of the car
        #     """ returns the shortest possible distance to the car chassis and whether or not the car overlaps the entered pos """
        #     translatedDistance = GF.vectorProjectDist(self.position, pos, self.angle)
        #     #self.debugLines.append([0, self.realToPixelPos(self.position), self.realToPixelPos(pos), 0])
        #     chassisBackLength = (-self.chassis_length/2)+self.chassis_length_offset #(negative number) length from middle of car to back-end of chassis (bumper)
        #     chassisFrontLength = (self.chassis_length/2)+self.chassis_length_offset #(positive number) length from middle of car to front-end of chassis (bumper)
        #     if(((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)) and (abs(translatedDistance[1]) < (self.chassis_width/2))): #if pos lies within the car
        #         return(True, 0) #return(yes pos overlaps car, 0 distance to car)
        #     else:
        #         returnDistance = 0
        #         if((translatedDistance[0] > chassisBackLength) and (translatedDistance[0] < chassisFrontLength)): #if it's beside the car
        #             returnDistance = abs(translatedDistance[1]) - (self.chassis_width/2) #shortest dist is a line perp. from the side of the car
        #             # debugAngle = (GF.radInv(self.angle) if (translatedDistance[0] < 0) else self.angle)
        #             # self.debugLines.append([1, self.realToPixelPos(pos), [returnDistance, GF.distAngleBetwPos(pos, [self.position[0] + abs(translatedDistance[0])*np.cos(debugAngle), self.position[1] + abs(translatedDistance[0])*np.sin(debugAngle)])[1]], 1])
        #         elif(abs(translatedDistance[1]) < (self.chassis_width/2)): #if it's in front of or behind the car
        #             if(translatedDistance[0] > 0): #in front of the car
        #                 returnDistance = translatedDistance[0] - chassisFrontLength #shortest dist is a line perp. from the front of the car
        #             else: #behind the car
        #                 returnDistance = abs(translatedDistance[0] - chassisBackLength) #shortest dist is a line perp. from the front of the car
        #             # debugAngle = self.angle + ((-np.pi/2) if (translatedDistance[1] < 0) else (np.pi/2))
        #             # self.debugLines.append([1, self.realToPixelPos(pos), [returnDistance, GF.distAngleBetwPos(pos, [self.position[0] + abs(translatedDistance[1])*np.cos(debugAngle), self.position[1] + abs(translatedDistance[1])*np.sin(debugAngle)])[1]], 1])
        #         else:
        #             tempAngle = np.arctan2(abs(translatedDistance[1]) - (self.chassis_width/2), abs((translatedDistance[0] - chassisBackLength) if (translatedDistance[0]<0) else (translatedDistance[0] - chassisFrontLength)))
        #             returnDistance = abs(translatedDistance[1]/np.sin(tempAngle))  #soh  #note: this should never divide by 0, but if it does, change the '<' from previous two if-statements to a '<='
        #             #returnDistance = translatedDistance[0]/np.cos(tempAngle)  #cah
        #             ## debug:
        #             # debugOffsets = [[np.cos(self.pointAngle+self.angle) * self.pointRadius, np.sin(self.pointAngle+self.angle) * self.pointRadius],
        #             #         [np.cos(np.pi-self.pointAngle+self.angle) * self.pointRadius, np.sin(np.pi-self.pointAngle+self.angle) * self.pointRadius]]
        #             # debugPos = [(self.position[i] + (-1 if (translatedDistance[1]<0) else 1)*debugOffsets[(0 if (((translatedDistance[0]<0) and (translatedDistance[1]<0)) or ((translatedDistance[0]>0) and (translatedDistance[1]>0))) else 1)][i])  for i in range(2)]
        #             # self.debugLines.append([0, self.realToPixelPos(pos), self.realToPixelPos(debugPos), 2])
        #         return(False, returnDistance) #pos doesnt overlap car

    class Cone:
        """ a small class to hold all pertinent information about boundry cones (like position, left-or-right-ness, whether it's part of the finish line, etc) """
        coneDiam = 0.14 #cone diameter in meters (constant)
        def __init__(self, coneID=-1, pos=[0,0], leftOrRight=False, isFinish=False):
            self.ID = coneID  #TO BE REPLACED BY PANDAS INDEXING
            self.position = np.array([pos[0], pos[1]], dtype=np.float32)
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is. True=right, False=left
            self.isFinish = isFinish
            
            self.connections = [] #appendable list, max 2 entries. will contain pointers to cones if succesfully connected
            
            self.coneConData = [] #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def __repr__(self): #print cone objects in a legible fashion
            return("Cone(ID="+str(self.ID)+","+("RIGHT" if self.LorR else "LEFT")+",pos=["+str(round(self.position[0],2))+','+str(round(self.position[1],2))+"],{"+(",".join([str(conn.ID) for conn in self.connections]))+("},FINISH)" if self.isFinish else "})"))

    class Target:
        """ a single point in the path of the car (a target to aim for) """
        def __init__(self, pos=[0,0]):
            #the targets should probably be just be ordered in the list to match their order in the track (shouldnt take much/any shuffling to get done), but if not: use pandas indexing?
            self.position = np.array([pos[0], pos[1]], dtype=np.float32)
            #self.isFinish = isFinish #i added this in a previous version, but never actually made use of it (since the 0th target in the list is usually also the finish), but this might be reinstated later on
            
            self.passed = 0 #counts the number of times it's been passed (from path-planning)
            
            self.coneConData = None #extra data specifically for cone-connection
            self.pathFolData = None #extra data specifically for path-planning
            self.slamData = None    #extra data specifically for SLAM
        
        def __repr__(self): #print target objects in a legible fashion
            return("Target(pos=["+str(round(self.position[0],2))+','+str(round(self.position[1],2))+"],passed="+str(self.passed)+")")
    
    def internalClock(self): #a function that is passed to Map.clock by default
        return(time.time() - self.clockStart)
    
    def clockSet(self, clockFunction): #set a different (simulation) function for self.clock, and insert a pointer to the self as a default argument (only used if function HAS an argument, if this is an issue, avoid usage of setClock() and edit self.clock manually)
        print("WARNING: Map.clockSet() will be removed in future versions, as this method results in the map clock being reset every time a new Map object is created (anywhere)")
        self.clock = clockFunction
        self.clock.__defaults__ = (self.clockStart, ) #insert the starting time as the default argument
    
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
    
    #@property
    def maxConeID(self):
        return(GF.findMaxAttrIndex((self.right_cone_list + self.left_cone_list), 'ID')[1])
    
    def addCone(self, pos, leftOrRight: bool, isFinish=False):
        """add a new cone to the map"""
        overlaps, overlappingCone = self.overlapConeCheck(pos)
        if(overlaps):
            print("cant addConeObj(), there's already a cone there with ID:", overlappingCone.ID)
            return(False, overlappingCone)
        if((len(self.finish_line_cones) >= 2) and isFinish):
            print("addCone warning: there's already 2 finish cones. adding as non-finish cone...")
            isFinish = False
        aNewCone = self.Cone(self.maxConeID()+1, pos, leftOrRight, bool(isFinish)) #bool is just to make sure it's not a 0/1 int
        (self.right_cone_list if leftOrRight else self.left_cone_list).append(aNewCone)
        if(isFinish):
            self.finish_line_cones.append(aNewCone)
        return(True, aNewCone)
    
    def addConeObj(self, coneObj: Cone):
        """add a new cone (object) to the map"""
        overlaps, overlappingCone = self.overlapConeCheck(coneObj.position)
        if(overlaps):
            print("cant addConeObj(), there's already a cone there with ID:", overlappingCone.ID)
            return(False, overlappingCone)
        maxConeID = self.maxConeID()
        if((coneObj.ID <= maxConeID) and ((len(self.left_cone_list)+len(self.right_cone_list))>0)):
            print("addConeObj warning, ID (", coneObj.ID ,") too low! changing it to:", maxConeID+1)
            coneObj.ID = maxConeID+1
        if(coneObj.isFinish and ((len(self.finish_line_cones)>=2) or ((self.finish_line_cones[0].LorR == coneObj.LorR) if (len(self.finish_line_cones)==1) else False))):
            print("addConeObj warning: there's already 2 finish cones. adding as non-finish cone...")
            coneObj.isFinish = False
        (self.right_cone_list if coneObj.LorR else self.left_cone_list).append(coneObj)
        if(coneObj.isFinish):
            self.finish_line_cones.append(coneObj)
        return(True, coneObj)
    
    def removeCone(self, ID: int, LorRhint=None): #LorRhint is either a boolean (indicating which conelist to search) or None, at which point both lists will be searched
        """remove a cone, given the cone's ID"""
        listToSearch = []; coneIndexInList=-1 #init vars (not needed, but it's clean)
        if((LorRhint is not None) and (type(LorRhint) is bool)):
            listToSearch = (self.right_cone_list if LorRhint else self.left_cone_list)
        else:
            listToSearch = (self.left_cone_list + self.right_cone_list)
        coneIndexInList = GF.findIndexByClassAttr(listToSearch,'ID', ID)
        if(coneIndexInList < 0):
            print("removeCone failed, couldn't find ID in list (", (("right_cone_list" if LorRhint else "left_cone_list") if ((LorRhint is not None) and (type(LorRhint) is bool)) else "both"),")")
            return(False)
        coneToDelete = listToSearch[coneIndexInList]
        if((LorRhint != coneToDelete.LorR) if ((LorRhint is not None) and (type(LorRhint) is bool)) else ((coneIndexInList >= len(self.left_cone_list)) != coneToDelete.LorR)): #if the cone.LorR value doesnt match the list in which it was found, something went very wrong earlier!
            print("removeCone WARNING: LorR list mismatch:", coneToDelete.LorR)
        
        if(not ((LorRhint is not None) and (type(LorRhint) is bool))): #if both lists were searched, select the list (to delete from)
            listToSearch = (self.right_cone_list if (coneIndexInList >= len(self.left_cone_list)) else self.left_cone_list)
            coneIndexInList = ((coneIndexInList-len(self.left_cone_list)) if (coneIndexInList >= len(self.left_cone_list)) else coneIndexInList)
        ##first, cleanly sever all ties to other things in the system
        if(coneToDelete.isFinish):
            finishConeIndex = GF.findIndexByClassAttr(self.finish_line_cones,'ID', ID) #(safely) try to find the cone in the finish_line_cones
            if(finishConeIndex < 0): #if the cone thinks it's a finish cone, but it's not in the finish_line_cones, something went very wrong earlier!
                print("removeCone WARNING: cone isFinish flag set, but not found in finish_line_cones!")
            else:
                self.finish_line_cones.pop(finishConeIndex)
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

