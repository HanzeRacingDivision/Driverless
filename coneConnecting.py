#TBD: consider the length of a track-boundry in connectCone() (longer = better), with exceptions for full-circle chains

#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)



import numpy as np
from pyparsing import dict_of  #general math library

from Map import Map # (currently) only used to help with syntax coloring
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use


class coneConnection: #a class to go in Map.Cone.coneConData. This carries some extra data which is only used by the coneConnecter functions
    """some data to go in .coneConData of Map.Cone objects"""
    def __init__(self, angle=0, dist=0, strength=0):
        #self.cone = conePointer #(pointer to) connected cone (ALREADY IN Cone CLASS UNDER Cone.connections)
        self.angle = angle #angle between cones (always from the perspective of the cone that holds this data)
        self.dist = dist #distance between cones
        self.strength = strength #connection strength (highest strength option was used)



########################### some (static!!!) functions ####################################

def _applyConeConnection(coneToConnect: Map.Cone, winningCone: Map.Cone, strengthVal, distAngle: list[float,float]=None):
    if(distAngle is None):
        distAngle = GF.distAngleBetwPos(coneToConnect.position, winningCone.position)
    ## input cone
    coneToConnect.connections.append(winningCone) #save cone pointer in the .connections list
    coneToConnect.coneConData.append(coneConnection(distAngle[1], distAngle[0], strengthVal))
    ## and the other cone
    winningCone.connections.append(coneToConnect)
    winningCone.coneConData.append(coneConnection(GF.radInv(distAngle[1]), distAngle[0], strengthVal))



def connectCone(mapToUse: Map, coneToConnect: Map.Cone, applyResult=True, printDebug=True): #attempt to connect a given cone
    """attempt to connect a cone to a suitable cone. selection is based on distance, angle and several other parameters"""
    #the correct cone should be selected based on a number of parameters:
    #distance to last cone, angle difference from last and second-to-last cones's, angle that 'track' is (presumably) going based on cones on other side (left/right) (if right cones make corner, left cones must also), etc
    # ideas: distance between last (existing) cone connection may be similar to current cone distance (human may place cones in inner corners close together and outer corners far apart, but at least consistent)
    # ideas: vincent's "most restrictive" angle could be done by realizing the a right corner (CW) is restrictive for left-cones, and a left corner (CCW) is restrictive for right-cones, so whether the angle delta is positive or negative (and leftOrRight boolean) could determine strength
    #more parameters to be added later, in non-simulation code
    currentConnectionCount = len(coneToConnect.connections)
    if(currentConnectionCount >= 2):#if cone is already doubly connected
        if(printDebug):
            print("input cone already doubly connected:")
        return(False, None)
    else:
        currentExistingAngle = 0.0
        if((currentConnectionCount>0) and (len(coneToConnect.coneConData)>0)): #only 1 of the 2 checks should be needed, but just to be safe
            currentExistingAngle = GF.radRoll(GF.radInv(coneToConnect.coneConData[0].angle))
        
        nearbyConeList = mapToUse.distanceToCone(coneToConnect.position, mapToUse.cone_lists[coneToConnect.LorR], 'SORTBY_DIST', [coneToConnect.ID], coneConnecter.maxConnectionDist, 'EXCL_DUBL_CONN', [coneToConnect.ID])  #note: list is sorted by distance, but that's not really needed given the (CURRENT) math
        # nearbyConeList structure: [[cone, [dist, angle]], ]
        if(len(nearbyConeList) < 1):
            if(printDebug):
                print("nearbyConeList empty")
            return(False, None)
        bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
        candidateList = []
        for i in range(len(nearbyConeList)):
            cone = nearbyConeList[i][0] #not strictly needed, just for legibility
            connectionCount = len(cone.connections)
            coneCandidateStrength = 1 #init var
            coneCandidateStrength *= 1.5-(nearbyConeList[i][1][0]/coneConnecter.maxConnectionDist)  #high distance, low strength. Linear. worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
            chainLenResult = mapToUse.getConeChainLen(cone)
            # if(chainLenResult[1].ID == coneToConnect.ID):
            #     print("chain potential circle:",coneToConnect,cone) #debugging
            coneCandidateStrength *= 0.5+min(chainLenResult[0]/coneConnecter.coneConnectionHighChainLen, 1) #long chain, high strength. linear.
            angleToCone = nearbyConeList[i][1][1]
            #hard no's: if the angle difference is above the max (like 135 degrees), the prospect cone is just too damn weird, just dont connect to this one
            #note: this can be partially achieved by using angleThreshRange in distanceToCone() to preventatively discard cones like  angleThreshRange=([currentExistingAngle - coneConnecter.coneConnectionMaxAngleDelta, currentExistingAngle + coneConnecter.coneConnectionMaxAngleDelta] if (currentConnectionsFilled[0] or currentConnectionsFilled[1]) else [])
            if(((abs(GF.radDiff(angleToCone, cone.coneConData[0].angle)) > coneConnecter.coneConnectionMaxAngleDelta) if (connectionCount > 0) else False) or ((abs(GF.radDiff(angleToCone, currentExistingAngle)) > coneConnecter.coneConnectionMaxAngleDelta) if (currentConnectionCount > 0) else False)):
                #print("very large angle delta")
                candidatesDiscarded += 1
            else:
                if(connectionCount > 0): #if cone already has a connection, check the angle delta
                    coneExistingAngle = cone.coneConData[0].angle
                    coneCandidateStrength *= 1.5- min(abs(GF.radDiff(coneExistingAngle, angleToCone))/coneConnecter.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                if(currentConnectionCount > 0):
                    angleDif = GF.radDiff(currentExistingAngle, angleToCone) #radians to add to currentExistingAngle to get to angleToCone (abs value is most interesting)
                    coneCandidateStrength *= 1.5- min(abs(angleDif)/coneConnecter.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                    ## (Vincent's) most-restrictive angle could be implemented here, or at the end, by using SORTBY_ANGL_DELT and scrolling through the list from bestCandidateIndex to one of the ends of the list (based on left/right-edness), however, this does require a previous connection (preferably a connection that is in, or leads to, the target_list) to get angleDeltaTarget
                    coneCandidateStrength *= 1 + (0.5 if coneToConnect.LorR else -0.5)*max(min(angleDif/coneConnecter.coneConnectionHighAngleDelta, 1), -1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                    ## if most-restrictive angle is applied at the end, when all candidates have been reviewed
                    candidateList.append([i, angleDif, coneCandidateStrength])
                #if(idk yet
                if(coneCandidateStrength > highestStrength):
                    highestStrength = coneCandidateStrength
                    bestCandidateIndex = i
        if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
            if(printDebug):
                print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
            return(False, None)
        ## success!
        ## most restrictive angle check
        if(currentConnectionCount > 0): #most restrictive angle can only be applied if there is a reference angle (you could also check len(candidateList)
            bestCandidateListIndex = GF.findIndexBy2DEntry(candidateList, 0, bestCandidateIndex) #this index is of candidateList, not of nearbyConeList
            for i in range(len(candidateList)):
                if((candidateList[i][1] > candidateList[bestCandidateListIndex][1]) if coneToConnect.LorR else (candidateList[i][1] < candidateList[bestCandidateListIndex][1])): #most restrictive angle is lowest angle
                    if(printDebug):
                        print("more restrictive angle found:", candidateList[i][1], "is better than", candidateList[bestCandidateListIndex][1], "LorR:", coneToConnect.LorR)
                    #if((abs(radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > coneConnecter.coneConnectionRestrictiveAngleChangeThreshold) and ((candidateList[i][2]/highestStrength) > coneConnecter.coneConnectionRestrictiveAngleStrengthThreshold)):
                    if(abs(GF.radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > coneConnecter.coneConnectionRestrictiveAngleChangeThreshold):
                        if(printDebug):
                            print("old highestStrength:", round(highestStrength, 2), " new highestStrength:", round(candidateList[i][2], 2), " ratio:", round(candidateList[i][2]/highestStrength, 2))
                        bestCandidateListIndex = i
                        bestCandidateIndex = candidateList[i][0]
                        highestStrength = candidateList[i][2]
        #if(printDebug):
        #    print("cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
        ## make the connection:
        winningCone = nearbyConeList[bestCandidateIndex][0] #pointer to a Cone class object
        if(applyResult): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
            _applyConeConnection(coneToConnect, winningCone, highestStrength, nearbyConeList[bestCandidateIndex][1])
        return(True, winningCone) #return the cone you connected with (or are capable of connecting with, if applyResult=False)

def connectConeSuperSimple(mapToUse: Map, coneToConnect: Map.Cone, applyResult=True, printDebug=True):
    """attempt to (quickly) connect a cone to a suitable cone. selection is based on (squared) distance and a few other parameters"""
    currentConnectionCount = len(coneToConnect.connections)
    if(currentConnectionCount >= 2):#if cone is already doubly connected
        if(printDebug):
            print("input cone already doubly connected?!:", coneToConnect.coneConData)
        return(False, None)
    else:
        nearbyConeList = mapToUse.distanceToConeSquared(coneToConnect.position, mapToUse.cone_lists[coneToConnect.LorR], True, [coneToConnect.ID], coneConnecter.maxConnectionDist**2, 'EXCL_DUBL_CONN', [coneToConnect.ID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
        if(len(nearbyConeList) < 1):
            if(printDebug):
                print("nearbyConeList empty")
            return(False, None)
        bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
        for i in range(len(nearbyConeList)):
            coneCandidateStrength = 1 #init var
            coneCandidateStrength *= 1.5-(nearbyConeList[i][1]/(coneConnecter.maxConnectionDist**2))  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
            coneCandidateStrength *= 0.5+min(mapToUse.getConeChainLen(nearbyConeList[i][0])[0]/coneConnecter.coneConnectionHighChainLen, 1) #long chain, high strength. linear.
            ## no angle math can be done, as Pythagoras's ABC is used, not sohcahtoa :)
            if(coneCandidateStrength > highestStrength):
                highestStrength = coneCandidateStrength
                bestCandidateIndex = i
        if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
            if(printDebug):
                print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
            return(False, None)
        ## success!
        #if(printDebug):
        #    print("simple cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
        ## make the connection:
        winningCone = nearbyConeList[bestCandidateIndex][0] #pointer to a Cone class object
        if(applyResult): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
            _applyConeConnection(coneToConnect, winningCone, highestStrength, None)
        return(True, winningCone) #return the cone you connected with (or are capable of connecting with, if applyResult=False)

def findFirstCones(mapToUse: Map, onlyOneSide=None):
    """find the first left/right cones"""
    firstCones = {False : None, True :None} # return a left and a right cone
    import pathFinding      as PF # used to retieve pathFirstLinePosDist
    for LorR in mapToUse.cone_lists: # iterates over keys!
        if((LorR != onlyOneSide) if (onlyOneSide is not None) else False):
            continue # skip this loop if the user specified onlyOneSide (and it wasn't this side)
        nearbyConeList = mapToUse.distanceToCone(mapToUse.car.position, mapToUse.cone_lists[LorR], 'SORTBY_DIST', simpleThreshold=PF.pathFinder.pathFirstLinePosDist)
        for cone, distAngle in nearbyConeList:
            # if cone == good ?
            firstCones[LorR] = cone
            break
    return(firstCones)

def connectFirstCones(mapToUse: Map, firstCones: dict[bool,Map.Cone], onlyOneSide=None, makeFinish=False):
    # if((firstCones[False] is None) or (firstCones[True] is None)):
    #     print("cant connectFirstCones(), firstCones are None")
    import pathFinding      as PF # used to retrieve pathFirstLineCarAngleDeltaMax
    connectedCones = {False : None, True : None}
    for LorR in firstCones: # iterates over keys!
        if((LorR != onlyOneSide) if (onlyOneSide is not None) else False):
            continue # skip this loop if the user specified onlyOneSide (and it wasn't this side)
        if(firstCones[LorR] is None):
            continue
        connectSucces, connectedCones[LorR] = connectCone(mapToUse, firstCones[LorR], True, True)
        if(not connectSucces):
            continue
        if(makeFinish): # if the code makes it here, the fistCone was probably good, in which case you might want to use it as a finish line (also makes pathFinding easier...)
            firstCones[LorR].isFinish = True # (note: this pointer cone should also affect the cone in the cone_lists)
        # the boundry needs to be built in the same direction as the car is facing, so make sure that this first connection actually does allows for that
        angleLikeCar = abs(GF.radDiff(mapToUse.car.angle, GF.distAngleBetwPos(firstCones[LorR].position, connectedCones[LorR].position)[1]))
        if(angleLikeCar > PF.pathFinder.pathFirstLineCarAngleDeltaMax):
            # the connection was with a cone on the wrong side, let's just try to connect another cone
            connectSucces, connectedCones[LorR] = connectCone(mapToUse, firstCones[LorR], True, True) # attempt to doubly-connect the fistCone, in hopes of 
            angleLikeCarSecond = abs(GF.radDiff(mapToUse.car.angle, GF.distAngleBetwPos(firstCones[LorR].position, connectedCones[LorR].position)[1]))
            if(not connectSucces):
                print("connectFirstCones() first connection was the wrong direction and there is no second connection.")
            elif(angleLikeCarSecond > PF.pathFinder.pathFirstLineCarAngleDeltaMax):
                print("connectFirstCones() somehow both connections are at bad angles with respect to the car!", angleLikeCar, angleLikeCarSecond)
                if(angleLikeCarSecond > angleLikeCar): # if the second one sucks even more than the first one
                    connectedCones[LorR] = firstCones[LorR].connections[0] # go back to the first one
    return(connectedCones)

class coneConnecter():
    """a static class with constants and (pointers to) functions to connect cones (to form a track boundry).
        The variables that this the functions make use of are stored in the Map object in Map.Cone.coneConData"""
    maxConnectionDist = 5.0  # (meters) hard threshold beyond which cones will NOT come into contention for (same-colored) connection
    ## boundary spacing constants are located in pathFinding.py
    # coneConnectionLowChainLen = 3 #if the connection length of coneToConnect is higher than this, devalue the length of the pospect cone's connections #NOT IMPLEMENTED (YET?)
    coneConnectionHighChainLen = 5 #the longer the sequence, the better, but any longer than this is just as good as this (strength saturation). (not hard threshold)
    coneConnectionHighAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
    coneConnectionMaxAngleDelta = np.deg2rad(80) #if the angle difference is larger than this, it just doesnt make sense to connect them. (this IS a hard threshold)
    coneConnectionRestrictiveAngleChangeThreshold = np.deg2rad(30) # the most-restrictive-angle code only switches if angle is this much more restrictive
    coneConnectionRestrictiveAngleStrengthThreshold = 0.5 # the strength of the more restrictive cone needs to be at least this proportion of the old (less restrictive angle) strength

    findFirstConesDelay = 3.0 # how long to wait before even looking for the first cones (give the sensor data some time to come in)
    findFirstConesInterval = 1.0 # how long inbetween findFirstCones() attempts
    defaultConeConnectInterval = 0.5 # how long between default boundry-building calls. Note: it will only build 1 cone at-a-time, so don't make this too long, but also not too long (sensor data needs time)
    
    # #this is mostly to keep compatibility with my older versions (where the pathPlanner class is inherited into the map object). I can't recommend that, as the map object is often transmitted to other processes/PCs
    # @staticmethod
    # def connectCone(mapToUse, coneToConnect, applyResult=True, printDebug=True):
    #     return(connectCone(mapToUse, coneToConnect, applyResult, printDebug))
    
    # @staticmethod
    # def connectConeSuperSimple(mapToUse, coneToConnect, applyResult=True, printDebug=True):
    #     return(connectConeSuperSimple(mapToUse, coneToConnect, applyResult, printDebug))
    
