#TBD: consider the length of a track-boundry in connectCone() (longer = better), with exceptions for full-circle chains

#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)



import numpy as np  #general math library

#from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use


class coneConnection: #a class to go in Map.Cone.coneConData. This carries some extra data which is only used by the coneConnecter functions
    """some data to go in .coneConData of Map.Cone objects"""
    def __init__(self, angle=0, dist=0, strength=0):
        #self.cone = conePointer #(pointer to) connected cone (ALREADY IN Cone CLASS UNDER Cone.connections)
        self.angle = angle #angle between cones (always from the perspective of the cone that holds this data)
        self.dist = dist #distance between cones
        self.strength = strength #connection strength (highest strength option was used)



class coneConnecter():
    """some functions (& constants) to connect cones to form a track boundry 
        (should not be copied along with Map objects)"""
    def __init__(self, mapToUse=None):
        
        self.mapToUse = self
        if(mapToUse is not None):
            self.mapToUse = mapToUse
        
        self.coneConnectionThreshold = 1.5  #in meters (or at least not pixels)  note: hard threshold beyond which cones will NOT come into contention for connection
        self.coneConnectionThresholdSquared = self.coneConnectionThreshold**2
        ## self.coneConnectionLowChainLen = 3 #if the connection length of coneToConnect is higher than this, devalue the length of the pospect cone's connections #NOT IMPLEMENTED (YET?)
        self.coneConnectionHighChainLen = 5 #the longer the sequence, the better, but any longer than this is just as good as this (strength saturation). (not hard threshold)
        self.coneConnectionHighAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.coneConnectionMaxAngleDelta = np.deg2rad(120) #if the angle difference is larger than this, it just doesnt make sense to connect them. (this IS a hard threshold)
        self.coneConnectionRestrictiveAngleChangeThreshold = np.deg2rad(20) # the most-restrictive-angle code only switches if angle is this much more restrictive
        self.coneConnectionRestrictiveAngleStrengthThreshold = 0.5 # the strength of the more restrictive cone needs to be at least this proportion of the old (less restrictive angle) strength
        
        #self.leftConesFullCircle = False  #TBD: no checking function yet, and because connectCone() can connect a cone to the 0th cone at any time without completing the circle, a special function this is required
        #self.rightConesFullCircle = False
    
    
    def connectCone(self, coneToConnect, applyResult=True): #attempt to connect a given cone
        """attempt to connect a cone to a suitable cone. selection is based on distance, angle and several other parameters"""
        #the correct cone should be selected based on a number of parameters:
        #distance to last cone, angle difference from last and second-to-last cones's, angle that 'track' is (presumably) going based on cones on other side (left/right) (if right cones make corner, left cones must also), etc
        # ideas: distance between last (existing) cone connection may be similar to current cone distance (human may place cones in inner corners close together and outer corners far apart, but at least consistent)
        # ideas: vincent's "most restrictive" angle could be done by realizing the a right corner (CW) is restrictive for left-cones, and a left corner (CCW) is restrictive for right-cones, so whether the angle delta is positive or negative (and leftOrRight boolean) could determine strength
        #more parameters to be added later, in non-simulation code
        currentConnectionCount = len(coneToConnect.connections)
        if(currentConnectionCount >= 2):#if cone is already doubly connected
            print("input cone already doubly connected:")
            return(False, None)
        else:
            currentExistingAngle = 0.0
            if((currentConnectionCount>0) and (len(coneToConnect.coneConData)>0)): #only 1 of the 2 checks should be needed, but just to be safe
                currentExistingAngle = GF.radRoll(GF.radInv(coneToConnect.coneConData[0].angle))
            
            nearbyConeList = self.mapToUse.distanceToCone(coneToConnect.position, self.mapToUse.right_cone_list if coneToConnect.LorR else self.mapToUse.left_cone_list, 'SORTBY_DIST', [coneToConnect.ID], self.coneConnectionThreshold, 'EXCL_DUBL_CONN', [coneToConnect.ID])  #note: list is sorted by distance, but that's not really needed given the (CURRENT) math
            # nearbyConeList structure: [[cone, [dist, angle]], ]
            if(len(nearbyConeList) < 1):
                print("nearbyConeList empty")
                return(False, None)
            bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
            candidateList = []
            for i in range(len(nearbyConeList)):
                cone = nearbyConeList[i][0] #not strictly needed, just for legibility
                connectionCount = len(cone.connections)
                coneCandidateStrength = 1 #init var
                coneCandidateStrength *= 1.5-(nearbyConeList[i][1][0]/self.coneConnectionThreshold)  #high distance, low strength. Linear. worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
                chainLenResult = self.mapToUse.getConeChainLen(cone)
                # if(chainLenResult[1].ID == coneToConnect.ID):
                #     print("chain potential circle:",coneToConnect,cone) #debugging
                coneCandidateStrength *= 0.5+min(chainLenResult[0]/self.coneConnectionHighChainLen, 1) #long chain, high strength. linear.
                angleToCone = nearbyConeList[i][1][1]
                #hard no's: if the angle difference is above the max (like 135 degrees), the prospect cone is just too damn weird, just dont connect to this one
                #note: this can be partially achieved by using angleThreshRange in distanceToCone() to preventatively discard cones like  angleThreshRange=([currentExistingAngle - self.coneConnectionMaxAngleDelta, currentExistingAngle + self.coneConnectionMaxAngleDelta] if (currentConnectionsFilled[0] or currentConnectionsFilled[1]) else [])
                if(((abs(GF.radDiff(angleToCone, cone.coneConData[0].angle)) > self.coneConnectionMaxAngleDelta) if (connectionCount > 0) else False) or ((abs(GF.radDiff(angleToCone, currentExistingAngle)) > self.coneConnectionMaxAngleDelta) if (currentConnectionCount > 0) else False)):
                    #print("very large angle delta")
                    candidatesDiscarded += 1
                else:
                    if(connectionCount > 0): #if cone already has a connection, check the angle delta
                        coneExistingAngle = cone.coneConData[0].angle
                        coneCandidateStrength *= 1.5- min(abs(GF.radDiff(coneExistingAngle, angleToCone))/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                    if(currentConnectionCount > 0):
                        angleDif = GF.radDiff(currentExistingAngle, angleToCone) #radians to add to currentExistingAngle to get to angleToCone (abs value is most interesting)
                        coneCandidateStrength *= 1.5- min(abs(angleDif)/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                        ## (Vincent's) most-restrictive angle could be implemented here, or at the end, by using SORTBY_ANGL_DELT and scrolling through the list from bestCandidateIndex to one of the ends of the list (based on left/right-edness), however, this does require a previous connection (preferably a connection that is in, or leads to, the target_list) to get angleDeltaTarget
                        coneCandidateStrength *= 1 + (0.5 if coneToConnect.LorR else -0.5)*max(min(angleDif/self.coneConnectionHighAngleDelta, 1), -1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                        ## if most-restrictive angle is applied at the end, when all candidates have been reviewed
                        candidateList.append([i, angleDif, coneCandidateStrength])
                    #if(idk yet
                    if(coneCandidateStrength > highestStrength):
                        highestStrength = coneCandidateStrength
                        bestCandidateIndex = i
            if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
                print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
                return(False, None)
            ## success!
            ## most restrictive angle check
            if(currentConnectionCount > 0): #most restrictive angle can only be applied if there is a reference angle (you could also check len(candidateList)
                bestCandidateListIndex = GF.findIndexBy2DEntry(candidateList, 0, bestCandidateIndex) #this index is of candidateList, not of nearbyConeList
                for i in range(len(candidateList)):
                    if((candidateList[i][1] > candidateList[bestCandidateListIndex][1]) if coneToConnect.LorR else (candidateList[i][1] < candidateList[bestCandidateListIndex][1])): #most restrictive angle is lowest angle
                        print("more restrictive angle found:", candidateList[i][1], "is better than", candidateList[bestCandidateListIndex][1], "LorR:", coneToConnect.LorR)
                        #if((abs(radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > self.coneConnectionRestrictiveAngleChangeThreshold) and ((candidateList[i][2]/highestStrength) > self.coneConnectionRestrictiveAngleStrengthThreshold)):
                        if(abs(GF.radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > self.coneConnectionRestrictiveAngleChangeThreshold):
                            print("old highestStrength:", round(highestStrength, 2), " new highestStrength:", round(candidateList[i][2], 2), " ratio:", round(candidateList[i][2]/highestStrength, 2))
                            bestCandidateListIndex = i
                            bestCandidateIndex = candidateList[i][0]
                            highestStrength = candidateList[i][2]
            #print("cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
            ## make the connection:
            winningCone = nearbyConeList[bestCandidateIndex][0] #pointer to a Cone class object
            if(applyResult): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                ## input cone
                coneToConnect.connections.append(winningCone) #save cone pointer in the .connections list
                coneToConnect.coneConData.append(coneConnection(nearbyConeList[bestCandidateIndex][1][1], nearbyConeList[bestCandidateIndex][1][0], highestStrength))
                ## and the other cone
                winningCone.connections.append(coneToConnect)
                winningCone.coneConData.append(coneConnection(GF.radInv(nearbyConeList[bestCandidateIndex][1][1]), nearbyConeList[bestCandidateIndex][1][0], highestStrength))
            return(True, winningCone) #return the cone you connected with (or are capable of connecting with, if applyResult=False)
    
    def connectConeSuperSimple(self, coneToConnect, applyResult=True):
        """attempt to (quickly) connect a cone to a suitable cone. selection is based on (squared) distance and a few other parameters"""
        currentConnectionCount = len(coneToConnect.connections)
        if(currentConnectionCount >= 2):#if cone is already doubly connected
            print("input cone already doubly connected?!:", coneToConnect.coneConData)
            return(False, None)
        else:
            nearbyConeList = self.mapToUse.distanceToConeSquared(coneToConnect.position, self.mapToUse.right_cone_list if coneToConnect.LorR else self.mapToUse.left_cone_list, True, [coneToConnect.ID], self.coneConnectionThresholdSquared, 'EXCL_DUBL_CONN', [coneToConnect.ID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
            if(len(nearbyConeList) < 1):
                print("nearbyConeList empty")
                return(False, None)
            bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
            for i in range(len(nearbyConeList)):
                coneCandidateStrength = 1 #init var
                coneCandidateStrength *= 1.5-(nearbyConeList[i][1]/self.coneConnectionThresholdSquared)  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
                coneCandidateStrength *= 0.5+min(self.mapToUse.getConeChainLen(nearbyConeList[i][0])[0]/self.coneConnectionHighChainLen, 1) #long chain, high strength. linear.
                ## no angle math can be done, as Pythagoras's ABC is used, not sohcahtoa :)
                if(coneCandidateStrength > highestStrength):
                    highestStrength = coneCandidateStrength
                    bestCandidateIndex = i
            if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
                print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
                return(False, None)
            ## success!
            #print("simple cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
            ## make the connection:
            winningCone = nearbyConeList[bestCandidateIndex][0] #pointer to a Cone class object
            if(applyResult): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                dist, angle = GF.distAngleBetwPos(coneToConnect.position, winningCone.position)
                ## input cone
                coneToConnect.connections.append(winningCone) #save cone pointer in the .connections list
                coneToConnect.coneConData.append(coneConnection(angle, dist, highestStrength))
                ## and the other cone
                winningCone.connections.append(coneToConnect)
                winningCone.coneConData.append(coneConnection(GF.radInv(angle), dist, highestStrength))
            return(True, winningCone) #return the cone you connected with (or are capable of connecting with, if applyResult=False)
