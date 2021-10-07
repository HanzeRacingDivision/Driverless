#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import numpy as np  #general math library

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use




class pathFinderData: #a class to go in Map.Target.coneConData or Map.Target.pathFolData, if Alex approves
    """some data to go in .coneConData of Map.Target objects (TBD: put in .pathFolData?)"""
    def __init__(self, heading=0, track_width=0, cones=[None, None], strength=0):
        #self.position = np.array([pos[0], pos[1]])
        self.heading = heading #angle the car should (roughly) face when crossing this target
        self.track_width = track_width #width of the track at that point (target is a point between 2 cones, so width is NOT very accurate if the cones are not on exactly opposite sides (shifted forw/backw))
        self.cones = [cones[0], cones[1]]
        self.strength = strength #path-finding strength (highest strength option was used)


class pathFinder():
    """some functions (& constants) to find Target points (target_list) using (track boundry) cones 
        (should not be copied along with Map objects)"""
    def __init__(self, mapToUse=None):
        
        self.mapToUse = self
        if(mapToUse is not None):
            self.mapToUse = mapToUse
        
        self.pathConnectionThreshold = 3 #in meters (or at least not pixels)  IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.pathConnectionMaxAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        
        self.pathFirstLineCarAngleDeltaMax = np.deg2rad(45) #if the radDiff() between car (.angle) and the first line's connections is bigger than this, switch conneections or stop
        self.pathFirstLineCarSideAngleDelta = np.deg2rad(80) #left cones should be within +- pathFirstLineCarSideAngleDelta radians of the side of the car (asin, car.angle + or - pi/2, depending on left/right side)
        self.pathFirstLinePosDist = 2 # simple center to center distance, hard threshold, used to filter out very far cones
        self.pathFirstLineCarDist = 1 # real distance, not hard threshold, just distance at which lowest strength-score is given
        
    
    def makePath(self):
        """find Target points between connected cones (track boundry),
            the first Target is placed on the finish lines (if available) or near the car"""
        # target_list content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        # left/right-ConeList content: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]
        if(self.mapToUse.targets_full_circle):
            print("not gonna make path, already full circle")
            return(False)
        if(len(self.mapToUse.target_list) == 0): #if there is no existing path to go on
            if((len(self.mapToUse.right_cone_list) < 2) or (len(self.mapToUse.left_cone_list) < 2)): #first pathLine can only be made between 2 connected cones
                print("not enough cones in one or more coneLists, cant place first pathLine")
                return(False)
            
            #search cones here
            firstCones = [None, None]
            strengths = [0.0, 0.0]
            if(len(self.mapToUse.finish_line_cones) > 0):
                print("using finish cone(s) to make first pathLine")
                if(self.mapToUse.finish_line_cones[0].LorR):
                    firstCones[1] = self.mapToUse.finish_line_cones[0]
                else:
                    firstCones[0] = self.mapToUse.finish_line_cones[0]
                if(len(self.mapToUse.finish_line_cones) > 1): #if both finish cones are set
                    if(self.mapToUse.finish_line_cones[1].LorR):
                        firstCones[1] = self.mapToUse.finish_line_cones[1]
                    else:
                        firstCones[0] = self.mapToUse.finish_line_cones[1]
            for LorR in range(2):
                if(firstCones[LorR] is None):
                    bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
                    sideAngleRange = [self.mapToUse.car.angle +((-np.pi/2) if LorR else (np.pi/2)) -self.pathFirstLineCarSideAngleDelta, self.mapToUse.car.angle +((-np.pi/2) if LorR else (np.pi/2)) +self.pathFirstLineCarSideAngleDelta] #left side is car.angle +pi/2, right side is car.angle -pi/2
                    firstConeCandidates = self.mapToUse.distanceToCone(self.mapToUse.car.position, self.mapToUse.right_cone_list if LorR else self.mapToUse.left_cone_list, 'SORTBY_DIST', [], self.pathFirstLinePosDist, 'EXCL_UNCONN', [], self.mapToUse.car.angle, sideAngleRange) #sorting is unnecessarry
                    for i in range(len(firstConeCandidates)):
                        cone = firstConeCandidates[i][0]
                        connectionCount = len(cone.connections) #due to the EXCL_UNCONN filtering, this value should never be 0
                        connectionAnglesAllowed = [((abs(GF.radDiff(cone.coneConData[0].angle, self.mapToUse.car.angle)) < self.pathFirstLineCarAngleDeltaMax) if (connectionCount > 0) else False), ((abs(GF.radDiff(cone.coneConData[1].angle, self.mapToUse.car.angle)) < self.pathFirstLineCarAngleDeltaMax) if (connectionCount > 1) else False)]
                        ## it's important to note that the distance calculated by distanceToCone() is between the center of the car and the cone, and therefore not the shortest path, or real distance to the car (a cone next to a wheel will have a higher distance than a cone next to the middle of the car, which is illogical)
                        ## this illogical distance can still be used to help filter out candidates, but for an accurate strength-rating, distanceToCar() (a function of the raceCar class) should be used
                        coneOverlapsCar, distToCar = self.mapToUse.car.distanceToCar(cone.position)
                        #print("evaluating "+("right" if (LorR==1) else "left")+" cone:", cone[0], connectionsFilled, connectionAnglesAllowed, coneOverlapsCar, distToCar)
                        if(connectionCount < 1):
                            ## somehow, an unconnected cone slipped past the filer in distanceToCone(). this should be impossible, but i wrote the filter code in a hurry, so little debugging cant hurt
                            print("impossible filter slip 1")
                            candidatesDiscarded += 1
                        elif(not (connectionAnglesAllowed[0] or connectionAnglesAllowed[1])):
                            ## neither of the connections on this candidate 
                            print("neither connections have acceptable angles")
                            candidatesDiscarded += 1
                        elif(connectionAnglesAllowed[0] and connectionAnglesAllowed[1]):
                            ## somehow, both connections are alligned with the car, and since coneConnectionMaxAngleDelta exists, that should be impossible
                            print("impossible angle sitch 1")
                            candidatesDiscarded += 1
                        elif(coneOverlapsCar):
                            print("cone overlaps car")
                            candidatesDiscarded += 1
                        else:
                            coneCandidateStrength = 1 #init var
                            coneCandidateStrength *= 1.5-min(distToCar/self.pathFirstLineCarDist, 1)  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5
                            coneCandidateStrength *= 1.5-abs(GF.radDiff(cone.coneConData[(0 if connectionAnglesAllowed[0] else 1)].angle, self.mapToUse.car.angle))/self.pathFirstLineCarAngleDeltaMax  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            ## this following check makes sure the pathline is perpendicular to the car
                            coneAvgConnAngle = 0 #init var
                            if(connectionCount >= 2):
                                coneAvgConnAngle = GF.radMidd(GF.radInv(cone.coneConData[0].angle), cone.coneConData[1].angle) #note: radMidd() has inputs (lowBound, upBound), so for right cones this will give an angle that points AWAY from the car, and for left cones it points towards the car (both in the same direction if they're paralel)
                            else:
                                connectionToUse = (1 if (connectionCount >= 2) else 0)
                                coneAvgConnAngle = cone.coneConData[connectionToUse].angle #see note on calculation with double connection above
                            coneCandidateStrength *= 1.5-min(min(abs(GF.radDiff(coneAvgConnAngle, self.mapToUse.car.angle)), abs(GF.radDiff(GF.radInv(coneAvgConnAngle), self.mapToUse.car.angle)))/self.pathConnectionMaxAngleDelta, 1)
                            ## using existing chosen firstCone, only works for the right cone (this is an unequal check, so i hate it, but it does work to make sure the first line is straight (not diagonal))
                            if(LorR == 1): #TO BE IMPROVED, but i dont know quite how yet
                                leftFirstConeConnectionCount = len(firstCones[0].connections) #this could technically be 0, if an unconnected finish cone was used
                                if(leftFirstConeConnectionCount > 0):
                                    leftAvgConnAngle = 0 #init var
                                    if(leftFirstConeConnectionCount >= 2):
                                        leftAvgConnAngle = GF.radMidd(GF.radInv(firstCones[0].coneConData[0].angle), firstCones[0].coneConData[1].angle)
                                    else:
                                        connectionToUse = (1 if (leftFirstConeConnectionCount >= 2) else 0)
                                        leftAvgConnAngle = firstCones[0].coneConData[connectionToUse].angle
                                    tempPathWidth, tempPathAngle = GF.distAngleBetwPos(firstCones[0].position, cone.position)
                                    coneCandidateStrength *= 1.5-min(min(abs(GF.radDiff(leftAvgConnAngle, tempPathAngle)), abs(GF.radDiff(GF.radInv(leftAvgConnAngle), tempPathAngle)))/self.pathConnectionMaxAngleDelta, 1)
                                    #you could also even do distance, but whatever
                            if(coneCandidateStrength > highestStrength):
                                highestStrength = coneCandidateStrength
                                bestCandidateIndex = i
                    if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(firstConeCandidates) == candidatesDiscarded)):
                        print("it seems no suitible candidates for first "+("right" if (LorR==1) else "left")+" cone were found at all... bummer.", len(firstConeCandidates), candidatesDiscarded, bestCandidateIndex, highestStrength)
                        return(False)
                    ## if the code makes it here, a suitable first cone has been selected.
                    #print("first "+("right" if (LorR==1) else "left")+" cone found!", highestStrength, bestCandidateIndex, len(firstConeCandidates), candidatesDiscarded)
                    firstCones[LorR] = firstConeCandidates[bestCandidateIndex][0]
                    strengths[LorR] = highestStrength
            
            ## and now just put the first cones into the target_list
            pathWidth, lineAngle = GF.distAngleBetwPos(firstCones[0].position, firstCones[1].position)
            carAngle = GF.radRoll(lineAngle + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
            centerPoint = [firstCones[1].position[0] + (firstCones[0].position[0]-firstCones[1].position[0])/2, firstCones[1].position[1] + (firstCones[0].position[1]-firstCones[1].position[1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
            #newTarget = Map.Target(centerPoint, (firstCones[0].isFinish or firstCones[1].isFinish))
            newTarget = Map.Target(centerPoint)
            newTarget.coneConData = pathFinderData(carAngle, pathWidth, [firstCones[0], firstCones[1]], max(strengths))
            self.mapToUse.target_list.append(newTarget)
        else: #if len(target_list) > 0
            lastPathLine = self.mapToUse.target_list[-1] # -1 gets the last item in list, you could also use (len(target_list)-1)
            lastCones = [];  lastConeAvgConnAngle = [];  prospectCones = [];
            for LorR in range(2):
                lastCones.append(lastPathLine.coneConData.cones[LorR])
                lastConnectionCount = len(lastCones[LorR].connections)
                if(lastConnectionCount < 1):
                    print("no connections on lastCones["+("right" if (LorR==1) else "left")+"] (impossible):", lastCones[LorR])
                    return(False)
                connectedConesProspectable = [];  prospectConnectionIndex = 0
                for i in range(lastConnectionCount):
                    connectedConesProspectable.append(True)
                    # if(len(self.mapToUse.target_list) > 1):
                    #     if(lastCones[LorR].connections[i].ID == self.mapToUse.target_list[-2].coneConData.cones[LorR].ID): #only check second to last path point
                    #         connectedConesProspectable[i] = False;
                    for target in self.mapToUse.target_list: #look through the entire list of targets (you could also just look at self.mapToUse.target_list[-2])
                        if(lastCones[LorR].connections[i].ID == target.coneConData.cones[LorR].ID): #if the only connected cone is ALSO (already) in the target_list, then you cant make any more path
                            connectedConesProspectable[i] = False;
                if((lastConnectionCount == 1) and (not connectedConesProspectable[0])):
                    print("single lastCones["+("right" if (LorR==1) else "left")+"] connection already in target_list (path at end of cone line, make more connections)")
                    return(False)
                elif(lastConnectionCount >= 2):
                    if(connectedConesProspectable[0] and connectedConesProspectable[1]):
                        ##in this (somewhat rare) scenario, the correct cone needs to be chosen to make sure the path propagates in the same direction as the car drives
                        ##right now i can only think of 2 ways, by considering the orientation of the car (which should(?) be nearby)
                        ## or by considering the number of subsequently connected cones (longer = better)
                        strengths = []
                        angleDeltas = []
                        connectionSeqLengths = []
                        for i in range(lastConnectionCount):
                            angleDeltas.append(abs(GF.radDiff(lastCones[LorR].coneConData[i].angle, self.mapToUse.car.angle)))
                            connectionSeqLengths.append(self.mapToUse.getConeChainLen(lastCones[LorR].connections[i], lastCones[LorR])[0])
                        for i in range(lastConnectionCount):
                            strengths.append(1.0)
                            strengths[i] *= 1.5-min(angleDeltas[i]/max(angleDeltas), 1) #lower delta = better
                            strengths[i] *= 0.5+min(connectionSeqLengths[i]/max(connectionSeqLengths), 1) #longer chain = better
                        prospectConnectionIndex = (1 if (strengths[1] > strengths[0]) else 0)
                        #print("deciding direction of path", prospectConnectionIndex, strengths, angleDeltas, connectionSeqLengths)
                    elif(connectedConesProspectable[0]):
                        prospectConnectionIndex = 0
                    elif(connectedConesProspectable[1]):
                        prospectConnectionIndex = 1
                    else: #both of the cones connected to lastCone are already used in pathList
                        #either something is very wrong, OR the path is about to go full-circle
                        if(lastCones[LorR].connections[0].ID == self.mapToUse.target_list[0].coneConData.cones[LorR].ID): #if the 0th connection is the first Target
                            #about to go full-circle, use connections[0] as prospectCone
                            prospectConnectionIndex = 0
                        elif(lastCones[LorR].connections[1].ID == self.mapToUse.target_list[0].coneConData.cones[LorR].ID): #if the 0th connection is the first Target
                            #about to go full-circle, use connections[1] as prospectCone
                            prospectConnectionIndex = 1
                        elif(lastCones[LorR].ID == self.mapToUse.target_list[0].coneConData.cones[LorR].ID): #the prospectcone on this side doesnt matter, it all about this lastCone
                            print("nearing a full circle")
                        else: #something is just very wrong
                            print("both lastCones["+("right" if (LorR==1) else "left")+"].connections already in target_list, something is very wrong")
                            return(False)
                lastConeAvgConnAngle.append((GF.radMidd(GF.radInv(lastCones[LorR].coneConData[0].angle), lastCones[LorR].coneConData[1].angle)) if (lastConnectionCount >= 2) else (lastCones[LorR].coneConData[0].angle))
                #and now the prospect cones
                prospectCones.append(lastCones[LorR].connections[prospectConnectionIndex])
            
            #check if you've gone full circle
            if(((prospectCones[0].ID == self.mapToUse.target_list[0].coneConData.cones[0].ID) and (prospectCones[1].ID == self.mapToUse.target_list[0].coneConData.cones[1].ID)) \
                or ((lastCones[0].ID == self.mapToUse.target_list[0].coneConData.cones[0].ID) and (prospectCones[1].ID == self.mapToUse.target_list[0].coneConData.cones[1].ID)) \
                or ((prospectCones[0].ID == self.mapToUse.target_list[0].coneConData.cones[0].ID) and (lastCones[1].ID == self.mapToUse.target_list[0].coneConData.cones[1].ID))):
                print("path full circle (by default)")
                self.mapToUse.targets_full_circle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            
            prospectConeAvgConnAngle = []
            for LorR in range(2):
                prospectConeConnectionCount = len(prospectCones[LorR].connections)
                if(prospectConeConnectionCount < 1):
                    print("BIG issue: lastCones["+("right" if (LorR==1) else "left")+"] pointed to this prospect cone, but this prospect cone has no connections?!?", lastCones[LorR].connections, prospectCones[LorR].connections)
                elif(not ((prospectCones[LorR].connections[0].ID == lastCones[LorR].ID) or ((prospectCones[LorR].connections[1].ID == lastCones[LorR].ID) if (prospectConeConnectionCount >= 2) else False))):
                    print("BIG issue: lastCones["+("right" if (LorR==1) else "left")+"] pointed to this prospect cone, but this prospect cone does not point back", lastCones[LorR].connections, prospectCones[LorR].connections)
                
                prospectConeAvgConnAngle.append(((GF.radMidd(GF.radInv(prospectCones[LorR].coneConData[0].angle), prospectCones[LorR].coneConData[1].angle)) if (prospectConeConnectionCount >= 2) else (prospectCones[LorR].coneConData[0].angle)))
            
            # self.mapToUse.debugLines = [] #clear debugLines
            # for LorR in range(2):
            #     self.mapToUse.debugLines.append([1, self.mapToUse.realToPixelPos(lastCones[LorR].position), [4, lastConeAvgConnAngle[LorR]], 1+LorR])
            #     self.mapToUse.debugLines.append([1, self.mapToUse.realToPixelPos(prospectCones[LorR].position), [4, prospectConeAvgConnAngle[LorR]], 1+LorR])
            
            strengths = [1,1,1] #4 possible path lines, one of which already exists (between lastLeftCone and lastRightCone), so calculate the strengths for the remaining three possible pathlines
            pathWidths = [0,0,0];   pathAngles = [0,0,0]
            allCones = lastCones + prospectCones  #combine lists
            allAvgConnAngles = lastConeAvgConnAngle + prospectConeAvgConnAngle #combine lists
            maxStrengthIndex = -1; maxStrengthVal = -1;  winningCones = [None, None]
            for i in range(3):
                pathWidths[i], pathAngles[i] = GF.distAngleBetwPos(allCones[(0 if (i==0) else 2)].position, allCones[(1 if (i==1) else 3)].position) #last left to next right (for left (CCW) corners, where one left cone (lastLeftCone) connects to several right cones  (lastRightCone AND prospectRightCone))
                pathAngles[i] += np.pi/2 #angle is always from left-cone to right-cone, so adding 90deg (CCW rotation) will make it face the direction the car is heading in
                strengths[i] *= 1.5-min(pathWidths[i]/self.pathConnectionThreshold, 1) #strength based on distance, the threshold just determines minimum score, distance can be larger than threshold without math errors
                strengths[i] *= 1.5-min(min(abs(GF.radDiff(pathAngles[i], allAvgConnAngles[(0 if (i==0) else 2)])), abs(GF.radDiff(pathAngles[i], GF.radInv(allAvgConnAngles[(0 if (i==0) else 2)]))))/self.pathConnectionMaxAngleDelta, 1) #strength based on angle delta from lastLeftConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
                strengths[i] *= 1.5-min(min(abs(GF.radDiff(GF.radInv(pathAngles[i]), allAvgConnAngles[(1 if (i==1) else 3)])), abs(GF.radDiff(GF.radInv(pathAngles[i]), GF.radInv(allAvgConnAngles[(1 if (i==1) else 3)]))))/self.pathConnectionMaxAngleDelta, 1) #strength based on angle delta from prospectRightConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
                if(strengths[i] >= maxStrengthVal):
                    maxStrengthVal = strengths[i]
                    maxStrengthIndex = i
                    winningCones[0] = allCones[(0 if (i==0) else 2)];  winningCones[1] = allCones[(1 if (i==1) else 3)]
            
            print("path found:", maxStrengthIndex, "at strength:", round(maxStrengthVal, 2))
            #check if you've gone full circle
            if((winningCones[0].ID == self.mapToUse.target_list[0].coneConData.cones[0].ID) and (winningCones[1].ID == self.mapToUse.target_list[0].coneConData.cones[1].ID)):
                print("path full circle (from winning cones)")
                self.mapToUse.targets_full_circle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            else:
                centerPoint = [winningCones[1].position[0] + (winningCones[0].position[0]-winningCones[1].position[0])/2, winningCones[1].position[1] + (winningCones[0].position[1]-winningCones[1].position[1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
                #newTarget = Map.Target(centerPoint, (winningCones[0].isFinish or winningCones[1].isFinish))
                newTarget = Map.Target(centerPoint)
                newTarget.coneConData = pathFinderData(pathAngles[maxStrengthIndex], pathWidths[maxStrengthIndex], [winningCones[0], winningCones[1]], maxStrengthVal)
                self.mapToUse.target_list.append(newTarget)
        return(True)



