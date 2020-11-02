#TBD: splitscreen, importing cone log files, 

#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import pygame
import numpy as np
import datetime


# Array Scalar Multiplication and Addition
global ASM, ASA
ASM = lambda scalar, inputArray : [scalar * entry for entry in inputArray]
ASA = lambda scalar, inputArray : [scalar + entry for entry in inputArray]
intBoolInv = lambda inputInt : (0 if (inputInt>0) else 1)

#angle rollover functions (for 2d positioning systems, given that arctan2 outputs between (-pi, pi))
degRollTwo = lambda angle : ((angle % 360) if (angle > 360) else ((angle % -360) if (angle < -360) else angle))
degRollOne = lambda angle : ((angle % -180) if (angle > 180) else ((angle % 180) if (angle < -180) else angle))
degRoll = lambda angle : degRollOne(degRollTwo(angle))
radRollTwo = lambda angle : ((angle % (2*np.pi)) if (angle > (2*np.pi)) else ((angle % (-2*np.pi)) if (angle < (-2*np.pi)) else angle))
radRollOne = lambda angle : ((angle % -np.pi) if (angle > np.pi) else ((angle % np.pi) if (angle < -np.pi) else angle))
radRoll = lambda angle : radRollOne(radRollTwo(angle))
#angle math funcitons, that incorporate rollover
degDiff = lambda angleOne, angleTwo : degRoll(degRoll(angleTwo)-degRoll(angleOne))  #recommend using abs(degDiff(angles)), but currently, it returns the value required to get from angleOne to angleTwo, so (90, -45) = -135
radDiff = lambda angleOne, angleTwo : radRoll(radRoll(angleTwo)-radRoll(angleOne))  #recommend using abs(radDiff(angles)), but currently, it returns the value required to get from angleOne to angleTwo, so (90, -45) = -135
degMiddOne = lambda lowBound, upBound : (degRoll(lowBound) + (degDiff(lowBound, upBound)/2) + (180 if (degDiff(lowBound, upBound) < 0) else 0))
degMidd = lambda lowBound, upBound : degRoll(degMiddOne(lowBound, upBound))
radMiddOne = lambda lowBound, upBound : (radRoll(lowBound) + (radDiff(lowBound, upBound)/2) + (np.pi if (radDiff(lowBound, upBound) < 0) else 0))
radMidd = lambda lowBound, upBound : radRoll(radMiddOne(lowBound, upBound))
simpleRange = lambda angle, lowBound, upBound : ((lowBound < angle) and (angle < upBound))
degRangeOne = lambda angle, lowBound, upBound, offset : simpleRange(degRoll(angle-offset), degRoll(lowBound-offset), degRoll(upBound-offset)) #offset values to make lowBound a negative number between (-180, 0) and upBound between (0, 180) and offset input angle the same, to make the check simple
degRange = lambda angle, lowBound, upBound : degRangeOne(degRoll(angle), degRoll(lowBound), degRoll(upBound), degMiddOne(lowBound, upBound))
radRangeOne = lambda angle, lowBound, upBound, offset : simpleRange(radRoll(angle-offset), radRoll(lowBound-offset), radRoll(upBound-offset)) #offset values to make lowBound a negative number between (-pi, 0) and upBound between (0, pi) and offset input angle the same, to make the check simple
radRange = lambda angle, lowBound, upBound : radRangeOne(radRoll(angle), radRoll(lowBound), radRoll(upBound), radMiddOne(lowBound, upBound))
degInv = lambda angle : degRoll(angle + 180)
radInv = lambda angle : radRoll(angle + np.pi)

def distAngleBetwPos(posOne, posTwo):
    funcPosDelta = [posTwo[0]-posOne[0], posTwo[1]-posOne[1]]
    funcDistance = 0 #var init
    funcAngle = np.arctan2(funcPosDelta[1], funcPosDelta[0]) 
    if(abs(funcPosDelta[0]) < 0.001):
        funcDistance = abs(funcPosDelta[1])
    elif(abs(funcPosDelta[1]) < 0.001): #floating point error, alternatively you could check the angle
        funcDistance = abs(funcPosDelta[0])
    else:
        funcDistance = funcPosDelta[1]/np.sin(funcAngle)  #soh
        #funcDistance = funcPosDelta[0]/np.cos(funcAngle)  #cah
    return(funcDistance, funcAngle)

def distSqrdBetwPos(posOne, posTwo):
    return((posTwo[0]-posOne[0])**2 + (posTwo[1]-posOne[1])**2)  #A^2 + B^2 = C^2

def distPowBetwPos(posOne, posTwo):
    return(distSqrdBetwPos(posOne, posTwo)**0.5) #just square root of the squared-distance (C instead of C^2 in A^2+B^2=C^2)

def distAnglePosToPos(funcRadius, funcAngle, funcPos):
    return([funcPos[0] + funcRadius * np.cos(funcAngle), funcPos[1] + funcRadius * np.sin(funcAngle)])

DONT_SORT = 0
SORTBY_DIST = 1
SORTBY_ANGL = 2
SORTBY_ANGL_DELT = 3
SORTBY_ANGL_DELT_ABS = 4

def findIndexBy2DEntry(listToSearch, indexToCompare, valueToFind):
    for i in range(len(listToSearch)):
        if(listToSearch[i][indexToCompare] == valueToFind):
            return(i)
    return(-1) #not found

def findIndexBy3DEntry(listToSearch, firstIndexToCompare, secondIndexToCompare, valueToFind):
    for i in range(len(listToSearch)):
        if(listToSearch[i][firstIndexToCompare][secondIndexToCompare] == valueToFind):
            return(i)
    return(-1) #not found

def minIndex(inputList):
    if(len(inputList) > 0):
        returnIndex = 0
        minVal = inputList[0]
        for i in range(len(inputList)):
            if(inputList[i] < minVal):
                minVal = inputList[i]
                returnIndex = i
        return(returnIndex, minVal)
    else:
        return(-1, 0)

def maxIndex(inputList):
    if(len(inputList) > 0):
        returnIndex = 0
        maxVal = inputList[0]
        for i in range(len(inputList)):
            if(inputList[i] > maxVal):
                maxVal = inputList[i]
                returnIndex = i
        return(returnIndex, maxVal)
    else:
        return(-1, 0)

global CD_FINISH
CD_FINISH = 'finish'


class raceCar:
    def __init__(self, pos, orient=0.0, color=[50,200,50]):
        self.pos = pos
        self.orient = orient
        self.color = color
    
    width = 1.5
    length = 3
    pointRadius = (((width**2)+(length**2))**0.5)/2 #Pythagoras
    pointAngle = np.arctan2(width, length)
    
    #drawing is done by calculating the position of the corners and drawing a polygon with those points. Not efficient, not pretty, just fun
    def draw(self, simSelf): #note: i'm just passing the 'parent' object, but you could absolutely use actual class inheritance structure to do this, i just dont like that sort of thing
        if(simSelf.isInsideWindowReal(self.pos)):
            cornerPoints = []
            # cornerPoints.append(simSelf.realToPixelPos([np.cos(self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
            # cornerPoints.append(simSelf.realToPixelPos([np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
            # cornerPoints.append(simSelf.realToPixelPos([np.cos(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
            # cornerPoints.append(simSelf.realToPixelPos([np.cos(-self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
            offsets = [[np.cos(self.pointAngle+self.orient) * self.pointRadius, np.sin(self.pointAngle+self.orient) * self.pointRadius],
                        [np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius, np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius]]
            cornerPoints.append(simSelf.realToPixelPos([self.pos[0] + offsets[0][0], self.pos[1] + offsets[0][1]]))
            cornerPoints.append(simSelf.realToPixelPos([self.pos[0] + offsets[1][0], self.pos[1] + offsets[1][1]]))
            cornerPoints.append(simSelf.realToPixelPos([self.pos[0] - offsets[0][0], self.pos[1] - offsets[0][1]]))
            cornerPoints.append(simSelf.realToPixelPos([self.pos[0] - offsets[1][0], self.pos[1] - offsets[1][1]]))
            pygame.draw.polygon(simSelf.window, self.color, cornerPoints) #draw car
            arrowPoints = [simSelf.realToPixelPos(self.pos), cornerPoints[1], cornerPoints[2]]
            oppositeColor = [255-self.color[0], 255-self.color[1], 255-self.color[1]]
            pygame.draw.polygon(simSelf.window, oppositeColor, arrowPoints) #draw arrow
    
    # def draw(self, window, sizeScale, drawWidth, drawHeight, drawPosX, drawPosY):  #old old way
    #     if(((self.pos[0]*sizeScale + drawPosX) < (drawWidth + drawPosX)) and ((self.pos[0]*sizeScale + drawPosX) > drawPosX) and ((self.pos[1]*sizeScale + drawPosY) < (drawHeight + drawPosY)) and ((self.pos[1]*sizeScale + drawPosY) > drawPosY)):
    #         cornerPoints = []
    #         cornerPoints.append(( (np.cos(self.pointAngle+self.orient) * self.pointRadius + self.pos[0]) * sizeScale + drawPosX, (np.sin(self.pointAngle+self.orient) * self.pointRadius + self.pos[1]) * sizeScale + drawPosY ))
    #         cornerPoints.append(( (np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[0]) * sizeScale + drawPosX, (np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]) * sizeScale + drawPosY ))
    #         cornerPoints.append(( (np.cos(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[0]) * sizeScale + drawPosX, (np.sin(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[1]) * sizeScale + drawPosY ))
    #         cornerPoints.append(( (np.cos(-self.pointAngle+self.orient) * self.pointRadius + self.pos[0]) * sizeScale + drawPosX, (np.sin(-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]) * sizeScale + drawPosY ))
    #         pygame.draw.polygon(window, self.color, cornerPoints)


class pygamesim:
    def __init__(self, window, cars=[], drawWidth=1200, drawHeight=600, drawPosX=0, drawPosY=0, sizeScale=30, invertYaxis=True, logging=True, logname="pygamesim"):
        self.window = window #pass on the window object (pygame)
        self.cars = cars #list of cars in this simulation (normaly a list with only 1 entry)
        self.drawWidth = int(drawWidth)  #width of the display area (does not have to be 100% of the window)
        self.drawHeight = int(drawHeight) #height of the display area (does not have to be 100% of the window)
        self.drawPosX = int(drawPosX) #draw position offset, 0 is left
        self.drawPosY = int(drawPosY) #draw position offset, 0 is top
        self.sizeScale = sizeScale #pixels per meter
        self.invertYaxis = invertYaxis
        self.logging = logging
        if(logging):
            timeString = datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
            self.logfilename = (logname + "_" + timeString)
            self.logfile = open(self.logfilename + ".csv", "w")
            self.logfile.write("cone ID,leftOrRight,Xpos,Ypos,prev ID,next ID,coneData\n")
            self.rewriteLogTimer = pygame.time.get_ticks() + 2000
    
    bgColor = [50,50,50] #grey
    
    finishLineColor = [255,40,0]
    finishLineWidth = 2 #pixels wide
    finishLinePos = [] #[ [cone ID, [x, y], left/right], [cone ID, [x, y], left/right]]
    
    leftConeColor = [255,255,0] #yellow
    rightConeColor = [0,50,255] #dark blue
    coneLineWidth = 2 #pixels wide
    
    pathColor = [0,220,255] #light blue
    pathLineWidth = 2 #pixels wide
    #pathCenterPixelDiam = 
    
    coneDiam = 0.2 #meters
    #conePixelDiam = coneDiam * sizeScale #might make math faster, but will require re-calculating on change
    
    coneConnectionThreshold = 5  #in meters (or at least not pixels)  note: hard threshold beyond which cones will NOT come into contention for connection
    coneConnectionThresholdSquared = coneConnectionThreshold**2
    coneConnectionMaxAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
    
    pathConnectionThreshold = 10 #in meters (or at least not pixels)  IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
    pathConnectionMaxAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
    
    pathFirstLineCarAngleDeltaMax = np.deg2rad(45) #if the radDiff() between car (.orient) and the first line's connections is bigger than this, switch conneections or stop
    
    totalConeList = [] #[ [cone ID, left/right, index in left/right array], ]             #note: left=False, right=True
    leftConeList = []  #[ [cone ID, [x,y], [[cone ID, index (left), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
    rightConeList = [] #[ [cone ID, [x,y], [[cone ID, index (right), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
    pathList = [] #[[center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength], ]
    # #the pathList is a (properly ordered) list of points/lines for the car to travel through. It (like most lists here) prioratizes math efficiency over RAM usage, so several stored parameters are redundant (can be recalculated given other elements), which should save time when calculating optimal path.
    newConeID = 0 #add 1 after adding a cone
    
    #leftConesFullCircle = False  #TBD: no checking function yet, and because connectCone() can connect a cone to the 0th cone at any time without completing the circle, a special function this is required
    #rightConesFullCircle = False
    pathFullCircle = False
    
    floatingCone = [] #[ [[xpixel,ypixel], color] ] #note: x and y are in pixels, not meters. Translation to meters happens on final placement
    
    rewriteLogTimer = 0
    logFileChanged = False
    
    debugLines = [] #[[lineType, pos, pos/angles, color_index (0-2)], ] #third entry is: (lineType==0: straight line from two positions), (lineType==1: straight line from pos and [radius, angle]), (lineType==2: arc from pos and [radius, startAngle, endAngle])
    debugLineColors = [[255,0,255],[255,255,255],[0,255,0]] #purple, white, green
    debugLineWidth = 3
    
    def closeLog(self):
        if(self.logging): #just a safety check
            self.logfile.close()
    
    def coneDataToString(self, coneData):
        resultString = "" if (len(coneData)==0) else str(coneData).replace(',',';') #TBD
        return(resultString)
    
    def coneDataCopy(self, coneData):
        copyOfConeData = []
        for entry in coneData:
            #check for special situations here (multi-dimensional arrays or lists within lists)
            #if it's just a regular entry, copy it
            copyOfConeData.append(entry)
        return(copyOfConeData)
    
    def logCone(self, coneID, leftOrRight, pos, connections, coneData):
        leftOrRightString = "RIGHT" if leftOrRight else "LEFT"
        self.logfile.write(str(coneID) +','+ leftOrRightString +','+ str(round(pos[0], 2)) +','+ str(round(pos[1], 2)) +','+ str(connections[0][0]) +','+ str(connections[1][0]) +','+ self.coneDataToString(coneData) + '\n')
    
    def rewriteLogfile(self):
        print("rewriting log file")
        if(self.logging): #just a safety check
            self.logfile.close() #close last file (because if anything goes wrong, that data is not lost
            self.logfile = open(self.logfilename + ".csv", "w")
            self.logfile.write("cone ID,leftOrRight,Xpos,Ypos,prev ID,next ID,coneData\n")
            for cone in self.totalConeList: #cone is an array with [cone ID, left/right, index in left/right array]
                conePos = self.rightConeList[cone[2]][1] if cone[1] else self.leftConeList[cone[2]][1]  #get pos from left/right arrays
                coneConnections = self.rightConeList[cone[2]][2] if cone[1] else self.leftConeList[cone[2]][2]  #get previous and next cone indexes from left/right arrays
                coneData = self.rightConeList[cone[2]][3] if cone[1] else self.leftConeList[cone[2]][3] #get coneData from left/right arrays
                self.logCone(cone[0], cone[1], conePos, coneConnections, coneData) #rewrite all data (some of which is updated)
            self.rewriteLogTimer = pygame.time.get_ticks() + 2000
            self.logFileChanged = False #reset flag
    
    
    def pixelsToRealPos(self, pixelPos):
        if(self.invertYaxis):
            return([(pixelPos[0]-self.drawPosX)/self.sizeScale, (self.drawHeight-pixelPos[1]+self.drawPosY)/self.sizeScale])
        else:
            return([(pixelPos[0]-self.drawPosX)/self.sizeScale, (pixelPos[1]-self.drawPosY)/self.sizeScale])
    
    def realToPixelPos(self, realPos):
        if(self.invertYaxis):
            return([(realPos[0]*self.sizeScale)+self.drawPosX, self.drawHeight-(realPos[1]*self.sizeScale)+self.drawPosY]) #invert Y-axis for normal (0,0) at bottomleft display
        else:
            return([(realPos[0]*self.sizeScale)+self.drawPosX, (realPos[1]*self.sizeScale)+self.drawPosY])
    
    def isInsideWindowPixels(self, pixelPos):
        return((pixelPos[0] < (self.drawWidth + self.drawPosX)) and (pixelPos[0] > self.drawPosX) and (pixelPos[1] < (self.drawHeight + self.drawPosY)) and (pixelPos[1] > self.drawPosY))
    
    def isInsideWindowReal(self, realPos):
        return((realPos[0] < (self.drawWidth/self.sizeScale)) and (realPos[0] > 0.0) and (realPos[1] < (self.drawHeight/self.sizeScale)) and (realPos[1] > 0.0)) #not changed by inverting Y because it doesnt care WHERE inside the Y-axis it is, just that it IS
    
    def distanceToConeSquared(self, pos, listsToCheck=[leftConeList, rightConeList], sortByDistance=False, mergeLists=True, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, excludeDoublyConnectedCones=False, ignoreLinkedConeIDs=[]):
        returnList = []  #[[ [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], squaredDist, index in left/right array, cone data (certainty, time spotted, etc)], ], ]  #note: coneID of first cone in first conelist is array[0][0][0]
        for conelist in range(len(listsToCheck)):
            if(not mergeLists):
                returnList.append([])
            #for cone in listsToCheck[conelist]: #doesnt let me store index
            for coneIndex in range(len(listsToCheck[conelist])):
                cone = listsToCheck[conelist][coneIndex]
                #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
                ignoreCone = False
                if(cone[0] in ignoreConeIDs):
                    ignoreCone = True
                if(excludeDoublyConnectedCones and (cone[2][0][1] >= 0) and (cone[2][1][1] >= 0)):
                    ignoreCone = True
                elif((cone[2][0][1] >= 0) or (cone[2][1][1] >= 0)):
                    for coneIDtoIgnore in ignoreLinkedConeIDs:  #using "((cone[2][0][0] in ignoreLinkedConeIDs) or (cone[2][0][0] in ignoreLinkedConeIDs))" would search the array twice, a single forloop is faster
                        if((cone[2][0][0] == coneIDtoIgnore) or (cone[2][1][0] == coneIDtoIgnore)):
                            ignoreCone = True
                if(not ignoreCone):
                    squaredDistance = (pos[0]-cone[1][0])**2 + (pos[1]-cone[1][1])**2  #A^2 + B^2 = C^2
                    if((simpleSquaredThreshold < 0) or ((simpleSquaredThreshold > 0) and (squaredDistance < simpleSquaredThreshold))):
                        returnCone = []
                        returnCone.append(cone[0]) #cone ID
                        returnCone.append([item for item in cone[1]]) #pos
                        returnCone.append([[subItem for subItem in item] for item in cone[2]]) #[[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]]
                        returnCone.append(squaredDistance)
                        returnCone.append(coneIndex)
                        returnCone.append(self.coneDataCopy(cone[3]))
                        #store the data in the return list. If 'sortByDistance', then sort it right here, as opposed to afterwards
                        #alternatively, 'listToAppend = returnList if mergeLists else returnList[conelist]' and the just append to that (saves a few lines of code)
                        if(mergeLists):
                            if(sortByDistance):
                                insertionDone = False
                                for i in range(len(returnList)):
                                    if((returnCone[3] < returnList[i][3]) and (not insertionDone)): #if the new entry is larger
                                        returnList.insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList.append(returnCone)
                            else:
                                returnList.append(returnCone)
                        else:
                            if(sortByDistance):
                                insertionDone = False
                                for i in range(len(returnList[conelist])):
                                    if((returnCone[3] < returnList[conelist][i][3]) and (not insertionDone)): #if the new entry is larger
                                        returnList[conelist].insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList[conelist].append(returnCone)
                            else:
                                returnList[conelist].append(returnCone)
        return(returnList)
    
    def distanceToCone(self, pos, listsToCheck=[leftConeList, rightConeList], sortBySomething=DONT_SORT, mergeLists=True, ignoreConeIDs=[], simpleThreshold=-1.0, excludeDoublyConnectedCones=False, ignoreLinkedConeIDs=[], angleDeltaTarget=0.0, angleThreshRange=[]): #note: angleThreshRange is [lowBound, upBound]
        returnList = []  #[[ [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data (certainty, time spotted, etc)], ], ]  #note: coneID of first cone in first conelist is array[0][0][0]
        for conelist in range(len(listsToCheck)):
            if(not mergeLists):
                returnList.append([])
            #for cone in listsToCheck[conelist]: #doesnt let me store index
            for coneIndex in range(len(listsToCheck[conelist])):
                cone = listsToCheck[conelist][coneIndex]
                #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
                ignoreCone = False
                if(cone[0] in ignoreConeIDs):
                    ignoreCone = True
                if(excludeDoublyConnectedCones and (cone[2][0][1] >= 0) and (cone[2][1][1] >= 0)):
                    ignoreCone = True
                elif((cone[2][0][1] >= 0) or (cone[2][1][1] >= 0)):
                    for coneIDtoIgnore in ignoreLinkedConeIDs:  #using "((cone[2][0][0] in ignoreLinkedConeIDs) or (cone[2][0][0] in ignoreLinkedConeIDs))" would search the array twice, a single forloop is faster
                        if((cone[2][0][0] == coneIDtoIgnore) or (cone[2][1][0] == coneIDtoIgnore)):
                            ignoreCone = True
                if(not ignoreCone):
                    # posDelta = [cone[1][0]-pos[0], cone[1][1]-pos[1]]
                    # angle = np.arctan2(posDelta[1], posDelta[0]) 
                    # distance = posDelta[1]/np.sin(angle)  #soh
                    # #distance = posDelta[0]/np.cos(angle)  #cah
                    # #distance = (posDelta[0]**2 + posDelta[1]**2)**0.5  #(A^2 + B^2)^0.5 = C
                    distance, angle = distAngleBetwPos(pos, cone[1]) #math obove was moved to a handy function
                    if(((distance < simpleThreshold) if (simpleThreshold > 0) else True) and ((radRange(angle, angleThreshRange[0], angleThreshRange[1])) if (len(angleThreshRange)==2) else True)):
                        returnCone = []
                        returnCone.append(cone[0]) #cone ID
                        returnCone.append([item for item in cone[1]]) #pos
                        returnCone.append([[subItem for subItem in item] for item in cone[2]]) #[[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]]
                        returnCone.append([distance, angle])
                        returnCone.append(coneIndex)
                        returnCone.append(self.coneDataCopy(cone[3]))
                        #store the data in the return list. If 'sortByDistance', then sort it right here, as opposed to afterwards
                        #alternatively, 'listToAppend = returnList if mergeLists else returnList[conelist]' and the just append to that (saves a few lines of code)
                        if(mergeLists):
                            if(sortBySomething == SORTBY_DIST):
                                insertionDone = False
                                for i in range(len(returnList)):
                                    if((returnCone[3][0] < returnList[i][3][0]) and (not insertionDone)): #if the new entry is larger
                                        returnList.insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList.append(returnCone)
                            elif(sortBySomething == SORTBY_ANGL):
                                insertionDone = False
                                for i in range(len(returnList)):
                                    if((returnCone[3][1] < returnList[i][3][1]) and (not insertionDone)): #if the new entry is larger
                                        returnList.insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList.append(returnCone)
                            elif(sortBySomething == SORTBY_ANGL_DELT):
                                insertionDone = False
                                for i in range(len(returnList)):
                                    if((radDiff(returnCone[3][1], angleDeltaTarget) < radDiff(returnList[i][3][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                        returnList.insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList.append(returnCone)
                            elif(sortBySomething == SORTBY_ANGL_DELT_ABS):
                                insertionDone = False
                                for i in range(len(returnList)):
                                    if((abs(radDiff(returnCone[3][1], angleDeltaTarget)) < abs(radDiff(returnList[i][3][1], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
                                        returnList.insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList.append(returnCone)
                            else:
                                returnList.append(returnCone)
                        else:
                            if(sortBySomething == SORTBY_DIST):
                                insertionDone = False
                                for i in range(len(returnList[conelist])):
                                    if((returnCone[3][0] < returnList[conelist][i][3][0]) and (not insertionDone)): #if the new entry is larger
                                        returnList[conelist].insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList[conelist].append(returnCone)
                            if(sortBySomething == SORTBY_ANGL):
                                insertionDone = False
                                for i in range(len(returnList[conelist])):
                                    if((returnCone[3][1] < returnList[conelist][i][3][1]) and (not insertionDone)): #if the new entry is larger
                                        returnList[conelist].insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList[conelist].append(returnCone)
                            if(sortBySomething == SORTBY_ANGL_DELT):
                                insertionDone = False
                                for i in range(len(returnList[conelist])):
                                    if((radDiff(returnCone[3][1], angleDeltaTarget) < radDiff(returnList[conelist][i][3][0], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                        returnList[conelist].insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList[conelist].append(returnCone)
                            if(sortBySomething == SORTBY_ANGL_DELT_ABS):
                                insertionDone = False
                                for i in range(len(returnList[conelist])):
                                    if((abs(radDiff(returnCone[3][1], angleDeltaTarget)) < abs(radDiff(returnList[conelist][i][3][0], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
                                        returnList[conelist].insert(i, returnCone)
                                        insertionDone = True
                                if(not insertionDone): #if the new entry is larger than the last entry, append it
                                    returnList[conelist].append(returnCone)
                            else:
                                returnList[conelist].append(returnCone)
        return(returnList)
    
    
    def overlapConeCheck(self, pos):
        boolAnswer = False ;  totalConeListEntry = []
        coneDistToler = self.coneDiam*2 #NOTE: area is square
        for cone in self.totalConeList: #cone is an array with [cone ID, left/right, index in left/right array]
            conePos = self.rightConeList[cone[2]][1] if cone[1] else self.leftConeList[cone[2]][1]
            if((pos[0] > (conePos[0]-coneDistToler)) and (pos[0] < (conePos[0]+coneDistToler)) and (pos[1] > (conePos[1]-coneDistToler)) and (pos[1] < (conePos[1]+coneDistToler))):
                boolAnswer = True
                totalConeListEntry = [item for item in cone] #copy cone to totalConeListEntry
        return(boolAnswer, totalConeListEntry)
    
    def connectCone(self, coneToConnectID, coneToConnectPos, leftOrRight, coneToConnectIndex, currentConeConnections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], coneToConnectPreferredConnection=1):
        #the correct cone should be selected based on a number of parameters:
        #distance to last cone, angle difference from last and second-to-last cones's, angle that 'track' is (presumably) going based on cones on other side (left/right) (if right cones make corner, left cones must also), etc
        # ideas: distance between last (existing) cone connection may be similar to current cone distance (human may place cones in inner corners close together and outer corners far apart, but at least consistent)
        # ideas: vincent's "most restrictive" angle could be done by realizing the a right corner (CW) is restrictive for left-cones, and a left corner (CCW) is restrictive for right-cones, so whether the angle delta is positive or negative (and leftOrRight boolean) could determine strength
        #all/more parameters to be added later, in non-simulation code
        currentConnectionsFilled = [(currentConeConnections[0][1] >= 0), (currentConeConnections[1][1] >= 0)] #2-size list of booleans
        if(currentConnectionsFilled[0] and currentConnectionsFilled[1]):#if both connection entries are already full
            print("input cone already doubly connected?!:", currentConeConnections)
            return(currentConeConnections)
        else:
            currentFrontOrBack = (intBoolInv(coneToConnectPreferredConnection) if (currentConnectionsFilled[coneToConnectPreferredConnection]) else coneToConnectPreferredConnection) # default to coneToConnectPreferredConnection
            currentExistingAngle = radRoll(radInv(currentConeConnections[intBoolInv(currentFrontOrBack)][2])) #if there is not existing connection, this will return radRoll(radInv(0.0)), which is pi (or -pi). only make use of this value if(currentConnectionsFilled[0] or currentConnectionsFilled[1])
            nearbyConeList = self.distanceToCone(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], SORTBY_DIST, True, [coneToConnectID], self.coneConnectionThreshold, True, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
            # nearbyConeList structure: [[cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data], ]
            if(len(nearbyConeList) > 0):
                #returnConnections = currentConeConnections #copy pointer
                returnConnections = [[subItem for subItem in item] for item in currentConeConnections]  #copy data, not pointer
                #coneCandidateStrengthList = [] #does not have to be list, can be newly assigned array
                bestCandidateIndex = -1; highestStrength = 0; otherfrontOrBack = -1
                for i in range(len(nearbyConeList)):
                    cone = nearbyConeList[i] #not needed, just for legibility
                    #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data]
                    connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
                    if(connectionsFilled[0] and connectionsFilled[1]): #cone already doubly connected
                        print("cone already doubly connected, but that was supposed to be filtered out in distanceToCone()!?!")
                        coneCandidateStrength = -1 #not used
                        #coneCandidateStrengthList.append(coneCandidateStrength)
                    elif((cone[2][0][0] == coneToConnectID) or (cone[2][1][0] == coneToConnectID)): #cone already connected to coneToConnect
                        print("cone connection already exists, but that was supposed to be filtered out in distanceToCone()!?!")
                        coneCandidateStrength = -1 #not used
                        #coneCandidateStrengthList.append(coneCandidateStrength)
                    else:
                        frontOrBack = (coneToConnectPreferredConnection if (connectionsFilled[intBoolInv(coneToConnectPreferredConnection)]) else intBoolInv(coneToConnectPreferredConnection)) # default to the inverse of coneToConnectPreferredConnection
                        coneCandidateStrength = 1 #init var
                        coneCandidateStrength *= 1.5-(cone[3][0]/self.coneConnectionThreshold)  #high distance, low strength. Linear. worst>0.5 best<1.5
                        # angleToCone = cone[3][1]
                        # if(connectionsFilled[0] or connectionsFilled[1]): #if cone already has a connection, check the angle delta
                        #     coneExistingAngle = cone[2][intBoolInv(frontOrBack)][2] #note: intBoolInv() is used to grab the existing connection
                        #     coneCandidateStrength *= 1.5- min(abs(radDiff(angleToCone, coneExistingAngle))/self.coneConnectionMaxAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                        # if(currentConnectionsFilled[0] or currentConnectionsFilled[1])
                        #     coneCandidateStrength *= 1.5- min(abs(radDiff(angleToCone, currentExistingAngle))/self.coneConnectionMaxAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                        # if(
                        #coneCandidateStrengthList.append(coneCandidateStrength)
                        if(coneCandidateStrength > highestStrength):
                            highestStrength = coneCandidateStrength
                            bestCandidateIndex = i
                            otherfrontOrBack = frontOrBack
                
                #bestCandidateIndex, highestStrength = maxIndex(coneCandidateStrengthList)
                print("ID:", coneToConnectID, "we have a winner:", bestCandidateIndex, "at strength", round(highestStrength, 2), "  ID:", nearbyConeList[bestCandidateIndex][0], "frontOrBack:", otherfrontOrBack)
                
                #make the connection 1; collect the data to be returned
                returnConnections[currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][0] #save coneID
                returnConnections[currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][4] #save index
                returnConnections[currentFrontOrBack][2] = nearbyConeList[bestCandidateIndex][3][1] #save angle (from perspective of coneToConnect)
                returnConnections[currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
                returnConnections[currentFrontOrBack][4] = highestStrength
                #make the connection 2; set the winning cone's connection data
                winnerConeIndexInList = returnConnections[currentFrontOrBack][1]
                if(leftOrRight):
                    self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
                    self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
                    self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][3][1]) #save angle (from perspective of that cone)
                    self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
                    self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
                else:
                    self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
                    self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
                    self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][3][1]) #save angle (from perspective of that cone)
                    self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
                    self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
                self.logFileChanged = True #set flag
                return(returnConnections)
            else:
                print("nearbyConeList empty")
                return(currentConeConnections)
    
    # def connectConeSuperSimple(self, coneToConnectID, coneToConnectPos, leftOrRight, coneToConnectIndex, currentConeConnections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], coneToConnectPreferredConnection=1):
    #     currentConnectionsFilled = [(currentConeConnections[0][1] >= 0), (currentConeConnections[1][1] >= 0)] #2-size list of booleans
    #     if(currentConnectionsFilled[0] and currentConnectionsFilled[1]):#if both connection entries are already full
    #         print("input cone already doubly connected?!:", currentConeConnections)
    #         return(currentConeConnections)
    #     else:
    #         currentFrontOrBack = (intBoolInv(coneToConnectPreferredConnection) if (currentConnectionsFilled[coneToConnectPreferredConnection]) else coneToConnectPreferredConnection) # default to coneToConnectPreferredConnection
    #         nearbyConeList = self.distanceToConeSquared(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], True, True, [coneToConnectID], self.coneConnectionThresholdSquared, True, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
    #         if(len(nearbyConeList) > 0):
    #             #returnConnections = currentConeConnections #copy pointer
    #             returnConnections = [[subItem for subItem in item] for item in currentConeConnections]  #copy data, not pointer
    #             #coneCandidateStrengthList = [] #does not have to be list, can be newly assigned array
    #             bestCandidateIndex = -1; highestStrength = 0; otherfrontOrBack = -1
    #             for i in range(len(nearbyConeList)):
    #                 cone = nearbyConeList[i] #not needed, just for legibility
    #                 #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], squared dist, index in left/right array, cone data (certainty, time spotted, etc)]
    #                 connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
    #                 if(connectionsFilled[0] and connectionsFilled[1]):
    #                     coneCandidateStrength = -1
    #                     #coneCandidateStrengthList.append(coneCandidateStrength)
    #                 else:
    #                     frontOrBack = (coneToConnectPreferredConnection if (connectionsFilled[intBoolInv(coneToConnectPreferredConnection)]) else intBoolInv(coneToConnectPreferredConnection)) #default to the inverse of coneToConnectPreferredConnection
    #                     coneCandidateStrength = 1 #init var
    #                     coneCandidateStrength *= 1.5-(cone[3]/self.coneConnectionThresholdSquared)  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5
    #                     #coneCandidateStrength *= 1.5-(cone[3][0]/self.coneConnectionThreshold)  #high distance, low strength. Linear. worst>0.5 best<1.
    #                     #curAngle = cone[3][1]
    #                     ##coneCandidateStrength *= 1.5- min(abs(cone[3][1]-angleTrend)/self.coneConnectionMaxAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
    #                     #if(connectionsFilled[0] or connectionsFilled[1]): #if cone already has a connection, check the angle delta
    #                     #    prevAngle = cone[2][intBoolInv(frontOrBack)][2] #note: intBoolInv() is used to grab the existing connection
    #                     #    delta between last and new angle can factor into strength, low delta, high strength
    #                     #    math...
    #                     #coneCandidateStrengthList.append(coneCandidateStrength)
    #                     if(coneCandidateStrength > highestStrength):
    #                         highestStrength = coneCandidateStrength
    #                         bestCandidateIndex = i
    #                         otherfrontOrBack = frontOrBack
                
    #             #bestCandidateIndex, highestStrength = maxIndex(coneCandidateStrengthList)
    #             print("we have a simple winner:", bestCandidateIndex, "at strength", round(highestStrength, 2), "  ID:", nearbyConeList[bestCandidateIndex][0], "frontOrBack:", otherfrontOrBack)
                
    #             #make the connection 1; collect the data to be returned
    #             returnConnections[currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][0] #save coneID
    #             returnConnections[currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][4] #save index
    #             #returnConnections[currentFrontOrBack][2] = nearbyConeList[bestCandidateIndex][3][1] #save angle (from perspective of coneToConnect)
    #             #returnConnections[currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
    #             returnConnections[currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][3] #save squared distance
    #             returnConnections[currentFrontOrBack][4] = highestStrength
                
    #             #make the connection 2; set the winning cone's connection data
    #             winnerConeIndexInList = returnConnections[currentFrontOrBack][1]
    #             if(leftOrRight):
    #                 self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
    #                 self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
    #                 #self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][3][1]) #save angle (from perspective of that cone)
    #                 #self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
    #                 self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3] #save squared distance
    #                 self.rightConeList[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
    #             else:
    #                 self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
    #                 self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
    #                 #self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][3][1]) #save angle (from perspective of that cone)
    #                 #self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
    #                 self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3] #save squared distance
    #                 self.leftConeList[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
    #             self.logFileChanged = True #set flag
    #             return(returnConnections)
    #         else:
    #             print("nearbyConeList empty")
    #             return(currentConeConnections)
    
    def makePath(self):
        # pathList content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        # left/right-ConeList content: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]
        if(self.pathFullCircle):
            print("not gonna make path, already full circle")
            return(False)
        if(len(self.pathList) == 0):
            #TBD !
            #delete this:
            firstLineLeftCone = self.leftConeList[0];   firstLineRightCone = self.rightConeList[0]
            
            ## if firstLine___Cone is connected to something, make sure those connections are angled similarly to the car itself, maybe
            
            pathWidth, lineAngle = distAngleBetwPos(firstLineLeftCone[1], firstLineRightCone[1])
            carAngle = radRoll(lineAngle + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
            centerPoint = [firstLineRightCone[1][0] + (firstLineLeftCone[1][0]-firstLineRightCone[1][0])/2, firstLineRightCone[1][1] + (firstLineLeftCone[1][1]-firstLineRightCone[1][1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
            self.pathList.append([centerPoint, [lineAngle, carAngle], pathWidth, [firstLineLeftCone[0], firstLineLeftCone[1], 0], [firstLineRightCone[0], firstLineRightCone[1], 0], 69.420])
        else:
            lastPathLine = self.pathList[-1] # -1 gets the last item in list, you could also use (len(pathList)-1)
            lastLeftCone = self.leftConeList[lastPathLine[3][2]]
            lastRightCone = self.rightConeList[lastPathLine[4][2]]
            lastLeftConeConnectionIndex = 1 #try to use 'front' connection by default
            lastRightConeConnectionIndex = 1
            lastLeftConnectionsFilled = [lastLeftCone[2][0][1] >= 0, lastLeftCone[2][1][1] >= 0]
            lastRightConnectionsFilled = [lastRightCone[2][0][1] >= 0, lastRightCone[2][1][1] >= 0]
            #most of the code below is sanity checks, but some of it is for the purpouses of flipping connections to fit the 'back','front' model. This can be done differently, by putting it in the connectCone() function, for example. Threre might be some useless redundancy in the code below, but fuck it, it works (for now)
            ## check basics NOTE: certain parts may become obsolete, given later improvements to the first-pathLine-finding code. The more restrictive the first pathLine is, the simpler finding the rest of the path becomes
            if(len(self.pathList) == 1): #in the (somewhat niche) scenario where the first pathline has been drawn, but the direction to follow is still unclear
                if(not (lastLeftConnectionsFilled[0] or lastLeftConnectionsFilled[1])):
                    #there is only one pathLine entry, and the last (and therefore also first) left cone has no connections, then why bother looking
                    print("no connections on first lastLeftCone (unlikely):", lastLeftCone)
                    return(False)
                elif(not lastLeftConnectionsFilled[lastLeftConeConnectionIndex]): #if it only has a 'back' connection, just move the back one to the front
                    #first, switch connection data to make preferable (front) the only valid one
                    #print("whipping lastLeft (1):", lastLeftCone[2])
                    tempConVal = lastLeftCone[2][0] #one of these is an empty connection
                    lastLeftCone[2][0] = lastLeftCone[2][1]
                    lastLeftCone[2][1] = tempConVal
                    self.logFileChanged = True #set flag
                    #then check the angle of that connection. If it is too far off from the car angle then something is terribly wrong (or 
                    if(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                        print("first and only left angle large:", lastLeftConeConnectionIndex, round(np.rad2deg(lastLeftCone[2][lastLeftConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient))),2))
                        return(False)
                elif(lastLeftConnectionsFilled[intBoolInv(lastLeftConeConnectionIndex)]): #if it has both connections:
                    if(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                        #print("first left angle large:", lastLeftConeConnectionIndex, round(np.rad2deg(lastLeftCone[2][lastLeftConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient))),2))
                        if(abs(radDiff(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                            print("second left angle also large", round(np.rad2deg(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2], self.cars[0].orient))),2))
                            return(False)
                        else: #first angle was large, but second angle wasnt, just switch the connections around and we're good to go
                            #print("whipping lastLeft (2):", lastLeftCone[2])
                            tempConVal = lastLeftCone[2][0] #one of these is an empty connection
                            lastLeftCone[2][0] = lastLeftCone[2][1]
                            lastLeftCone[2][1] = tempConVal
                            self.logFileChanged = True #set flag
                ##else do nothing, everything is allready good and there's no need to worry
            elif(not lastLeftConnectionsFilled[lastLeftConeConnectionIndex]): #if it doesnt have a connected cone on the 
                #print("no preferable (front) connection on lastLeftCone")
                if(not lastLeftConnectionsFilled[intBoolInv(lastLeftConeConnectionIndex)]):
                    print("no connections on lastLeftCone (impossible):", lastLeftCone)
                    return(False)
                else:
                    if(findIndexBy3DEntry(self.pathList, 3, 0, lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][0]) >= 0): #check if that isnt already in pathlist
                        ## if it is, then we just stop here. no more path generation can be done for now
                        print("single lastLeft connection already in pathList")
                        return(False)
                    else: #if not, then the 'back' connection is the next (prospect) one, and this (last) cone has it all backwards. Switch the connection data for this cone around
                        #print("whipping lastLeft (3):", lastLeftCone[2])
                        tempConVal = lastLeftCone[2][0]
                        lastLeftCone[2][0] = lastLeftCone[2][1]
                        lastLeftCone[2][1] = tempConVal
                        self.logFileChanged = True #set flag
            
            if(len(self.pathList) == 1): #in the (somewhat niche) scenario where the first pathline has been drawn, but the direction to follow is still unclear
                if(not (lastRightConnectionsFilled[0] or lastRightConnectionsFilled[1])):
                    #there is only one pathLine entry, and the last (and therefore also first) right cone has no connections, then why bother looking
                    print("no connections on first lastRightCone (unlikely):", lastRightCone)
                    return(False)
                elif(not lastRightConnectionsFilled[lastRightConeConnectionIndex]): #if it only has a 'back' connection, just move the back one to the front
                    #first, switch connection data to make preferable (front) the only valid one
                    #print("whipping lastRight (1):", lastRightCone[2])
                    tempConVal = lastRightCone[2][0] #one of these is an empty connection
                    lastRightCone[2][0] = lastRightCone[2][1]
                    lastRightCone[2][1] = tempConVal
                    self.logFileChanged = True #set flag
                    #then check the angle of that connection. If it is too far off from the car angle then something is terribly wrong (or 
                    if(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                        print("first and only right angle large:", lastRightConeConnectionIndex, round(np.rad2deg(lastRightCone[2][lastRightConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient))),2))
                        return(False)
                elif(lastRightConnectionsFilled[intBoolInv(lastRightConeConnectionIndex)]): #if it has both connections:
                    if(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                        #print("first right angle large:", lastRightConeConnectionIndex, round(np.rad2deg(lastRightCone[2][lastRightConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient))),2))
                        if(abs(radDiff(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2], self.cars[0].orient)) > self.pathFirstLineCarAngleDeltaMax):
                            print("second right angle also large", round(np.rad2deg(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2], self.cars[0].orient))),2))
                            return(False)
                        else: #first angle was large, but second angle wasnt, just switch the connections around and we're good to go
                            #print("whipping lastRight (2):", lastRightCone[2])
                            tempConVal = lastRightCone[2][0] #one of these is an empty connection
                            lastRightCone[2][0] = lastRightCone[2][1]
                            lastRightCone[2][1] = tempConVal
                            self.logFileChanged = True #set flag
                ##else do nothing, everything is allready good and there's no need to worry
            elif(not lastRightConnectionsFilled[lastRightConeConnectionIndex]): #if it doesnt have a connected cone on the 
                #print("no preferable (front) connection on lastRightCone")
                if(not lastRightConnectionsFilled[intBoolInv(lastRightConeConnectionIndex)]):
                    print("no connections on lastRightCone (impossible):", lastRightCone)
                    return(False)
                else:
                    if(findIndexBy3DEntry(self.pathList, 4, 0, lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][0]) >= 0): #check if that isnt already in pathlist
                        ## if it is, then we just stop here. no more path generation can be done for now
                        print("single lastRight connection already in pathList")
                        return(False)
                    else: #if not, then the 'back' connection is the next (prospect) one, and this (last) cone has it all backwards. Switch the connection data for this cone around
                        #print("whipping lastRight (3):", lastRightCone[2])
                        tempConVal = lastRightCone[2][0]
                        lastRightCone[2][0] = lastRightCone[2][1]
                        lastRightCone[2][1] = tempConVal
                        self.logFileChanged = True #set flag
            
            if(len(self.pathList) == 1): #now check both angles again, just to be sure:
                if(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient) > self.pathFirstLineCarAngleDeltaMax)):
                    print("post correction first left angle large:", lastLeftConeConnectionIndex, round(np.rad2deg(lastLeftCone[2][lastLeftConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastLeftCone[2][lastLeftConeConnectionIndex][2], self.cars[0].orient))),2))
                    if(((lastLeftConnectionsFilled[intBoolInv(lastLeftConeConnectionIndex)])) and (abs(radDiff(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2], self.cars[0].orient) > self.pathFirstLineCarAngleDeltaMax))):
                        print("post correction second angle also large", round(np.rad2deg(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][2], self.cars[0].orient))),2))
                    return(False)
                if(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient) > self.pathFirstLineCarAngleDeltaMax)):
                    print("post correction first right angle large:", lastRightConeConnectionIndex, round(np.rad2deg(lastRightCone[2][lastRightConeConnectionIndex][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastRightCone[2][lastRightConeConnectionIndex][2], self.cars[0].orient))),2))
                    if(abs(radDiff(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2], self.cars[0].orient) > self.pathFirstLineCarAngleDeltaMax)):
                        print("post correction second angle also large", round(np.rad2deg(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2]), 2), round(np.rad2deg(self.cars[0].orient), 2), round(np.rad2deg(abs(radDiff(lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][2], self.cars[0].orient))),2))
                    return(False)
            
            lastLeftConePerpAngle = (radMidd(lastLeftCone[2][0][2], lastLeftCone[2][1][2]) if (lastLeftCone[2][intBoolInv(lastLeftConeConnectionIndex)][1] >= 0) else radRoll(lastLeftCone[2][lastLeftConeConnectionIndex][2] + (np.pi*(-0.5 if (lastLeftConeConnectionIndex==1) else 0.5)))) #all of this assumes CCW is positive and CW is negative
            lastRightConePerpAngle = (radMidd(lastRightCone[2][1][2], lastRightCone[2][0][2]) if (lastRightCone[2][intBoolInv(lastRightConeConnectionIndex)][1] >= 0) else radRoll(lastRightCone[2][lastRightConeConnectionIndex][2] + (np.pi*(0.5 if (lastRightConeConnectionIndex==1) else -0.5))))
            prospectLeftCone = self.leftConeList[lastLeftCone[2][lastLeftConeConnectionIndex][1]] #get index from connection from lastLeftCone
            prospectRightCone = self.rightConeList[lastRightCone[2][lastRightConeConnectionIndex][1]] #get index from connection from lastRightCone
            # #check if you've gone full circle
            # if((prospectLeftCone[0] == self.pathList[0][3][0]) and (prospectRightCone[0] == self.pathList[0][4][0])):
            #     print("path full circle (by default)")
            #     self.pathFullCircle = True
            #     return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            prospectLeftConeConnectionIndex = (0 if (prospectLeftCone[2][0][0] == lastLeftCone[0]) else (1 if (prospectLeftCone[2][1][0] == lastLeftCone[0]) else -1)) #match connection. In simple, regular situations you could assume that the 'front' connection of the lastCone is the 'back' connection of prospectCone, but this is not a simple system, now is it :)
            prospectRightConeConnectionIndex = (0 if (prospectRightCone[2][0][0] == lastRightCone[0]) else (1 if (prospectRightCone[2][1][0] == lastRightCone[0]) else -1)) #match connection. In simple, regular situations you could assume that the 'front' connection of the lastCone is the 'back' connection of prospectCone, but this is not a simple system, now is it :)
            ## check connections and stop or switch around if needed
            if(prospectLeftConeConnectionIndex == -1):
                print("BIG issue: lastLeftCone pointed to this prospect cone, but this prospect cone does not point back", lastLeftCone[2], prospectLeftCone[2])
            elif(prospectLeftConeConnectionIndex == 1): #prospect cone has its connections switched around (lastLeftCone's 'front' should connect to prospectLeftCone's 'back')
                #print("whipping prospect left:", prospectLeftCone[2])
                tempConVal = prospectLeftCone[2][0]
                prospectLeftCone[2][0] = prospectLeftCone[2][1]
                prospectLeftCone[2][1] = tempConVal
                prospectRightConeConnectionIndex = 0
                self.logFileChanged = True #set flag
            
            if(prospectRightConeConnectionIndex == -1):
                print("BIG issue: lastRightCone pointed to this prospect cone, but this prospect cone does not point back", lastRightCone[2], prospectRightCone[2])
            elif(prospectRightConeConnectionIndex == 1): #prospect cone has its connections switched around (lastRightCone's 'front' should connect to prospectRightCone's 'back')
                #print("whipping prospect right:", prospectRightCone[2])
                tempConVal = prospectRightCone[2][0]
                prospectRightCone[2][0] = prospectRightCone[2][1]
                prospectRightCone[2][1] = tempConVal
                prospectRightConeConnectionIndex = 0
                self.logFileChanged = True #set flag
            
            prospectLeftConePerpAngle = (radMidd(prospectLeftCone[2][0][2], prospectLeftCone[2][1][2]) if (prospectLeftCone[2][intBoolInv(prospectLeftConeConnectionIndex)][1] >= 0) else radRoll(prospectLeftCone[2][prospectLeftConeConnectionIndex][2] + (np.pi*(-0.5 if (prospectLeftConeConnectionIndex==1) else 0.5))))
            prospectRightConePerpAngle = (radMidd(prospectRightCone[2][1][2], prospectRightCone[2][0][2]) if (prospectRightCone[2][intBoolInv(prospectRightConeConnectionIndex)][1] >= 0) else radRoll(prospectRightCone[2][prospectRightConeConnectionIndex][2] + (np.pi*(0.5 if (prospectRightConeConnectionIndex==1) else -0.5))))
            #note: currently i am using radInv() when calculating angle delta, because angles are from left cone to right cone, but if you reverse radMidd(lastRightCone[2][1][2], lastRightCone[2][0][2]) to be radMidd(lastRightCone[2][0][2], lastRightCone[2][1][2]) it will give an inverted angle already. less human, more efficient
            
            self.debugLines = [] #clear debugLines
            self.debugLines.append([1, self.realToPixelPos(lastLeftCone[1]), [4, lastLeftConePerpAngle], 1])
            self.debugLines.append([1, self.realToPixelPos(prospectLeftCone[1]), [4, prospectLeftConePerpAngle], 1])
            self.debugLines.append([1, self.realToPixelPos(lastRightCone[1]), [4, lastRightConePerpAngle], 2])
            self.debugLines.append([1, self.realToPixelPos(prospectRightCone[1]), [4, prospectRightConePerpAngle], 2])
            
            ## all of should really be in a forloop of some kind, but fuck it; manual it is
            strengths = [1,1,1] #4 possible path lines, one of which already exists (between lastLeftCone and lastRightCone), so calculate the strengths for the remaining three possible pathlines
            pathWidths = [0,0,0];   pathAngles = [0,0,0]
            pathWidths[0], pathAngles[0] = distAngleBetwPos(lastLeftCone[1], prospectRightCone[1]) #last left to next right (for left (CCW) corners, where one left cone (lastLeftCone) connects to several right cones  (lastRightCone AND prospectRightCone))
            strengths[0] *= 1.5-(min(pathWidths[0], self.pathConnectionThreshold)/self.pathConnectionThreshold) #strength based on distance, the threshold just determines minimum score, distance can be larger than threshold without math errors
            strengths[0] *= 1.5-(min(abs(radDiff(pathAngles[0], lastLeftConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta) #strength based on angle delta from lastLeftConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
            strengths[0] *= 1.5-(min(abs(radDiff(radInv(pathAngles[0]), prospectRightConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta) #strength based on angle delta from prospectRightConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
            pathWidths[1], pathAngles[1] = distAngleBetwPos(prospectLeftCone[1], lastRightCone[1]) #last right to next left (for right (CW) corners, where one right cone (lastRightCone) connects to several left cones (lastLeftCone AND prospectLeftCone))
            strengths[1] *= 1.5-(min(pathWidths[1], self.pathConnectionThreshold)/self.pathConnectionThreshold)
            strengths[1] *= 1.5-(min(abs(radDiff(pathAngles[1], prospectLeftConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta)
            strengths[1] *= 1.5-(min(abs(radDiff(radInv(pathAngles[1]), lastRightConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta)
            pathWidths[2], pathAngles[2] = distAngleBetwPos(prospectLeftCone[1], prospectRightCone[1]) #prospect cones to each other (for straight sections of track, lekker simpel)
            strengths[2] *= 1.5-(min(pathWidths[2], self.pathConnectionThreshold)/self.pathConnectionThreshold)
            strengths[2] *= 1.5-(min(abs(radDiff(pathAngles[2], prospectLeftConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta)
            strengths[2] *= 1.5-(min(abs(radDiff(radInv(pathAngles[2]), prospectRightConePerpAngle)), self.pathConnectionMaxAngleDelta)/self.pathConnectionMaxAngleDelta)
            maxStrengthIndex, maxStrengthVal = maxIndex(strengths)
            print("we have a path winner:", maxStrengthIndex, "at strength:", round(maxStrengthVal, 2))
            carAngle = radRoll(pathAngles[maxStrengthIndex] + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
            ## the next section could especially benefit from a forloop, as none of these values are in lists/arrays and they absolutely could be. At least it is slightly legible, i guess
            if(maxStrengthIndex == 0):
                winningLeftCone = lastLeftCone;  winningRightCone = prospectRightCone
                winningLeftConeIndex = lastPathLine[3][2];  winningRightConeIndex = lastRightCone[2][lastRightConeConnectionIndex][1]
            elif(maxStrengthIndex == 1):
                winningLeftCone = prospectLeftCone;  winningRightCone = lastRightCone
                winningLeftConeIndex = lastLeftCone[2][lastLeftConeConnectionIndex][1];  winningRightConeIndex = lastPathLine[4][2]
            elif(maxStrengthIndex == 2):
                winningLeftCone = prospectLeftCone;  winningRightCone = prospectRightCone
                winningLeftConeIndex = lastLeftCone[2][lastLeftConeConnectionIndex][1];  winningRightConeIndex = lastRightCone[2][lastRightConeConnectionIndex][1]
            else: #should never happen
                print("OH GOD NO")
            #check if you've gone full circle
            if((winningLeftCone[0] == self.pathList[0][3][0]) and (winningRightCone[0] == self.pathList[0][4][0])):
                print("path full circle (from winning cones)")
                self.pathFullCircle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            else:
                centerPoint = [winningRightCone[1][0] + (winningLeftCone[1][0]-winningRightCone[1][0])/2, winningRightCone[1][1] + (winningLeftCone[1][1]-winningRightCone[1][1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
                self.pathList.append([centerPoint, [pathAngles[maxStrengthIndex], carAngle], pathWidths[maxStrengthIndex], [winningLeftCone[0], winningLeftCone[1], winningLeftConeIndex], [winningRightCone[0], winningRightCone[1], winningRightConeIndex], maxStrengthVal])
        return(True)
    
    def addCone(self, leftOrRight, pos, coneData=[], connections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], connectNewCone=True, reconnectOverlappingCone=False): #note: left=False, right=True
        isNewCone = True; returnConeID = 0; indexInTotalList = 0; indexInLRlist=0; returnLeftOrRight=leftOrRight; returnPos=pos; returnConnections=connections; returnConeData=coneData #init vars
        overlapsCone, overlappingConeData = self.overlapConeCheck(pos) #check if the new cone overlaps with an existing cone
        if(overlapsCone): #if the new cone overlaps an existing cone
            isNewCone = False
            returnConeID = overlappingConeData[0]
            indexInTotalList = findIndexBy2DEntry(self.totalConeList, 0, overlappingConeData[0])
            indexInLRlist = overlappingConeData[2]
            returnLeftOrRight = overlappingConeData[1]
            returnPos = [item for item in (self.rightConeList[indexInLRlist][1] if returnLeftOrRight else self.leftConeList[indexInLRlist][1])] #copy data (dont copy pointer)
            #returnConnections = [[subItem for subItem in item] for item in (self.rightConeList[indexInLRlist][2] if returnLeftOrRight else self.leftConeList[indexInLRlist][2])] #copy data (dont copy pointer)
            realConnections = self.rightConeList[indexInLRlist][2] if returnLeftOrRight else self.leftConeList[indexInLRlist][2] #copy pointer, sothat it can be edited if reconnectOverlappingCone
            if(reconnectOverlappingCone): #really just for mouse-based UI, clicking on an existing cone will make it attempt to connect
                realConnections = self.connectCone(returnConeID, returnPos, returnLeftOrRight, indexInLRlist, realConnections, 1) #fill in (ID, pos, leftOrRight, index in left/right, currentConnections
                if(returnLeftOrRight):
                    self.rightConeList[indexInLRlist][2] = realConnections #put new connections back into array (this can all be made easier by not copying things, but it works for now
                else:
                    self.leftConeList[indexInLRlist][2] = realConnections #put new connections back into array (this can all be made easier by not copying things, but it works for now
            returnConnections = [[subItem for subItem in item] for item in realConnections] #copy data (dont copy pointer)
            returnConeData = self.coneDataCopy(self.rightConeList[indexInLRlist][3] if returnLeftOrRight else self.leftConeList[indexInLRlist][3]) #copy data (dont copy pointer)
        else:
            isNewCone = True
            returnConeID = self.newConeID
            if(connectNewCone):
                returnConnections = self.connectCone(returnConeID, pos, leftOrRight, len(self.rightConeList) if leftOrRight else len(self.leftConeList), connections, 0) #little tricky: technically the left/right list index that the new cone is in doesnt exist yet, but it is about to be added to the end, so current list length = index. MAY NOT work if multithreaded
            if(leftOrRight): #if right
                self.rightConeList.append([returnConeID, [item for item in pos], [[subItem for subItem in item] for item in returnConnections], self.coneDataCopy(coneData)]) #copy data (dont copy pointer)
                indexInLRlist = len(self.rightConeList)-1
            else:            #if left
                self.leftConeList.append([returnConeID, [item for item in pos], [[subItem for subItem in item] for item in returnConnections], self.coneDataCopy(coneData)]) #copy data (dont copy pointer)
                indexInLRlist = len(self.leftConeList)-1
            self.totalConeList.append([returnConeID, leftOrRight, indexInLRlist])
            if(self.logging):
                self.logCone(returnConeID, leftOrRight, pos, returnConnections, coneData)
            self.newConeID += 1
        return(isNewCone, returnConeID, indexInTotalList, indexInLRlist, returnLeftOrRight, returnPos, returnConnections, returnConeData) #is new cone, coneID, index in totalConeList, index in leftConeList/rightConeList
    
    def addCar(self, pos=[4.0, 8.0], orient=0.0, color=[50,200,50]):
        self.cars.append(raceCar(pos, orient, color))
        return(len(self.cars)-1) #return the length of the cars list minus 1, because that is the list index of this new car
    
    def setFinishCone(self, leftOrRight, pos, coneData=[CD_FINISH]): #note: left=False, right=True
        #check if the requested position already has a cone
        addConeResult = self.addCone(leftOrRight, pos, coneData, connectNewCone=True, reconnectOverlappingCone=True) #attempt to add new cone, if a cone is already at that position, addCone() will return that info
        if(not addConeResult[0]): #if this is True, a new cone was added
            print("setting finish on existing cone with ID:", addConeResult[1], ("(right)" if addConeResult[4] else "(left)"))
            for coneDataEntry in coneData: #for all coneData identifiers
                if not (coneDataEntry in addConeResult[6]): #if it's not already in there
                    if(addConeResult[4]): #left/right
                        self.rightConeList[addConeResult[3]][3].append(coneDataEntry)
                    else:
                        self.leftConeList[addConeResult[3]][3].append(coneDataEntry)
            #self.rewriteLogfile()
            self.logFileChanged = True #set flag
        self.finishLinePos.append([addConeResult[1], addConeResult[5], addConeResult[4]]) # [cone ID, [x,y], left/right]
        if(len(self.finishLinePos) > 2):
            print("too many finish line points")
    
    #drawing funtions
    def background(self):
        self.window.fill(self.bgColor, (self.drawPosX, self.drawPosY, self.drawWidth, self.drawHeight))
    
    def drawCones(self, drawLines=True):
        conePixelDiam = self.coneDiam * self.sizeScale
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        for cone in self.totalConeList: #cone is an array with [cone ID, left/right, index in left/right array]
            conePos = self.rightConeList[cone[2]][1] if cone[1] else self.leftConeList[cone[2]][1]
            if(self.isInsideWindowReal(conePos)): #if it is within bounds, draw it
                conePos = self.realToPixelPos(conePos) #convert to pixel positions
                coneColor = self.rightConeColor if cone[1] else self.leftConeColor
                if(drawLines):
                    coneConnections = self.rightConeList[cone[2]][2] if cone[1] else self.leftConeList[cone[2]][2]
                    connectionsFilled = [(coneConnections[0][1] >= 0), (coneConnections[1][1] >= 0)] #2-size list of booleans
                    alreadyDrawn = [False, False]
                    for drawnLine in drawnLineList:
                        for i in range(2):
                            if(connectionsFilled[i]):
                                if(((cone[0] == drawnLine[0]) and (coneConnections[i][0] == drawnLine[1])) or ((cone[0] == drawnLine[1]) and (coneConnections[i][0] == drawnLine[0]))):
                                    alreadyDrawn[i] = True
                    for i in range(2):
                        if(connectionsFilled[i] and (not alreadyDrawn[i])): #if the 'back' conenction isnt already drawn
                            pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(self.rightConeList[coneConnections[i][1]][1] if cone[1] else self.leftConeList[coneConnections[i][1]][1]), self.coneLineWidth)
                            drawnLineList.append([cone[0], coneConnections[i][0]]) #put established 'back' connection in list of drawn lines
                #pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
                conePos = ASA(-(conePixelDiam/2), conePos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
    
    def drawPathLines(self, drawConeLines=True, drawCenterPoints=False):
        # pathList content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        pathCenterPixelDiam = self.coneDiam * self.sizeScale
        for i in range(len(self.pathList)):
            if(self.isInsideWindowReal(self.pathList[i][0])):
                if(drawCenterPoints):
                    #centerPixelPos = self.realToPixelPos(self.pathList[i][0])
                    #pygame.draw.circle(self.window, self.pathColor, [int(centerPixelPos[0]), int(centerPixelPos[1])], int(pathCenterPixelDiam/2)) #draw center point (as filled circle, not ellipse)
                    pygame.draw.ellipse(self.window, self.pathColor, [ASA(-(pathCenterPixelDiam/2), self.realToPixelPos(self.pathList[i][0])), [pathCenterPixelDiam, pathCenterPixelDiam]]) #draw center point
                if(drawConeLines):
                    pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.pathList[i][3][1]), self.realToPixelPos(self.pathList[i][4][1]), self.pathLineWidth) #line from left cone to right cone
                if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                    #draw line between center points of current pathline and previous pathline (to make a line that the car should (sort of) follow)
                    pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.pathList[i-1][0]), self.realToPixelPos(self.pathList[i][0]), self.pathLineWidth) #line from center pos to center pos
        if(self.pathFullCircle):
            pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.pathList[-1][0]), self.realToPixelPos(self.pathList[0][0]), self.pathLineWidth) #line that loops around to start
    
    def drawFinishLine(self):
        if(len(self.finishLinePos) == 2):
            pygame.draw.line(self.window, self.finishLineColor, self.realToPixelPos(self.finishLinePos[0][1]), self.realToPixelPos(self.finishLinePos[1][1]), self.finishLineWidth)
    
    def drawCars(self):
        for car in self.cars:
            car.draw(self)
            #car.draw(self.window, self.sizeScale, self.drawWidth, self.drawHeight, self.drawPosX, self.drawPosY) #old style
    
    def drawFloatingCone(self, drawPossibleConnections=True, drawConnectionThresholdCircle=False):
        if(len(self.floatingCone) == 2): #if there is a floating cone to be drawn
            conePixelDiam = self.coneDiam * self.sizeScale
            self.floatingCone[0] = pygame.mouse.get_pos() #update position to match mouse position
            if(self.isInsideWindowPixels(self.floatingCone[0])):
                coneColor = self.rightConeColor if self.floatingCone[1] else self.leftConeColor
                if(drawConnectionThresholdCircle):
                    pygame.draw.circle(self.window, coneColor, [int(self.floatingCone[0][0]), int(self.floatingCone[0][1])], self.coneConnectionThreshold * self.sizeScale, self.coneLineWidth) #draw circle with coneConnectionThreshold radius 
                overlapsCone, overlappingConeData = self.overlapConeCheck(self.pixelsToRealPos(self.floatingCone[0]))
                if(overlapsCone and drawPossibleConnections): #if mouse is hovering over existing cone
                    ##overlappingConeData is the data from the entry in totalConeList with format: [cone ID, left/right, index in left/right array]
                    overlappingConeDataExt = self.rightConeList[overlappingConeData[2]] if overlappingConeData[1] else self.leftConeList[overlappingConeData[2]] #note: dont copy data, just copy pointer to entry
                    ##overlappingConeDataExt is the data from the entry in left/right-ConeList with format: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]
                    nearbyConeList = self.distanceToConeSquared(overlappingConeDataExt[1], [self.rightConeList if overlappingConeData[1] else self.leftConeList], False, True, [overlappingConeData[0]], self.coneConnectionThresholdSquared, True, [])
                    overlappingConePixelPos = self.realToPixelPos(overlappingConeDataExt[1])
                    for cone in nearbyConeList:
                        pygame.draw.line(self.window, coneColor, overlappingConePixelPos, self.realToPixelPos(cone[1]), int(self.coneLineWidth/2))
                else:
                    #pygame.draw.circle(self.window, coneColor, [int(self.floatingCone[0][0]), int(self.floatingCone[0][1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
                    conePos = ASA(-(conePixelDiam/2), self.floatingCone[0]) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                    pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
                    if(drawPossibleConnections):
                        nearbyConeList = self.distanceToConeSquared(self.pixelsToRealPos(self.floatingCone[0]), [self.rightConeList if self.floatingCone[1] else self.leftConeList], False, True, [], self.coneConnectionThresholdSquared, True, [])
                        # ## debug
                        # nearbyConeList = self.distanceToCone(self.pixelsToRealPos(self.floatingCone[0]), [self.rightConeList if self.floatingCone[1] else self.leftConeList], DONT_SORT, True, [], self.coneConnectionThreshold, True, [], 0.0, someAngles)
                        # someAngles = [-self.coneConnectionMaxAngleDelta, self.coneConnectionMaxAngleDelta]
                        # self.debugLines = []
                        # self.debugLines.append([1, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[0]], 0]) #line from floating cone at someAngles[0] radians with a length of coneConnectionThreshold
                        # self.debugLines.append([1, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[1]], 0]) #line from floating cone at someAngles[1] radians with a length of coneConnectionThreshold
                        # self.debugLines.append([2, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[0], someAngles[1]], 0]) #arc centered at floating cone with radius coneConnectionThreshold, startAngle being someAngles[0] radians and endAngle being someAngles[1] radians
                        # ## end debug
                        for cone in nearbyConeList:
                            pygame.draw.line(self.window, coneColor, self.floatingCone[0], self.realToPixelPos(cone[1]), int(self.coneLineWidth/2))
    
    def drawDebugLines(self):
         #debugLines structure: [pos,pos, color_index (0-2)]
        for debugLine in self.debugLines:
            if(abs(debugLine[0]) == 2):
                if(debugLine[0] == 2):
                    pixelRactSize = debugLine[2][0] * self.sizeScale
                    debugLine[1] = ASA(-(pixelRactSize), debugLine[1])
                    pixelRactSize *= 2
                    debugLine[2][0] = pixelRactSize
                    debugLine[0] = -2
                pygame.draw.arc(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], [debugLine[1], [debugLine[2][0], debugLine[2][0]]], debugLine[2][1], debugLine[2][2], self.debugLineWidth)
            else:
                if(debugLine[0] == 1):
                    secondPos = self.realToPixelPos(distAnglePosToPos(debugLine[2][0], debugLine[2][1], self.pixelsToRealPos(debugLine[1]))) #convert
                    debugLine[2] = secondPos
                    debugLine[0] = -1
                pygame.draw.line(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], debugLine[1], debugLine[2], self.debugLineWidth)
    
    def redraw(self):
        self.background()
        self.drawCones(True) #boolean parameter is whether to draw lines between connected cones (track bounds) or not
        self.drawDebugLines()
        self.drawPathLines(True, True) #boolean parameters are whether to draw the lines between cones (not the line the car follows) and whether to draw circles (conesized ellipses) on the center points of path lines respectively
        self.drawFinishLine()
        self.drawCars()
        self.drawFloatingCone(True, False)
        if(self.logging and (pygame.time.get_ticks() > self.rewriteLogTimer) and self.logFileChanged):
            self.rewriteLogfile()
    
    def updateWindowSize(self, drawWidth, drawHeight, drawPosX=0, drawPosY=0, sizeScale=-1, autoMatchSizeScale=True):
        if(sizeScale > 0):
            self.sizeScale = sizeScale
        elif(autoMatchSizeScale):
            self.sizeScale = min(drawWidth/self.drawWidth, drawHeight/self.drawHeight) * self.sizeScale #auto update sizeScale to match previous size
        self.drawWidth = drawWidth
        self.drawHeight = drawHeight
        if(self.drawPosX != drawPosX):
            self.drawPosX = int(drawPosX)
        if(self.drawPosY != drawPosY):
            self.drawPosY = int(drawPosY)
    


#cursor in the shape of a flag
# flagCurs = ("XXX         XXXXXXXXX   ",
#             "XX XXXXXXXXX...XXXX..XXX",
#             "XX ....XXXX....XXXX....X",
#             "XX ....XXXX....XXXX....X",
#             "XX ....XXXX.XXX....XX..X",
#             "XX XXXX....XXXX....XXXXX",
#             "XX XXXX....XXXX....XXXXX",
#             "XX XXXX....X...XXXX..XXX",
#             "XX ....XXXX....XXXX....X",
#             "XX ....XXXX....XXXX....X",
#             "XXX....XXXX.XXXXXXXXX..X",
#             "XX XXXXXXXXX         XXX",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ",
#             "XX                      ")
flagCurs = ("ooo         ooooooooo   ",
            "oo ooooooooo...XXXX..ooo",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX.XXX....XX..o",
            "oo XXXX....XXXX....XXXXo",
            "oo XXXX....XXXX....XXXXo",
            "oo XXXX....X...XXXX..XXo",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX....XXXX....o",
            "ooo....XXXX.ooooooooo..o",
            "oo ooooooooo         ooo",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ")
# flagCurs16  =  ("XXXXXXXXXXXXXXXX", #1
#                 "XX ...XXX...XXXX",
#                 "XX ...XXX...XXXX",
#                 "XX XXX...XXX...X", #4
#                 "XX XXX...XXX...X",
#                 "XX ...XXX...XXXX",
#                 "XX ...XXX...XXXX",
#                 "XX XXX...XXX...X", #8
#                 "XX XXX...XXX...X",
#                 "XXXXXXXXXXXXXXXX",
#                 "XX              ",
#                 "XX              ", #12
#                 "XX              ",
#                 "XX              ",
#                 "XX              ",
#                 "XX              ") #16
flagCurs16  =  ("oooooooooooooooo", #1
                "oo ...XXX...XXXo",
                "oo ...XXX...XXXo",
                "oo XXX...XXX...o", #4
                "oo XXX...XXX...o",
                "oo ...XXX...XXXo",
                "oo ...XXX...XXXo",
                "oo XXX...XXX...o", #8
                "oo XXX...XXX...o",
                "oooooooooooooooo",
                "oo              ",
                "oo              ", #12
                "oo              ",
                "oo              ",
                "oo              ",
                "oo              ") #16
global normalCursData, flagCurs24Data, flagCurs16Data
flagCurs24Data = ((24,24),(0,23)) + pygame.cursors.compile(flagCurs, 'X', '.', 'o')
flagCurs16Data = ((16,16),(0,15)) + pygame.cursors.compile(flagCurs16, 'X', '.', 'o')
normalCursData = []

global windowKeepRunning, windowStarted
windowStarted = False
windowKeepRunning = False

def shouldKeepRunning():
    global windowKeepRunning
    return(windowKeepRunning)

def pygameInit():
    pygame.init()
    pygame.font.init()
    global window
    window = pygame.display.set_mode([1200, 600], pygame.RESIZABLE)
    pygame.display.set_caption("(pygame) selfdriving sim")
    global normalCursData
    normalCursData = pygame.mouse.get_cursor() #remember the normal cursor (because we'll change the cursor later
    global windowKeepRunning, windowStarted
    windowStarted = True
    windowKeepRunning = True

def pygameEnd():
    global windowStarted
    if(windowStarted): #if the window never started, quit might error out or something stupid
        print("quitting pygame window...")
        pygame.quit()

def frameRefresh():
    pygame.display.flip() #send (finished) frame to display

def handleMousePress(pygamesimInput, buttonDown, button, pos, eventToHandle):
    if(button==1): #left mouse button
        if(buttonDown): #mouse pressed down
            pygamesimInput.floatingCone = [pos, False]
            pygame.event.set_grab(1)
            if(pygame.key.get_pressed()[102]):
                pygame.mouse.set_cursor(flagCurs16Data[0], flagCurs16Data[1], flagCurs16Data[2], flagCurs16Data[3])
        else:           #mouse released
            pygame.event.set_grab(0)
            pygamesimInput.floatingCone = []
            if(pygame.key.get_pressed()[102]):
                print("placing finish")
                pygamesimInput.setFinishCone(False, pygamesimInput.pixelsToRealPos(pos))
                pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
            else:
                pygamesimInput.addCone(False, pygamesimInput.pixelsToRealPos(pos), connectNewCone=True, reconnectOverlappingCone=True)
    if(button==3): #left mouse button
        if(buttonDown): #mouse pressed down
            pygamesimInput.floatingCone = [pos, True]
            pygame.event.set_grab(1)
            if(pygame.key.get_pressed()[102]):
                pygame.mouse.set_cursor(flagCurs16Data[0], flagCurs16Data[1], flagCurs16Data[2], flagCurs16Data[3])
        else:           #mouse released
            pygame.event.set_grab(0)
            pygamesimInput.floatingCone = []
            if(pygame.key.get_pressed()[102]):
                pygamesimInput.setFinishCone(True, pygamesimInput.pixelsToRealPos(pos))
                pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
            else:
                pygamesimInput.addCone(True, pygamesimInput.pixelsToRealPos(pos), connectNewCone=True, reconnectOverlappingCone=True)

def handleKeyPress(pygamesimInput, keyDown, key, keyName, eventToHandle):
    if(key==102): # f
        if(keyDown):
            pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
        else:
            pygame.mouse.set_cursor(normalCursData[0], normalCursData[1], normalCursData[2], normalCursData[3])
    elif(key==114): # r
        if(keyDown):
            pygamesimInput.makePath()
            #pygamesimInput.cars[0].orient += 0.1 #turn first (only) car 0.1 radians
    elif(key==108): # l
        if(keyDown):
            pygamesimInput.rewriteLogfile()

def handleWindowEvent(pygamesimInput, eventToHandle):
    if(eventToHandle.type == pygame.QUIT):
        global windowKeepRunning
        windowKeepRunning = False #stop program (soon)
        
    elif(eventToHandle.type == pygame.VIDEORESIZE):
        if not ((pygame.display.Info().current_w == eventToHandle.size[0]) and (pygame.display.Info().current_h == eventToHandle.size[1])): #if new size is actually different
            newWidth = pygame.display.Info().current_w #init var
            newHeight = pygame.display.Info().current_h#init var
            aspectRatio = round(newWidth/newHeight,2) #easier than grabbing a global var
            print("video resize from", [newWidth, newHeight], "to", eventToHandle.size)
            if(newWidth == eventToHandle.size[0]): # only width changed
                newWidth = int(eventToHandle.size[1] * aspectRatio) # new height * ratio = matching width
                newHeight = eventToHandle.size[1]
            elif(newHeight == eventToHandle.size[1]):
                newWidth = int(eventToHandle.size[0] - (eventToHandle.size[0] % aspectRatio))
                newHeight = int(newWidth / aspectRatio) # new width / ratio = matching height
            else:
                newWidth = int(min(eventToHandle.size[0], eventToHandle.size[1]*2))
                newHeight = int(min(eventToHandle.size[1], newWidth/2))
            global window
            window = pygame.display.set_mode([newWidth, newHeight], pygame.RESIZABLE)
            pygamesimInput.updateWindowSize(newWidth, newHeight, autoMatchSizeScale=True)
    
    elif((eventToHandle.type == pygame.MOUSEBUTTONDOWN) or (eventToHandle.type == pygame.MOUSEBUTTONUP)):
        #print("mouse press", eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos)
        handleMousePress(pygamesimInput, eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        
    elif((eventToHandle.type == pygame.KEYDOWN) or (eventToHandle.type == pygame.KEYUP)):
        #print("keypress:", eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key))
        handleKeyPress(pygamesimInput, eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key), eventToHandle)

def handleAllWindowEvents(pygamesimInput):
    for eventToHandle in pygame.event.get():
        handleWindowEvent(pygamesimInput, eventToHandle)




def main():
    pygameInit()
    
    sim1 = pygamesim(window)
    sim1.addCar()
    
    global windowKeepRunning
    while windowKeepRunning:
        handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        sim1.redraw()
        pygame.display.flip() #draw display
    
    print("closing logging file(s)...")
    sim1.closeLog()
    pygameEnd()
    

if __name__ == '__main__':
    main()