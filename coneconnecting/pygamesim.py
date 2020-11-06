#TBD: splitscreen, importing cone log files, 

#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import pygame
import numpy as np
import datetime
import sys

# Array Scalar Multiplication and Addition
global ASM, ASA
ASM = lambda scalar, inputArray : [scalar * entry for entry in inputArray]
ASA = lambda scalar, inputArray : [scalar + entry for entry in inputArray]
intBoolInv = lambda inputInt : (0 if (inputInt>0) else 1)

#angle rollover functions (for 2d positioning systems, given that arctan2 outputs between (-pi, pi))
degRollTwo = lambda angle : ((angle % -360) if (angle < 0) else (angle % 360))
degRollOne = lambda angle : ((angle % -180) if (angle > 180) else ((angle % 180) if (angle < -180) else angle))
degRoll = lambda angle : degRollOne(degRollTwo(angle))
radRollTwo = lambda angle : ((angle % (-2*np.pi)) if (angle < 0) else (angle % (2*np.pi)))
radRollOne = lambda angle : ((angle % -np.pi) if (angle > np.pi) else ((angle % np.pi) if (angle < -np.pi) else angle))
radRoll = lambda angle : radRollOne(radRollTwo(angle))
#angle math funcitons, that incorporate rollover
degDiff = lambda angleOne, angleTwo : degRoll(angleTwo-angleOne)  #recommend using abs(degDiff()), but currently, it returns the value required to get from angleOne to angleTwo, so (90, -45) = -135
radDiff = lambda angleOne, angleTwo : radRoll(angleTwo-angleOne)  #recommend using abs(radDiff()), but currently, it returns the value required to get from angleOne to angleTwo, so (pi/2, -pi/4) = -3pi/4
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

global CD_FINISH, coneLogTableColumnDef
CD_FINISH = 'finish'
coneLogTableColumnDef = "cone ID,leftOrRight,Xpos,Ypos,prev ID,next ID,coneData\n"

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
    def __init__(self, window, cars=[], drawWidth=1200, drawHeight=600, drawPosX=0, drawPosY=0, sizeScale=30, invertYaxis=True, importConeLogFilename='', logging=True, logname="pygamesim"):
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
            global coneLogTableColumnDef
            self.logfile.write(coneLogTableColumnDef)
            self.rewriteLogTimer = pygame.time.get_ticks() + 2000
        ## now init all other variables/lists
        self.bgColor = [50,50,50] #grey
        
        self.finishLineColor = [255,40,0]
        self.finishLineWidth = 2 #pixels wide
        self.finishLinePos = [] #[ [cone ID, [x, y], left/right], [cone ID, [x, y], left/right]]
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneLineWidth = 2 #pixels wide
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        #self.pathCenterPixelDiam = 
        
        self.coneDiam = 0.2 #meters
        #self.conePixelDiam = coneDiam * sizeScale #might make math faster, but will require re-calculating on change
        
        self.coneConnectionThreshold = 5  #in meters (or at least not pixels)  note: hard threshold beyond which cones will NOT come into contention for connection
        self.coneConnectionThresholdSquared = self.coneConnectionThreshold**2
        self.coneConnectionHighAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.coneConnectionMaxAngleDelta = np.deg2rad(120) #if the angle difference is larger than this, it just doesnt make sense to connect them. (this IS a hard threshold)
        
        self.pathConnectionThreshold = 10 #in meters (or at least not pixels)  IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.pathConnectionMaxAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        
        self.pathFirstLineCarAngleDeltaMax = np.deg2rad(45) #if the radDiff() between car (.orient) and the first line's connections is bigger than this, switch conneections or stop
        
        self.leftConeList = []  #[ [cone ID, [x,y], [[cone ID, index (left), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
        self.rightConeList = [] #[ [cone ID, [x,y], [[cone ID, index (right), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
        self.pathList = [] #[[center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength], ]
        # #the pathList is a (properly ordered) list of points/lines for the car to travel through. It (like most lists here) prioratizes math efficiency over RAM usage, so several stored parameters are redundant (can be recalculated given other elements), which should save time when calculating optimal path.
        self.newConeID = 0 #add 1 after adding a cone
        
        #self.leftConesFullCircle = False  #TBD: no checking function yet, and because connectCone() can connect a cone to the 0th cone at any time without completing the circle, a special function this is required
        #self.rightConesFullCircle = False
        self.pathFullCircle = False
        
        self.floatingCone = [] #[ [[xpixel,ypixel], color] ] #note: x and y are in pixels, not meters. Translation to meters happens on final placement
        
        self.rewriteLogTimer = 0
        self.logFileChanged = False
        
        self.debugLines = [] #[[lineType, pos, pos/angles, color_index (0-2)], ] #third entry is: (lineType==0: straight line from two positions), (lineType==1: straight line from pos and [radius, angle]), (lineType==2: arc from pos and [radius, startAngle, endAngle])
        self.debugLineColors = [[255,0,255],[255,255,255],[0,255,0]] #purple, white, green
        self.debugLineWidth = 3
        
        #this has to happen AFTER variables/lists are initialized, (becuase it's hard to append to nonexistant conelists)
        if((len(importConeLogFilename) > 0) and (importConeLogFilename != '') and (importConeLogFilename != "")):
            self.importConeLog(importConeLogFilename, True)
    
    def closeLog(self):
        if(self.logging): #just a safety check
            self.logfile.close()
    
    def coneDataToString(self, coneData):
        resultString = "" if (len(coneData)==0) else str(coneData).replace(',',';') #TBD
        return(resultString)
    
    def stringToConeData(self, coneDataString):
        if((coneDataString == '') or (coneDataString == "") or (len(coneDataString) < 1)):
            return([])
        else:
            coneDataStringList = coneDataString.strip().split(';')
            returnConeData = []
            for i in range(len(coneDataStringList)):
                #if((coneDataStringList[i].count['['] > 0) and (coneDataStringList[i].count['['] == coneDataStringList[i].count[']'])): #nesteld lists
                returnConeData.append(coneDataStringList[i])
            return(returnConeData)
    
    def coneDataCopy(self, coneData):
        copyOfConeData = []
        for entry in coneData:
            #check for special situations here (multi-dimensional arrays or lists within lists)
            #if it's just a regular entry, copy it
            copyOfConeData.append(entry)
        return(copyOfConeData)
    
    def logCone(self, leftOrRight, coneID, pos, connections, coneData):
        if(self.logging): #just a safety check
            leftOrRightString = "RIGHT" if leftOrRight else "LEFT"
            self.logfile.write(str(coneID) +','+ leftOrRightString +','+ str(round(pos[0], 2)) +','+ str(round(pos[1], 2)) +','+ str(connections[0][0]) +','+ str(connections[1][0]) +','+ self.coneDataToString(coneData) + '\n')
    
    def rewriteLogfile(self):
        if(self.logging): #just a safety check
            #print("rewriting log file")
            self.logfile.close() #close last file (because if anything goes wrong, that data is not lost
            self.logfile = open(self.logfilename + ".csv", "w")
            global coneLogTableColumnDef
            self.logfile.write(coneLogTableColumnDef)
            combinedConeList = (self.rightConeList + self.leftConeList)
            rightListLength = len(self.rightConeList)
            for i in range(len(combinedConeList)):
                self.logCone((i < rightListLength), combinedConeList[i][0], combinedConeList[i][1], combinedConeList[i][2], combinedConeList[i][3]) #rewrite all data (some of which is updated)
            self.rewriteLogTimer = pygame.time.get_ticks() + 1000
            self.logFileChanged = False #reset flag
    
    def importConeLog(self, filename, keepExistingData=False):
        if(not keepExistingData):
            self.rightConeList.clear() #does not clear subArrays, but garbage collector should take care of it, i guess
            self.leftConeList.clear()
            self.newConeID = 0
        try:
            if('.' not in filename):
                print("(importConeLog) filename doesn't contain a '.', so i'm adding '.csv' to the end")
                filename += '.csv'
            readFile = open(filename, 'r')
            global coneLogTableColumnDef
            if(readFile.readline() == coneLogTableColumnDef): #alternatively, you could try to find columns (column names) in the first line and determine the indexes of data in lineArray
                discardedLines = 0
                highestImportedConeID = 0
                for line in readFile:
                    lineArray = line.strip().split(',')
                    lineData = [0, False, 0.0, 0.0, -1, -1, []] #init var
                    if(len(lineArray) >= 7):
                        if(lineArray[0].isnumeric() and ((lineArray[1].upper() == 'LEFT') or (lineArray[1].upper() == 'RIGHT')) and (lineArray[4].isnumeric() or (lineArray[4]=='-1')) and (lineArray[5].isnumeric() or (lineArray[5]=='-1'))): #note: cant check floats
                            try:
                                lineData[0] = int(lineArray[0])
                                lineData[1] = (lineArray[1].upper() == 'RIGHT')
                                lineData[2] = float(lineArray[2])
                                lineData[3] = float(lineArray[3])
                                lineData[4] = int(lineArray[4])
                                lineData[5] = int(lineArray[5])
                                lineData[6] = self.stringToConeData(lineArray[6].strip())
                                ## add a value to all coneIDs to allow for existing data to remain
                                highestImportedConeID = max(lineData[0], highestImportedConeID)
                                lineData[0] = ((lineData[0]+self.newConeID) if (lineData[0] >= 0) else -1)
                                lineData[4] = ((lineData[4]+self.newConeID) if (lineData[4] >= 0) else -1)
                                lineData[5] = ((lineData[5]+self.newConeID) if (lineData[5] >= 0) else -1)
                                ## append a coneList
                                listToAppend = (self.rightConeList if lineData[1] else self.leftConeList)
                                listToAppend.append([lineData[0], [lineData[2], lineData[3]], [[lineData[4],-1,0.0,0.0,0.0],[lineData[5],-1,0.0,0.0,0.0]], lineData[6]])
                            except:
                                print("string to numeric conversion faillure ([2] or [3] probably not float)")
                                discardedLines += 1
                        else:
                            print("string to numeric conversion faillure: simple check failed (isnumeric() and LEFT/RIGHT check)")
                            discardedLines += 1
                    else:
                        print("string to numeric conversion faillure: ")
                        discardedLines += 1
                if(discardedLines > 0):
                    print("had to discard", discardedLines, "lines when importing coneLog")
                self.newConeID += highestImportedConeID
                for currentConeList in [self.leftConeList, self.rightConeList]: #for each list seperately
                    for i in range(len(currentConeList)):
                        connections = currentConeList[i][2]
                        for j in range(2):
                            if(connections[j][0] >= 0):
                                ## connection[j] structure: [cone ID, index, angle, distance, cone-connection-strength]
                                connections[j][1] = findIndexBy2DEntry(currentConeList, 0, connections[j][0])
                                if(connections[j][1] < 0): #sanity check
                                    print("couldnt find index in list for connecction!?!:", currentConeList[i])
                                connectedCone = currentConeList[connections[j][1]]
                                distance, angle = distAngleBetwPos(currentConeList[i][1], connectedCone[1])
                                connections[j][2] = angle
                                connections[j][3] = distance
                                connections[j][4] = -1 #connection strength is not imported and cant be (easily) calculated, so just set it to -1 and call it a day
                self.logFileChanged = True #logging shouldnt be on (in which case this will do nothing), but if it is, this is required
            else:
                print("can't import coneLog because the first line is not equal to coneLogTableColumnDef")
            readFile.close()
        except FileNotFoundError:
            print("can't import coneLog because file:", "'" + filename + "'", "wasnt found.")
        except:
            print("exception in importConeLog()")
    
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
    
    def distanceToConeSquared(self, pos, listsToCheck=[], sortByDistance=False, mergeLists=True, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, excludeDoublyConnectedCones=False, ignoreLinkedConeIDs=[]):
        if(len(listsToCheck) < 1):
            listsToCheck = [self.leftConeList, self.rightConeList] #cant be default parameter
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
                        returnCone.append(cone[1]) #pos
                        returnCone.append(cone[2]) #[[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]]
                        returnCone.append(squaredDistance)
                        returnCone.append(coneIndex)
                        returnCone.append(cone[3])
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
    
    def distanceToCone(self, pos, listsToCheck=[], sortBySomething=DONT_SORT, mergeLists=True, ignoreConeIDs=[], simpleThreshold=-1.0, excludeDoublyConnectedCones=False, ignoreLinkedConeIDs=[], angleDeltaTarget=0.0, angleThreshRange=[]): #note: angleThreshRange is [lowBound, upBound]
        if(len(listsToCheck) < 1):
            listsToCheck = [self.leftConeList, self.rightConeList] #cant be default parameter    
        returnList = []  #[[ [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data (certainty, time spotted, etc)], ], ]  #note: coneID of first cone in first conelist is array[0][0][0]
        for conelist in range(len(listsToCheck)):
            returnListPointer = returnList
            if(not mergeLists):
                returnList.append([])
                returnListPointer = returnList[conelist] # makes appending to it easier
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
                        returnCone.append(cone[1]) #pos
                        returnCone.append(cone[2]) #[[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]]
                        returnCone.append([distance, angle])
                        returnCone.append(coneIndex)
                        returnCone.append(cone[3])
                        #store the data in the return list. If 'sortByDistance', then sort it right here, as opposed to afterwards
                        #alternatively, 'listToAppend = returnList if mergeLists else returnList[conelist]' and the just append to that (saves a few lines of code)
                        if(sortBySomething == SORTBY_DIST):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((returnCone[3][0] < returnListPointer[i][3][0]) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((returnCone[3][1] < returnListPointer[i][3][1]) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL_DELT):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((radDiff(returnCone[3][1], angleDeltaTarget) < radDiff(returnListPointer[i][3][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL_DELT_ABS):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((abs(radDiff(returnCone[3][1], angleDeltaTarget)) < abs(radDiff(returnListPointer[i][3][1], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        else:
                            returnListPointer.append(returnCone)
        return(returnList)
    
    
    def overlapConeCheck(self, posToCheck):
        boolAnswer = False;   leftOrRight = False;   indexInLeftRightList = 0;   coneListEntry = [] #boolAnswer MUST default to False, the other variables dont matter as much
        coneDistToler = self.coneDiam*2 #overlap tolerance  NOTE: area is square, not round
        combinedConeList = (self.rightConeList + self.leftConeList)
        rightListLength = len(self.rightConeList)
        for i in range(len(combinedConeList)):
            conePos = combinedConeList[i][1]
            if((posToCheck[0] > (conePos[0]-coneDistToler)) and (posToCheck[0] < (conePos[0]+coneDistToler)) and (posToCheck[1] > (conePos[1]-coneDistToler)) and (posToCheck[1] < (conePos[1]+coneDistToler))):
                boolAnswer = True
                leftOrRight = (i < rightListLength)
                indexInLeftRightList = (i if leftOrRight else (i-rightListLength))
                coneListEntry = combinedConeList[i]
        return(boolAnswer, leftOrRight, indexInLeftRightList, coneListEntry) #leftOrRight and coneListEntry arer only filled if boolAnswer==True
    
    def connectCone(self, coneToConnectID, coneToConnectPos, leftOrRight, coneToConnectIndex, currentConeConnections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], coneToConnectPreferredConnection=1, updateInputConeInList=True, updateWinnerConeInList=True):
        #the correct cone should be selected based on a number of parameters:
        #distance to last cone, angle difference from last and second-to-last cones's, angle that 'track' is (presumably) going based on cones on other side (left/right) (if right cones make corner, left cones must also), etc
        # ideas: distance between last (existing) cone connection may be similar to current cone distance (human may place cones in inner corners close together and outer corners far apart, but at least consistent)
        # ideas: vincent's "most restrictive" angle could be done by realizing the a right corner (CW) is restrictive for left-cones, and a left corner (CCW) is restrictive for right-cones, so whether the angle delta is positive or negative (and leftOrRight boolean) could determine strength
        #all/more parameters to be added later, in non-simulation code
        currentConnectionsFilled = [(currentConeConnections[0][1] >= 0), (currentConeConnections[1][1] >= 0)] #2-size list of booleans
        if(currentConnectionsFilled[0] and currentConnectionsFilled[1]):#if both connection entries are already full
            print("input cone already doubly connected?!:", currentConeConnections)
            return(False, [])
        else:
            currentFrontOrBack = (intBoolInv(coneToConnectPreferredConnection) if (currentConnectionsFilled[coneToConnectPreferredConnection]) else coneToConnectPreferredConnection) # default to coneToConnectPreferredConnection
            currentExistingAngle = radRoll(radInv(currentConeConnections[intBoolInv(currentFrontOrBack)][2])) #if there is not existing connection, this will return radRoll(radInv(0.0)), which is pi (or -pi). only make use of this value if(currentConnectionsFilled[0] or currentConnectionsFilled[1])
            nearbyConeList = self.distanceToCone(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], SORTBY_DIST, True, [coneToConnectID], self.coneConnectionThreshold, True, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
            # nearbyConeList structure: [[cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data], ]
            if(len(nearbyConeList) > 0):
                bestCandidateIndex = -1;   highestStrength = 0;   otherfrontOrBack = -1;  candidatesDiscarded = 0
                for i in range(len(nearbyConeList)):
                    cone = nearbyConeList[i] #not needed, just for legibility
                    #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data]
                    connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
                    if(connectionsFilled[0] and connectionsFilled[1]): #cone already doubly connected
                        print("cone already doubly connected, but that was supposed to be filtered out in distanceToCone()!?!")
                        candidatesDiscarded += 1
                    elif((cone[2][0][0] == coneToConnectID) or (cone[2][1][0] == coneToConnectID)): #cone already connected to coneToConnect
                        print("cone connection already exists, but that was supposed to be filtered out in distanceToCone()!?!")
                        candidatesDiscarded += 1
                    else:
                        frontOrBack = (coneToConnectPreferredConnection if (connectionsFilled[intBoolInv(coneToConnectPreferredConnection)]) else intBoolInv(coneToConnectPreferredConnection)) # default to the inverse of coneToConnectPreferredConnection
                        coneCandidateStrength = 1 #init var
                        coneCandidateStrength *= 1.5-(cone[3][0]/self.coneConnectionThreshold)  #high distance, low strength. Linear. worst>0.5 best<1.5
                        angleToCone = cone[3][1]
                        #hard no's: if the angle difference is above the max (like 135 degrees), the prospect cone is just too damn weird, just dont connect to this one
                        #note: this can be partially achieved by using angleThreshRange in distanceToCone() to preventatively discard cones like  angleThreshRange=([currentExistingAngle - self.coneConnectionMaxAngleDelta, currentExistingAngle + self.coneConnectionMaxAngleDelta] if (currentConnectionsFilled[0] or currentConnectionsFilled[1]) else [])
                        if(((connectionsFilled[0] or connectionsFilled[1]) and (abs(radDiff(angleToCone, cone[2][intBoolInv(frontOrBack)][2])) > self.coneConnectionMaxAngleDelta)) or ((currentConnectionsFilled[0] or currentConnectionsFilled[1]) and (abs(radDiff(angleToCone, currentExistingAngle)) > self.coneConnectionMaxAngleDelta))):
                            #print("very large angle delta")
                            candidatesDiscarded += 1
                        else:
                            if(connectionsFilled[0] or connectionsFilled[1]): #if cone already has a connection, check the angle delta
                                coneExistingAngle = cone[2][intBoolInv(frontOrBack)][2] #note: intBoolInv() is used to grab the existing connection
                                coneCandidateStrength *= 1.5- min(abs(radDiff(angleToCone, coneExistingAngle))/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            if(currentConnectionsFilled[0] or currentConnectionsFilled[1]):
                                coneCandidateStrength *= 1.5- min(abs(radDiff(angleToCone, currentExistingAngle))/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            #if(idk yet
                            #(Vincent's) most-restrictive angle could be implemented here, or at the end, by using SORTBY_ANGL_DELT and scrolling through the list from bestCandidateIndex to one of the ends of the list (based on left/right-edness), however, this does require a previous connection (preferably a connection that is in, or leads to, the pathList) to get angleDeltaTarget
                            if(coneCandidateStrength > highestStrength):
                                highestStrength = coneCandidateStrength
                                bestCandidateIndex = i
                                otherfrontOrBack = frontOrBack
                if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
                    print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
                    return(False, [])
                ## else (because it didnt return() and stop the function)
                #print("cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
                ## make the connection:
                coneListToUpdate = (self.rightConeList if leftOrRight else self.leftConeList)
                if(updateInputConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                    ## input cone
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][0] #save coneID
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][4] #save index
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][2] = nearbyConeList[bestCandidateIndex][3][1] #save angle (from perspective of coneToConnect)
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][4] = highestStrength
                    self.logFileChanged = True #set flag
                if(updateWinnerConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                    ## and the other cone
                    winnerConeIndexInList = nearbyConeList[bestCandidateIndex][4]
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][3][1]) #save angle (from perspective of that cone)
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3][0] #save distance
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
                    self.logFileChanged = True #set flag
                newConnectionData = [nearbyConeList[bestCandidateIndex][0], nearbyConeList[bestCandidateIndex][4], nearbyConeList[bestCandidateIndex][3][1], nearbyConeList[bestCandidateIndex][3][0], highestStrength, currentFrontOrBack, otherfrontOrBack]
                return(True, newConnectionData) # newConnectionData = [ID, index, angle, dist, strength, connection_index_inputCone, connection_index_winnerCone]
            else:
                #print("nearbyConeList empty")
                return(False, [])
    
    # def connectConeSuperSimple(self, coneToConnectID, coneToConnectPos, leftOrRight, coneToConnectIndex, currentConeConnections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], coneToConnectPreferredConnection=1, updateInputConeInList=True, updateWinnerConeInList=True):
    #     currentConnectionsFilled = [(currentConeConnections[0][1] >= 0), (currentConeConnections[1][1] >= 0)] #2-size list of booleans
    #     if(currentConnectionsFilled[0] and currentConnectionsFilled[1]):#if both connection entries are already full
    #         print("input cone already doubly connected?!:", currentConeConnections)
    #         return(False, [])
    #     else:
    #         currentFrontOrBack = (intBoolInv(coneToConnectPreferredConnection) if (currentConnectionsFilled[coneToConnectPreferredConnection]) else coneToConnectPreferredConnection) # default to coneToConnectPreferredConnection
    #         nearbyConeList = self.distanceToConeSquared(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], True, True, [coneToConnectID], self.coneConnectionThresholdSquared, True, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
    #         if(len(nearbyConeList) > 0):
    #             bestCandidateIndex = -1;   highestStrength = 0;   otherfrontOrBack = -1;  candidatesDiscarded = 0
    #             for i in range(len(nearbyConeList)):
    #                 cone = nearbyConeList[i] #not needed, just for legibility
    #                 #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], squared dist, index in left/right array, cone data (certainty, time spotted, etc)]
    #                 connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
    #                 if(connectionsFilled[0] and connectionsFilled[1]): #cone already doubly connected
    #                     print("cone already doubly connected, but that was supposed to be filtered out in distanceToCone()!?!")
    #                     candidatesDiscarded += 1
    #                 elif((cone[2][0][0] == coneToConnectID) or (cone[2][1][0] == coneToConnectID)): #cone already connected to coneToConnect
    #                     print("cone connection already exists, but that was supposed to be filtered out in distanceToCone()!?!")
    #                     candidatesDiscarded += 1
    #                 else:
    #                     frontOrBack = (coneToConnectPreferredConnection if (connectionsFilled[intBoolInv(coneToConnectPreferredConnection)]) else intBoolInv(coneToConnectPreferredConnection)) #default to the inverse of coneToConnectPreferredConnection
    #                     coneCandidateStrength = 1 #init var
    #                     coneCandidateStrength *= 1.5-(cone[3]/self.coneConnectionThresholdSquared)  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5
    #                     ## no angle math can be done, as Pythagoras's ABC is used, not sohcahtoa :)
    #                     if(coneCandidateStrength > highestStrength):
    #                         highestStrength = coneCandidateStrength
    #                         bestCandidateIndex = i
    #                         otherfrontOrBack = frontOrBack
    #             if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
    #                 print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
    #                 return(False, [])
    #             ## else (because it didnt return() and stop the function)
    #             print("cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
    #             ## make the connection:
    #             coneListToUpdate = (self.rightConeList if leftOrRight else self.leftConeList)
    #             if(updateInputConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
    #                 ## input cone
    #                 coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][0] #save coneID
    #                 coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][4] #save index
    #                 #cant save angle data
    #                 coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][3] #save squared distance
    #                 coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][4] = highestStrength
    #             if(updateWinnerConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
    #                 ## and the other cone
    #                 winnerConeIndexInList = nearbyConeList[bestCandidateIndex][4]
    #                 coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
    #                 coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
    #                 ## cant save angle data
    #                 coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][3] #save squared distance
    #                 coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
    #             newConnectionData = [nearbyConeList[bestCandidateIndex][0], nearbyConeList[bestCandidateIndex][4], nearbyConeList[bestCandidateIndex][3], highestStrength, currentFrontOrBack, otherfrontOrBack]
    #             return(True, newConnectionData) # newConnectionData = [ID, index, squared dist, strength, connection_index_inputCone, connection_index_winnerCone]
    #         else:
    #             print("nearbyConeList empty")
    #             return(False, [])
    
    def makePath(self):
        # pathList content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        # left/right-ConeList content: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]
        if(self.pathFullCircle):
            print("not gonna make path, already full circle")
            return(False)
        if(len(self.pathList) == 0):
            if((len(self.rightConeList) > 0) and (len(self.leftConeList) > 0)):
                #TBD !
                #delete this:
                firstLineLeftCone = self.leftConeList[0];   firstLineRightCone = self.rightConeList[0]
                
                ## if firstLine___Cone is connected to something, make sure those connections are angled similarly to the car itself, maybe
                
                pathWidth, lineAngle = distAngleBetwPos(firstLineLeftCone[1], firstLineRightCone[1])
                carAngle = radRoll(lineAngle + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
                centerPoint = [firstLineRightCone[1][0] + (firstLineLeftCone[1][0]-firstLineRightCone[1][0])/2, firstLineRightCone[1][1] + (firstLineLeftCone[1][1]-firstLineRightCone[1][1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
                self.pathList.append([centerPoint, [lineAngle, carAngle], pathWidth, [firstLineLeftCone[0], firstLineLeftCone[1], 0], [firstLineRightCone[0], firstLineRightCone[1], 0], 69.420])
            else:
                print("one or more of the coneLists is empty, cant place first pathLine")
                return(False)
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
                        print("single lastLeft connection already in pathList (path at end of cone line, make more connections)")
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
                        print("single lastRight connection already in pathList (path at end of cone line, make more connections)")
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
            #check if you've gone full circle
            if(((prospectLeftCone[0] == self.pathList[0][3][0]) and (prospectRightCone[0] == self.pathList[0][4][0])) \
               or ((lastLeftCone[0] == self.pathList[0][3][0]) and (prospectRightCone[0] == self.pathList[0][4][0])) \
               or ((prospectLeftCone[0] == self.pathList[0][3][0]) and (lastRightCone[0] == self.pathList[0][4][0]))):
                print("path full circle (by default)")
                self.pathFullCircle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
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
            print("path found:", maxStrengthIndex, "at strength:", round(maxStrengthVal, 2))
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
        isNewCone = True; returnConeID = 0; indexInLRlist=0; returnLeftOrRight=leftOrRight; returnPos=pos; returnConnections=[[subItem for subItem in item] for item in connections]; returnConeData=self.coneDataCopy(coneData) #init vars
        overlapsCone, overlappingConeLeftOrRight, overlappingConeIndex, overlappingConeData = self.overlapConeCheck(pos) #check if the new cone overlaps with an existing cone
        if(overlapsCone): #if the new cone overlaps an existing cone
            isNewCone = False
            returnConeID = overlappingConeData[0]
            indexInLRlist = overlappingConeIndex
            returnLeftOrRight = overlappingConeLeftOrRight
            returnPos = overlappingConeData[1] #copy pointer
            returnConnections = overlappingConeData[2] #copy pointer to connection data
            if(reconnectOverlappingCone): #really just for mouse-based UI, clicking on an existing cone will make it attempt to connect
                self.connectCone(returnConeID, returnPos, returnLeftOrRight, indexInLRlist, returnConnections, 1, True, True) #fill in (ID, pos, leftOrRight, index in left/right, currentConnections, preferred_connection_index)
                ## returnConnections will also change if , because it is only a pointer to the data in the left/right-conelist
            returnConeData = overlappingConeData[3]
        else:
            isNewCone = True
            returnConeID = self.newConeID
            if(connectNewCone):
                hasConnected, newConnectionData = self.connectCone(returnConeID, pos, leftOrRight, len(self.rightConeList) if leftOrRight else len(self.leftConeList), connections, 0, False, True) #little tricky: technically the left/right list index that the new cone is in doesnt exist yet, but it is about to be added to the end, so current list length = index. MAY NOT work if multithreaded
                if(hasConnected): ## newConnectionData structure: [ID, index, angle, dist, strength, connection_index_inputCone, connection_index_winnerCone]
                    #returnConnections[newConnectionData[5]] = [item for item in newConnectionData[0:5]]
                    returnConnections[newConnectionData[5]] = newConnectionData[0:5]
            ## now append the list
            listToAppend = (self.rightConeList if leftOrRight else self.leftConeList) #this saves a simgle line of code, totally worth it
            listToAppend.append([returnConeID, pos, returnConnections, returnConeData])
            indexInLRlist = len(listToAppend)-1
            if(self.logging):
                self.logCone(leftOrRight, returnConeID, pos, returnConnections, coneData)
            self.newConeID += 1
        return(isNewCone, returnConeID, indexInLRlist, returnLeftOrRight, returnPos, returnConnections, returnConeData) #return some useful data
    
    def addCar(self, pos=[12.0, 4.0], orient=0.0, color=[50,200,50]):
        self.cars.append(raceCar(pos, orient, color))
        return(len(self.cars)-1) #return the length of the cars list minus 1, because that is the list index of this new car
    
    def setFinishCone(self, leftOrRight, pos, coneDataInput=[CD_FINISH]): #note: left=False, right=True
        coneData = self.coneDataCopy(coneDataInput)
        # global CD_FINISH
        # coneData.append(CD_FINISH)
        #check if the requested position already has a cone
        addConeResult = self.addCone(leftOrRight, pos, coneData, connectNewCone=True, reconnectOverlappingCone=False) #attempt to add new cone, if a cone is already at that position, addCone() will return that info
        if((addConeResult[3] != self.finishLinePos[0][2]) if (len(self.finishLinePos) > 0) else True): #cant have two finish cones that are one-sided (left/right)
            if(not addConeResult[0]): #if this is True, a new cone was added
                print("setting finish on existing cone with ID:", addConeResult[1], ("(right)" if addConeResult[3] else "(left)"))
                for coneDataEntry in coneData: #for all coneData identifiers
                    if not (coneDataEntry in addConeResult[6]): #if it's not already in there
                        if(addConeResult[3]): #left/right
                            self.rightConeList[addConeResult[2]][3].append(coneDataEntry)
                        else:
                            self.leftConeList[addConeResult[2]][3].append(coneDataEntry)
                self.logFileChanged = True #set flag
            self.finishLinePos.append([addConeResult[1], addConeResult[4], addConeResult[3]]) # [cone ID, [x,y], left/right]
            return(True)
        else:
            print("existing finish cone is also", ("right," if addConeResult[3] else "left,"), "they can't both be")
            return(False)
        if(len(self.finishLinePos) > 2):
            print("too many finish line points")
            return(False)
    
    #drawing funtions
    def background(self):
        self.window.fill(self.bgColor, (self.drawPosX, self.drawPosY, self.drawWidth, self.drawHeight))
    
    def drawCones(self, drawLines=True):
        conePixelDiam = self.coneDiam * self.sizeScale
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        combinedConeList = (self.rightConeList + self.leftConeList)
        rightListLength = len(self.rightConeList)
        for i in range(len(combinedConeList)):
            conePos = combinedConeList[i][1]
            if(self.isInsideWindowReal(conePos)): #if it is within bounds, draw it
                conePos = self.realToPixelPos(conePos) #convert to pixel positions
                coneColor = self.rightConeColor if (i < rightListLength) else self.leftConeColor
                if(drawLines):
                    coneConnections = combinedConeList[i][2]
                    connectionsFilled = [(coneConnections[0][1] >= 0), (coneConnections[1][1] >= 0)] #2-size list of booleans
                    alreadyDrawn = [False, False]
                    for drawnLine in drawnLineList:
                        for j in range(2):
                            if(connectionsFilled[j]):
                                if(((combinedConeList[i][0] == drawnLine[0]) and (coneConnections[j][0] == drawnLine[1])) or ((combinedConeList[i][0] == drawnLine[1]) and (coneConnections[j][0] == drawnLine[0]))):
                                    alreadyDrawn[j] = True
                    for j in range(2):
                        if(connectionsFilled[j] and (not alreadyDrawn[j])): #if the 'back' conenction isnt already drawn
                            pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(self.rightConeList[coneConnections[j][1]][1] if (i < rightListLength) else self.leftConeList[coneConnections[j][1]][1]), self.coneLineWidth)
                            drawnLineList.append([combinedConeList[i][0], coneConnections[j][0]]) #put established 'back' connection in list of drawn lines
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
                overlapsCone, overlappingConeLeftOrRight, overlappingConeIndex, overlappingConeData = self.overlapConeCheck(self.pixelsToRealPos(self.floatingCone[0]))
                if(overlapsCone and drawPossibleConnections): #if mouse is hovering over existing cone
                    nearbyConeList = self.distanceToConeSquared(overlappingConeData[1], [self.rightConeList if overlappingConeLeftOrRight else self.leftConeList], False, True, [overlappingConeData[0]], self.coneConnectionThresholdSquared, True, [])
                    overlappingConePixelPos = self.realToPixelPos(overlappingConeData[1])
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
                        # someAngles = [-self.coneConnectionHighAngleDelta, self.coneConnectionHighAngleDelta]
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
global flagCurs24Data, flagCurs16Data, flagCursorSet
flagCurs24Data = ((24,24),(0,23)) + pygame.cursors.compile(flagCurs, 'X', '.', 'o')
flagCurs16Data = ((16,16),(0,15)) + pygame.cursors.compile(flagCurs16, 'X', '.', 'o')
flagCursorSet = False

global windowKeepRunning, windowStarted
windowStarted = False
windowKeepRunning = False

global pygamesimInputLast, oldWindowSize
pygamesimInputLast = None #to be filled
oldWindowSize = [1200,600]

def shouldKeepRunning():
    global windowKeepRunning
    return(windowKeepRunning)

def pygameInit():
    pygame.init()
    pygame.font.init()
    global window, oldWindowSize
    window = pygame.display.set_mode([1200, 600], pygame.RESIZABLE)
    oldWindowSize = window.get_size()
    pygame.display.set_caption("(pygame) selfdriving sim")
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
        global flagCursorSet
        if(keyDown):
            if(not flagCursorSet): #in pygame SDL2, holding a button makes it act like a keyboard button, and event gets spammed.
                pygame.event.set_grab(1)
                pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
                flagCursorSet = True
        else:
            pygame.event.set_grab(0)
            pygame.mouse.set_system_cursor(pygame.SYSTEM_CURSOR_ARROW)
            flagCursorSet = False
    elif(key==114): # r
        if(keyDown):
            pygamesimInput.makePath()
            # doesNothing = 0
            # while(pygamesimInput.makePath()): #stops when path can no longer be advanced
            #     doesNothing += 1  # "python is so versitile, you can do anything" :) haha good joke
    elif(key==108): # l
        if(keyDown):
            pygamesimInput.rewriteLogfile()

def currentPygamesimInput(pygamesimInputList, mousePos=None, demandMouseFocus=True): #if no pos is specified, retrieve it using get_pos()
    if(len(pygamesimInputList) > 1):
        if(mousePos is None):
            mousePos = pygame.mouse.get_pos()
        global pygamesimInputLast
        if(pygame.mouse.get_focused() or (not demandMouseFocus)):
            print("now"); inputCount = 0
            for pygamesimInput in pygamesimInputList:
                # localBoundries = [[pygamesimInput.drawPosX, pygamesimInput.drawPosY], [pygamesimInput.drawWidth, pygamesimInput.drawHeight]]
                # if(((mousePos[0]>=localBoundries[0][0]) and (mousePos[0]<(localBoundries[0][0]+localBoundries[1][0]))) and ((mousePos[1]>=localBoundries[0][1]) and (mousePos[0]<(localBoundries[0][1]+localBoundries[1][1])))):
                if(pygamesimInput.isInsideWindowPixels(mousePos)):
                    print("found input", inputCount)
                    pygamesimInputLast = pygamesimInput
                    return(pygamesimInput)
                inputCount += 1
        if(type(pygamesimInputLast) is not pygamesim): #if this is the first interaction
            pygamesimInputLast = pygamesimInputList[0]
        return(pygamesimInputLast)
    else:
        return(pygamesimInputList[0])

def handleWindowEvent(pygamesimInputList, eventToHandle):
    global window, oldWindowSize
    if(eventToHandle.type == pygame.QUIT):
        global windowKeepRunning
        windowKeepRunning = False #stop program (soon)
    
    elif(eventToHandle.type == pygame.VIDEORESIZE):
        newSize = eventToHandle.size
        if((oldWindowSize[0] != newSize[0]) or (oldWindowSize[1] != newSize[1])): #if new size is actually different
            print("video resize from", oldWindowSize, "to", newSize)
            correctedSize = [newSize[0], newSize[1]]
            window = pygame.display.set_mode(correctedSize, pygame.RESIZABLE)
            for pygamesimInput in pygamesimInputList:
                localOldSize = [pygamesimInput.drawWidth, pygamesimInput.drawHeight]
                localOldDrawPos = [pygamesimInput.drawPosX, pygamesimInput.drawPosY]
                localNewSize = [int((localOldSize[0]*correctedSize[0])/oldWindowSize[0]), int((localOldSize[1]*correctedSize[1])/oldWindowSize[1])]
                localNewDrawPos = [int((localOldDrawPos[0]*correctedSize[0])/oldWindowSize[0]), int((localOldDrawPos[1]*correctedSize[1])/oldWindowSize[1])]
                pygamesimInput.updateWindowSize(localNewSize[0], localNewSize[1], localNewDrawPos[0], localNewDrawPos[1], autoMatchSizeScale=False)
        oldWindowSize = window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.WINDOWEVENT):
        if(eventToHandle.event == 6): #in SDL, SDL_WINDOWEVENT_SIZE_CHANGED is 6
            newSize = window.get_size()
            if((oldWindowSize[0] != newSize[0]) or (oldWindowSize[1] != newSize[1])): #if new size is actually different
                print("video resize from", oldWindowSize, "to", newSize)
                correctedSize = [newSize[0], newSize[1]]
                for pygamesimInput in pygamesimInputList:
                    localOldSize = [pygamesimInput.drawWidth, pygamesimInput.drawHeight]
                    localOldDrawPos = [pygamesimInput.drawPosX, pygamesimInput.drawPosY]
                    localNewSize = [int((localOldSize[0]*correctedSize[0])/oldWindowSize[0]), int((localOldSize[1]*correctedSize[1])/oldWindowSize[1])]
                    localNewDrawPos = [int((localOldDrawPos[0]*correctedSize[0])/oldWindowSize[0]), int((localOldDrawPos[1]*correctedSize[1])/oldWindowSize[1])]
                    pygamesimInput.updateWindowSize(localNewSize[0], localNewSize[1], localNewDrawPos[0], localNewDrawPos[1], autoMatchSizeScale=False)
            oldWindowSize = window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.DROPFILE): #drag and drop files to import them
        if((pygame.mouse.get_pos()[0] == 0) and (pygame.mouse.get_pos()[1] == 0) and (len(pygamesimInputList) > 1)):
            print("skipping file import, please make sure to select the pygame window beforehand or something")
        else:
            currentPygamesimInput(pygamesimInputList, None, False).importConeLog(eventToHandle.file, True) #note: drag and drop functionality is a little iffy for multisim applications
    
    elif((eventToHandle.type == pygame.MOUSEBUTTONDOWN) or (eventToHandle.type == pygame.MOUSEBUTTONUP)):
        #print("mouse press", eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos)
        handleMousePress(currentPygamesimInput(pygamesimInputList, eventToHandle.pos, True), eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        
    elif((eventToHandle.type == pygame.KEYDOWN) or (eventToHandle.type == pygame.KEYUP)):
        #print("keypress:", eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key))
        handleKeyPress(currentPygamesimInput(pygamesimInputList, None, True), eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key), eventToHandle)

def handleAllWindowEvents(pygamesimInput): #input can be pygamesim object, 1D list of pygamesim objects or 2D list of pygamesim objects
    pygamesimInputList = []
    if(type(pygamesimInput) is pygamesim): #if it's actually a single input, not a list
        pygamesimInputList = [pygamesimInput] #convert to 1-sizes array
    elif(type(pygamesimInput) is list):
        if(len(pygamesimInput) > 0):
            for entry in pygamesimInput:
                if(type(entry) is list):
                    for subEntry in entry:
                        pygamesimInputList.append(subEntry) #2D lists
                else:
                    pygamesimInputList.append(entry) #1D lists
    #pygamesimInputList = pygamesimInput #assume input is list of pygamesims
    if(len(pygamesimInputList) < 1):
        print("len(pygamesimInputList) < 1")
        global windowKeepRunning
        windowKeepRunning = False
        pygame.event.pump()
        return()
    for eventToHandle in pygame.event.get(): #handle all events
        handleWindowEvent(pygamesimInputList, eventToHandle)
    


if __name__ == '__main__':
    pygameInit()
    
    sim1 = pygamesim(window) #just a basic class object with all default attributes
    ## auto import
    if(len(sys.argv) > 1):
        if(type(sys.argv[1]) is str):
            if(sys.argv[1].endswith('.csv')):
                print("found sys.argv[1] with a '.csv' extesion, attempting to import:", sys.argv[1])
                sim1.importConeLog(sys.argv[1])
            else:
                print("found sys.argv[1] but does not have '.csv' extension, so NOT importing that shit")
    # ## manual import
    # sim1 = pygamesim(window, importConeLogFilename='fixed problem.csv', logging=False)
    # ## alt
    # sim1.importConeLog('pygamesim_2020-11-04_15;38;48.csv')
    
    sim1.addCar() #add a default car
    
    while windowKeepRunning:
        handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        sim1.redraw()
        pygame.display.flip() #draw display
    
    print("closing logging file(s)...")
    sim1.closeLog()
    pygameEnd()