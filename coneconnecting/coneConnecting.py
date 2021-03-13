#TBD: add spoof coneConnecter class that gets data from network, to run visualization for non-local coneConnecter

#note: for numpy sin/cos/tan angles, 0 is at 3 o'clock, positive values are CCW and the range returned by arctan2 is (-180,180) or (-pi, pi)


import pygame       #python game library, used for the visualization
import numpy as np  #general math library
import datetime     #used for naming log files
import time         #used for (temporary) driving math in raceCar() class
import sys          #used for importing files from commandline (DOS run argument)

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
simpleRange = lambda angle, lowBound, upBound : ((lowBound <= angle) and (angle <= upBound))
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

def vectorProjectDist(posOne, posTwo, angleToProjectOnto):
    #approach (loosely) derived from vector math (not my area of maximum expertise)
    posDeltas = [posTwo[0] - posOne[0], posTwo[1] - posOne[1]]
    cosAndSin = [np.cos(angleToProjectOnto), np.sin(angleToProjectOnto)]
    return(posDeltas[0]*cosAndSin[0] + posDeltas[1]*cosAndSin[1], posDeltas[1]*cosAndSin[0] - posDeltas[0]*cosAndSin[1])
    # # system shifting approach
    # hypotenuse, oldAngle = distAngleBetwPos(posOne, posTwo)
    # return(np.cos(oldAngle-angleToProjectOnto)*hypotenuse, np.sin(oldAngle-angleToProjectOnto)*hypotenuse)

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

global CD_FINISH, coneLogTableColumnDef
CD_FINISH = 'finish'
coneLogTableColumnDef = "cone ID,leftOrRight,Xpos,Ypos,prev ID,next ID,coneData\n"

class raceCar:
    def __init__(self, pos=[12.0, 4.0], orient=0.0, steering=0.0, speed=0.0):
        self.pos = [pos[0], pos[1]] #position (real)
        self.orient = orient #orientation (radians)
        self.steering = steering #steering variable (angle, radius, whatever, IDK yet)
        self.speed = speed #forward velocity (wheel speed)
        self.lastUpdateTime = time.time() #a timestamp to get accurate deltaTime for updatePos()
        self.width = 1.5 #width of car (real, not pixels)
        self.length = 3 #height of car (real, not pixels)
    
    def updatePos(self): #update the position of the car
        timeRightNow = time.time() #python is not the fastest program, using time.time() at different points in this function will give different values, this won't
        deltaTime = timeRightNow - self.lastUpdateTime #get dt
        #turning math
        angularVelocity = 0 #init var
        if(abs(self.steering) > 0.001): #avoid divide by 0
            turningRadius = self.length / np.sin(self.steering)
            angularVelocity = self.speed / turningRadius
        #update variables
        self.orient += (deltaTime * angularVelocity) / 2 #half now...
        self.pos[0] += deltaTime * self.speed * np.cos(self.orient)
        self.pos[1] += deltaTime * self.speed * np.sin(self.orient)
        self.orient += (deltaTime * angularVelocity) / 2 #second half of angle change is added after linear movement is calulated, to (hopefully) increase accuracy slightly
        self.lastUpdateTime = timeRightNow
    
    def distanceToCar(self, pos): #a more comprehensive/accurate function for getting the SHORTEST distance to the surface of the car
        translatedDistance = vectorProjectDist(self.pos, pos, self.orient)
        #simSelf.debugLines.append([0, simSelf.realToPixelPos(self.pos), simSelf.realToPixelPos(pos), 0])
        if((abs(translatedDistance[0]) < (self.length/2)) and (abs(translatedDistance[1]) < (self.width/2))):
            return(True, 0)
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
                #returnDistance = ((abs(translatedDistance[0]) - (self.length/2))**2 + (abs(translatedDistance[1]) - (self.width/2))**2)**0.5  #A^2 + B^2 = C^2
                tempAngle = np.arctan2(abs(translatedDistance[1]) - (self.width/2), abs(translatedDistance[0]) - (self.length/2))
                returnDistance = abs(translatedDistance[1]/np.sin(tempAngle))  #soh  #note: this should never divide by 0, but if it does, change the '<' from previous two if-statements to a '<='
                #returnDistance = translatedDistance[0]/np.cos(tempAngle)  #cah
                ## debug:
                # debugOffsets = [[np.cos(self.pointAngle+self.orient) * self.pointRadius, np.sin(self.pointAngle+self.orient) * self.pointRadius],
                #         [np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius, np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius]]
                # debugPos = [(self.pos[i] + (-1 if (translatedDistance[1]<0) else 1)*debugOffsets[(0 if (((translatedDistance[0]<0) and (translatedDistance[1]<0)) or ((translatedDistance[0]>0) and (translatedDistance[1]>0))) else 1)][i])  for i in range(2)]
                # simSelf.debugLines.append([0, simSelf.realToPixelPos(pos), simSelf.realToPixelPos(debugPos), 2])
            return(False, returnDistance)


class coneConnecter:
    def __init__(self, car=None, importConeLogFilename='', logging=True, logname="coneLog"):
        self.car = car #list of cars in this simulation (normaly a list with only 1 entry)
        if(car is None):
            print("warning: no car in coneConnector, some functions will not work")
        self.logging = logging
        self.logfilename = ''
        
        if(logging):
            timeString = datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
            self.logfilename = (logname + "_" + timeString)
            self.logfile = open(self.logfilename + ".csv", "w")
            global coneLogTableColumnDef
            self.logfile.write(coneLogTableColumnDef)
            self.rewriteLogTimer = pygame.time.get_ticks() + 2000
        
        ## now init all other variables/lists
        self.coneDiam = 0.2 #meters
        
        self.coneConnectionThreshold = 5  #in meters (or at least not pixels)  note: hard threshold beyond which cones will NOT come into contention for connection
        self.coneConnectionThresholdSquared = self.coneConnectionThreshold**2
        self.coneConnectionHighAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.coneConnectionMaxAngleDelta = np.deg2rad(120) #if the angle difference is larger than this, it just doesnt make sense to connect them. (this IS a hard threshold)
        self.coneConnectionRestrictiveAngleChangeThreshold = np.deg2rad(20) # the most-restrictive-angle code only switches if angle is this much more restrictive
        self.coneConnectionRestrictiveAngleStrengthThreshold = 0.5 # the strength of the more restrictive cone needs to be at least this proportion of the old (less restrictive angle) strength
        
        self.pathConnectionThreshold = 10 #in meters (or at least not pixels)  IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        self.pathConnectionMaxAngleDelta = np.deg2rad(60) #IMPORTANT: not actual hard threshold, just distance at which lowest strength-score is given
        
        self.pathFirstLineCarAngleDeltaMax = np.deg2rad(45) #if the radDiff() between car (.orient) and the first line's connections is bigger than this, switch conneections or stop
        self.pathFirstLineCarSideAngleDelta = np.deg2rad(80) #left cones should be within +- pathFirstLineCarSideAngleDelta radians of the side of the car (asin, car.orient + or - pi/2, depending on left/right side)
        self.pathFirstLinePosDist = 4 # simple center to center distance, hard threshold, used to filter out very far cones
        self.pathFirstLineCarDist = 3 # real distance, not hard threshold, just distance at which lowest strength-score is given
        
        self.leftConeList = []  #[ [cone ID, [x,y], [[cone ID, index (left), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
        self.rightConeList = [] #[ [cone ID, [x,y], [[cone ID, index (right), angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)], ]     #note: 3rd argument is for drawing a line (track boundy) through the cones, every cone can connect to 2 (or less) cones in this way
        self.pathList = [] #[[center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength], ]
        # #the pathList is a (properly ordered) list of points/lines for the car to travel through. It (like most lists here) prioratizes math efficiency over RAM usage, so several stored parameters are redundant (can be recalculated given other elements), which should save time when calculating optimal path.
        self.newConeID = 0 #add 1 after adding a cone
        
        self.finishLinePos = [None, None] #[ left Cone, right Cone] #just pointers to cones
        
        #self.leftConesFullCircle = False  #TBD: no checking function yet, and because connectCone() can connect a cone to the 0th cone at any time without completing the circle, a special function this is required
        #self.rightConesFullCircle = False
        self.pathFullCircle = False
        
        self.rewriteLogTimer = 0
        self.logFileChanged = False
        
        #this has to happen AFTER variables/lists are initialized, (becuase it's hard to append to nonexistant conelists)
        if((len(importConeLogFilename) > 0) and (importConeLogFilename != '') and (importConeLogFilename != "")):
            self.importConeLog(importConeLogFilename, True)
    
    
    #logging functions
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
            coneDataStringList = coneDataString.strip().strip("[]").split(';')
            returnConeData = []
            for i in range(len(coneDataStringList)):
                #if((coneDataStringList[i].count['['] > 0) and (coneDataStringList[i].count['['] == coneDataStringList[i].count[']'])): #nesteld lists
                returnConeData.append(coneDataStringList[i].strip("'"))
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
            if(not (self.logging and (self.logfilename in filename))):
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
                    coneLists = [self.leftConeList, self.rightConeList]
                    for k in range(len(coneLists)): #for each list seperately
                        currentConeList = coneLists[k] #just for legibility
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
                            global CD_FINISH
                            if(CD_FINISH in currentConeList[i][3]):#if it has finish flag in coneData
                                self.finishLinePos[k] = currentConeList[i]
                    self.logFileChanged = True #logging shouldnt be on (in which case this will do nothing), but if it is, this is required
                else:
                    print("can't import coneLog because the first line is not equal to coneLogTableColumnDef")
                readFile.close()
            else:
                print("cant import own logfile")
        except FileNotFoundError:
            print("can't import coneLog because file:", "'" + filename + "'", "wasnt found.")
        except:
            print("exception in importConeLog()")
    
    #cone connecting functions    
    def distanceToConeSquared(self, pos, listsToCheck=[], sortByDistance=False, mergeLists=True, ignoreConeIDs=[], simpleSquaredThreshold=-1.0, coneConnectionExclusions=NO_CONN_EXCL, ignoreLinkedConeIDs=[]):
        if(len(listsToCheck) < 1):
            listsToCheck = [self.leftConeList, self.rightConeList] #cant be default parameter
        returnList = []  #[[ [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], squaredDist, index in left/right array, cone data (certainty, time spotted, etc)], ], ]  #note: coneID of first cone in first conelist is array[0][0][0]
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
                coneConnections = [cone[2][0][1] >= 0, cone[2][1][1] >= 0]
                if(((coneConnectionExclusions == EXCL_UNCONN) and not (coneConnections[0] or coneConnections[1])) or \
                   ((coneConnectionExclusions == EXCL_SING_CONN) and ((coneConnections[0] and not coneConnections[1]) or (not coneConnections[0] and coneConnections[1]))) or \
                   ((coneConnectionExclusions == EXCL_DUBL_CONN) and (coneConnections[0] and coneConnections[1])) or \
                   ((coneConnectionExclusions == EXCL_ANY_CONN) and (coneConnections[0] or coneConnections[1]))):
                    ignoreCone = True
                elif((cone[2][0][1] >= 0) or (cone[2][1][1] >= 0)):
                    for coneIDtoIgnore in ignoreLinkedConeIDs:  #using "((cone[2][0][0] in ignoreLinkedConeIDs) or (cone[2][0][0] in ignoreLinkedConeIDs))" would search the array twice, a single forloop is faster
                        if((cone[2][0][0] == coneIDtoIgnore) or (cone[2][1][0] == coneIDtoIgnore)):
                            ignoreCone = True
                if(not ignoreCone):
                    squaredDistance = (pos[0]-cone[1][0])**2 + (pos[1]-cone[1][1])**2  #A^2 + B^2 = C^2
                    if((simpleSquaredThreshold < 0) or ((simpleSquaredThreshold > 0) and (squaredDistance < simpleSquaredThreshold))):
                        ## returnCone structure: [squaredDistance, coneIndex, cone] (where 'cone' is just a simple pointer to the cone)
                        returnCone = [squaredDistance, coneIndex, cone]
                        if(sortByDistance):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((returnCone[0] < returnListPointer[i][0]) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        else:
                            returnListPointer.append(returnCone)
        return(returnList)
    
    def distanceToCone(self, pos, listsToCheck=[], sortBySomething=DONT_SORT, mergeLists=True, ignoreConeIDs=[], simpleThreshold=-1.0, coneConnectionExclusions=NO_CONN_EXCL, ignoreLinkedConeIDs=[], angleDeltaTarget=0.0, angleThreshRange=[]): #note: angleThreshRange is [lowBound, upBound] or [[lowBound, upBound], [lowBound, upBound], etc] for however many input lists there are (normally only 2, left and right)
        if(len(listsToCheck) < 1):
            listsToCheck = [self.leftConeList, self.rightConeList] #cant be default parameter    
        returnList = []  #[[ [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], [dist, angle], index in left/right array, cone data (certainty, time spotted, etc)], ], ]  #note: coneID of first cone in first conelist is array[0][0][0]
        multiAngleThreshRange = ((type(angleThreshRange[0]) is list) if (len(angleThreshRange)>0) else False)
        for conelist in range(len(listsToCheck)):
            returnListPointer = returnList
            if(not mergeLists):
                returnList.append([])
                returnListPointer = returnList[conelist] # makes appending to it easier
            hasAngleThreshRange = (len(angleThreshRange) == 2) #init var
            currentAngleThreshRange = angleThreshRange #init var
            if(multiAngleThreshRange):
                hasAngleThreshRange = ((len(angleThreshRange[conelist])==2) if (type(angleThreshRange[conelist]) is list) else False) #extra safety check
                if(hasAngleThreshRange):
                    currentAngleThreshRange = angleThreshRange[conelist]
                    # self.debugLines.append([1, self.realToPixelPos(self.car.pos), [simpleThreshold, currentAngleThreshRange[0]], 1]) #line from floating cone at someAngles[0] radians with a length of coneConnectionThreshold
                    # self.debugLines.append([1, self.realToPixelPos(self.car.pos), [simpleThreshold, currentAngleThreshRange[1]], 1]) #line from floating cone at someAngles[1] radians with a length of coneConnectionThreshold
                    # self.debugLines.append([2, self.realToPixelPos(self.car.pos), [simpleThreshold, currentAngleThreshRange[0], currentAngleThreshRange[1]], 1]) #arc centered at floating cone with radius coneConnectionThreshold, startAngle being someAngles[0] radians and endAngle being someAngles[1] radians
            #for cone in listsToCheck[conelist]: #doesnt let me store index
            for coneIndex in range(len(listsToCheck[conelist])):
                cone = listsToCheck[conelist][coneIndex]
                #cone stucture:  [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data (certainty, time spotted, etc)]
                ignoreCone = False
                if(cone[0] in ignoreConeIDs):
                    ignoreCone = True
                coneConnections = [cone[2][0][1] >= 0, cone[2][1][1] >= 0]
                if(((coneConnectionExclusions == EXCL_UNCONN) and not (coneConnections[0] or coneConnections[1])) or \
                   ((coneConnectionExclusions == EXCL_SING_CONN) and ((coneConnections[0] and not coneConnections[1]) or (not coneConnections[0] and coneConnections[1]))) or \
                   ((coneConnectionExclusions == EXCL_DUBL_CONN) and (coneConnections[0] and coneConnections[1])) or \
                   ((coneConnectionExclusions == EXCL_ANY_CONN) and (coneConnections[0] or coneConnections[1]))):
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
                    if(((distance < simpleThreshold) if (simpleThreshold > 0) else True) and ((radRange(angle, currentAngleThreshRange[0], currentAngleThreshRange[1])) if (hasAngleThreshRange) else True)):
                        ## returnCone structure: [[distance, angle], coneIndex, cone] (where 'cone' is just a simple pointer to the cone)
                        returnCone = [[distance, angle], coneIndex, cone]
                        
                        if(sortBySomething == SORTBY_DIST):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((returnCone[0][0] < returnListPointer[i][0][0]) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((returnCone[0][1] < returnListPointer[i][0][1]) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL_DELT):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((radDiff(returnCone[0][1], angleDeltaTarget) < radDiff(returnListPointer[i][0][1], angleDeltaTarget)) and (not insertionDone)): #if the new entry is larger
                                    returnListPointer.insert(i, returnCone)
                                    insertionDone = True
                            if(not insertionDone): #if the new entry is larger than the last entry, append it
                                returnListPointer.append(returnCone)
                        elif(sortBySomething == SORTBY_ANGL_DELT_ABS):
                            insertionDone = False
                            for i in range(len(returnListPointer)):
                                if((abs(radDiff(returnCone[0][1], angleDeltaTarget)) < abs(radDiff(returnListPointer[i][0][1], angleDeltaTarget))) and (not insertionDone)): #if the new entry is larger
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
            nearbyConeList = self.distanceToCone(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], SORTBY_DIST, True, [coneToConnectID], self.coneConnectionThreshold, EXCL_DUBL_CONN, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
            # nearbyConeList structure: [[dist, angle], index in left/right array, [[cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]]]
            if(len(nearbyConeList) < 1):
                #print("nearbyConeList empty")
                return(False, [])
            bestCandidateIndex = -1;   highestStrength = 0;   otherfrontOrBack = -1;  candidatesDiscarded = 0
            candidateList = []
            for i in range(len(nearbyConeList)):
                cone = nearbyConeList[i][2] #not strictly needed, just for legibility
                #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]  (regular cone structure)
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
                    coneCandidateStrength *= 1.5-(nearbyConeList[i][0][0]/self.coneConnectionThreshold)  #high distance, low strength. Linear. worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
                    angleToCone = nearbyConeList[i][0][1]
                    #hard no's: if the angle difference is above the max (like 135 degrees), the prospect cone is just too damn weird, just dont connect to this one
                    #note: this can be partially achieved by using angleThreshRange in distanceToCone() to preventatively discard cones like  angleThreshRange=([currentExistingAngle - self.coneConnectionMaxAngleDelta, currentExistingAngle + self.coneConnectionMaxAngleDelta] if (currentConnectionsFilled[0] or currentConnectionsFilled[1]) else [])
                    if(((connectionsFilled[0] or connectionsFilled[1]) and (abs(radDiff(angleToCone, cone[2][intBoolInv(frontOrBack)][2])) > self.coneConnectionMaxAngleDelta)) or ((currentConnectionsFilled[0] or currentConnectionsFilled[1]) and (abs(radDiff(angleToCone, currentExistingAngle)) > self.coneConnectionMaxAngleDelta))):
                        #print("very large angle delta")
                        candidatesDiscarded += 1
                    else:
                        if(connectionsFilled[0] or connectionsFilled[1]): #if cone already has a connection, check the angle delta
                            coneExistingAngle = cone[2][intBoolInv(frontOrBack)][2] #note: intBoolInv() is used to grab the existing connection
                            coneCandidateStrength *= 1.5- min(abs(radDiff(coneExistingAngle, angleToCone))/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                        if(currentConnectionsFilled[0] or currentConnectionsFilled[1]):
                            angleDif = radDiff(currentExistingAngle, angleToCone) #radians to add to currentExistingAngle to get to angleToCone (abs value is most interesting)
                            coneCandidateStrength *= 1.5- min(abs(angleDif)/self.coneConnectionHighAngleDelta, 1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            ## (Vincent's) most-restrictive angle could be implemented here, or at the end, by using SORTBY_ANGL_DELT and scrolling through the list from bestCandidateIndex to one of the ends of the list (based on left/right-edness), however, this does require a previous connection (preferably a connection that is in, or leads to, the pathList) to get angleDeltaTarget
                            coneCandidateStrength *= 1 + (0.5 if leftOrRight else -0.5)*max(min(angleDif/self.coneConnectionHighAngleDelta, 1), -1)  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            ## if most-restrictive angle is applied at the end, when all candidates have been reviewed
                            candidateList.append([i, angleDif, coneCandidateStrength])
                        #if(idk yet
                        if(coneCandidateStrength > highestStrength):
                            highestStrength = coneCandidateStrength
                            bestCandidateIndex = i
                            otherfrontOrBack = frontOrBack
            if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
                print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
                return(False, [])
            
            ## most restrictive angle check
            if(currentConnectionsFilled[0] or currentConnectionsFilled[1]): #most restrictive angle can only be applied if there is a reference angle (you could also check len(candidateList)
                bestCandidateListIndex = findIndexBy2DEntry(candidateList, 0, bestCandidateIndex)
                for i in range(len(candidateList)):
                    if((candidateList[i][1] > candidateList[bestCandidateListIndex][1]) if leftOrRight else (candidateList[i][1] < candidateList[bestCandidateListIndex][1])): #most restrictive angle is lowest angle
                        print("more restrictive angle found:", candidateList[i][1], "is better than", candidateList[bestCandidateListIndex][1])
                        #if((abs(radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > self.coneConnectionRestrictiveAngleChangeThreshold) and ((candidateList[i][2]/highestStrength) > self.coneConnectionRestrictiveAngleStrengthThreshold)):
                        if(abs(radDiff(candidateList[i][1], candidateList[bestCandidateListIndex][1])) > self.coneConnectionRestrictiveAngleChangeThreshold):
                            print("old highestStrength:", round(highestStrength, 2), " new highestStrength:", round(candidateList[i][2], 2), " ratio:", round(candidateList[i][2]/highestStrength, 2))
                            bestCandidateListIndex = i
                            bestCandidateIndex = candidateList[i][0]
                            highestStrength = candidateList[i][2]
            
            #print("cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
            ## make the connection:
            coneListToUpdate = (self.rightConeList if leftOrRight else self.leftConeList)
            if(updateInputConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                ## input cone
                coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][2][0] #save coneID
                coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][1] #save index
                coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][2] = nearbyConeList[bestCandidateIndex][0][1] #save angle (from perspective of coneToConnect)
                coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][0][0] #save distance
                coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][4] = highestStrength
                self.logFileChanged = True #set flag
            if(updateWinnerConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                ## and the other cone
                winnerConeIndexInList = nearbyConeList[bestCandidateIndex][1]
                coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
                coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
                coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][2] = radInv(nearbyConeList[bestCandidateIndex][0][1]) #save angle (from perspective of that cone)
                coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][0][0] #save distance
                coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
                self.logFileChanged = True #set flag
            newConnectionData = [nearbyConeList[bestCandidateIndex][2][0], nearbyConeList[bestCandidateIndex][1], nearbyConeList[bestCandidateIndex][0][1], nearbyConeList[bestCandidateIndex][0][0], highestStrength, currentFrontOrBack, otherfrontOrBack]
            return(True, newConnectionData) # newConnectionData = [ID, index, angle, dist, strength, connection_index_inputCone, connection_index_winnerCone]
    
    def connectConeSuperSimple(self, coneToConnectID, coneToConnectPos, leftOrRight, coneToConnectIndex, currentConeConnections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], coneToConnectPreferredConnection=1, updateInputConeInList=True, updateWinnerConeInList=True):
        currentConnectionsFilled = [(currentConeConnections[0][1] >= 0), (currentConeConnections[1][1] >= 0)] #2-size list of booleans
        if(currentConnectionsFilled[0] and currentConnectionsFilled[1]):#if both connection entries are already full
            print("input cone already doubly connected?!:", currentConeConnections)
            return(False, [])
        else:
            currentFrontOrBack = (intBoolInv(coneToConnectPreferredConnection) if (currentConnectionsFilled[coneToConnectPreferredConnection]) else coneToConnectPreferredConnection) # default to coneToConnectPreferredConnection
            nearbyConeList = self.distanceToConeSquared(coneToConnectPos, [self.rightConeList if leftOrRight else self.leftConeList], True, True, [coneToConnectID], self.coneConnectionThresholdSquared, EXCL_DUBL_CONN, [coneToConnectID])  #note: list is sorted by (squared) distance, but that's not really needed given the (CURRENT) math
            if(len(nearbyConeList) > 0):
                bestCandidateIndex = -1;   highestStrength = 0;   otherfrontOrBack = -1;  candidatesDiscarded = 0
                for i in range(len(nearbyConeList)):
                    cone = nearbyConeList[i][2] #not needed, just for legibility
                    #cone data structure: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], squared dist, index in left/right array, cone data (certainty, time spotted, etc)]
                    connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
                    if(connectionsFilled[0] and connectionsFilled[1]): #cone already doubly connected
                        print("cone already doubly connected, but that was supposed to be filtered out in distanceToCone()!?!")
                        candidatesDiscarded += 1
                    elif((cone[2][0][0] == coneToConnectID) or (cone[2][1][0] == coneToConnectID)): #cone already connected to coneToConnect
                        print("cone connection already exists, but that was supposed to be filtered out in distanceToCone()!?!")
                        candidatesDiscarded += 1
                    else:
                        frontOrBack = (coneToConnectPreferredConnection if (connectionsFilled[intBoolInv(coneToConnectPreferredConnection)]) else intBoolInv(coneToConnectPreferredConnection)) #default to the inverse of coneToConnectPreferredConnection
                        coneCandidateStrength = 1 #init var
                        coneCandidateStrength *= 1.5-(nearbyConeList[i][0]/self.coneConnectionThresholdSquared)  #high distance, low strength. non-Linear (quadratic?). worst>0.5 best<1.5  (note, no need to limit, because the min score is at the hard threshold)
                        ## no angle math can be done, as Pythagoras's ABC is used, not sohcahtoa :)
                        if(coneCandidateStrength > highestStrength):
                            highestStrength = coneCandidateStrength
                            bestCandidateIndex = i
                            otherfrontOrBack = frontOrBack
                if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(nearbyConeList) == candidatesDiscarded)):
                    print("it seems no suitible candidates for cone connection were found at all... bummer.", len(nearbyConeList), candidatesDiscarded, bestCandidateIndex, highestStrength)
                    return(False, [])
                ## else (because it didnt return() and stop the function)
                #print("simple cone connection made between (ID):", coneToConnectID, "and (ID):", nearbyConeList[bestCandidateIndex][0])
                ## make the connection:
                coneListToUpdate = (self.rightConeList if leftOrRight else self.leftConeList)
                if(updateInputConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                    ## input cone
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][0] = nearbyConeList[bestCandidateIndex][2][0] #save coneID
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][1] = nearbyConeList[bestCandidateIndex][1] #save index
                    #cant save angle data
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][3] = nearbyConeList[bestCandidateIndex][0] #save squared distance
                    coneListToUpdate[coneToConnectIndex][2][currentFrontOrBack][4] = highestStrength
                if(updateWinnerConeInList): #True in 99% of situations, but if you want to CHECK a connection without committing to it, then this should be False
                    ## and the other cone
                    winnerConeIndexInList = nearbyConeList[bestCandidateIndex][1]
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][0] = coneToConnectID #save coneID
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][1] = coneToConnectIndex #save index
                    ## cant save angle data
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][3] = nearbyConeList[bestCandidateIndex][0] #save squared distance
                    coneListToUpdate[winnerConeIndexInList][2][otherfrontOrBack][4] = highestStrength
                newConnectionData = [nearbyConeList[bestCandidateIndex][2][0], nearbyConeList[bestCandidateIndex][1], nearbyConeList[bestCandidateIndex][0], highestStrength, currentFrontOrBack, otherfrontOrBack]
                return(True, newConnectionData) # newConnectionData = [ID, index, squared dist, strength, connection_index_inputCone, connection_index_winnerCone]
            else:
                print("nearbyConeList empty")
                return(False, [])
    
    def makePath(self):
        # pathList content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        # left/right-ConeList content: [cone ID, [x,y], [[cone ID, index, angle, distance, cone-connection-strength], [(same as last entry)]], cone data]
        if(self.pathFullCircle):
            print("not gonna make path, already full circle")
            return(False)
        if(self.car is None):
            print("cant make path, there is no car")
            return(False)
        if(len(self.pathList) == 0):
            if((len(self.rightConeList) < 2) or (len(self.leftConeList) < 2)): #first pathLine can only be made between 2 connected cones
                print("not enough cones in one or more coneLists, cant place first pathLine")
                return(False)
            
            #search cones here
            firstCone = [None, None]
            firstConeIndexInArray = [-1,-1]
            if((self.finishLinePos[0] is not None) or (self.finishLinePos[1] is not None)):
                print("using finish cones to make first pathLine")
                if((self.finishLinePos[0] is not None) and (self.finishLinePos[1] is not None)):
                    firstCone = self.finishLinePos #finishLinePos is already a list of 2 pointers, so it should be fine to go without copying them(, right?)
                    firstConeIndexInArray = [findIndexBy2DEntry(self.leftConeList, 0, firstCone[0][0]), findIndexBy2DEntry(self.rightConeList, 0, firstCone[1][0])]
                else:
                    print("only 1 finish cone set, makePath can just wait untill the second is found, right?")
                    return(False)
            else: #if there aren't already finish cones, then find it the old-fashioned way, by looking for cones near the car
                sideAngleRanges = [[self.car.orient+(np.pi/2)-self.pathFirstLineCarSideAngleDelta, self.car.orient+(np.pi/2)+self.pathFirstLineCarSideAngleDelta], [self.car.orient-(np.pi/2)-self.pathFirstLineCarSideAngleDelta, self.car.orient-(np.pi/2)+self.pathFirstLineCarSideAngleDelta]] #left side is car.orient +pi/2, right side is car.orient -pi/2
                firstConeCandidates = self.distanceToCone(self.car.pos, [self.leftConeList, self.rightConeList], SORTBY_DIST, False, [], self.pathFirstLinePosDist, EXCL_UNCONN, [], self.car.orient, sideAngleRanges)
                for LorR in range(2):
                    bestCandidateIndex = -1;   highestStrength = 0;   candidatesDiscarded = 0
                    #carPerpAngle = self.car.orient + (np.pi*(0.5 if (LorR == 1) else -0.5))
                    carPerpAngle = self.car.orient - (np.pi/2) #always get CW perpendicular
                    for i in range(len(firstConeCandidates[LorR])):
                        cone = firstConeCandidates[LorR][i][2]
                        connectionsFilled = [(cone[2][0][1] >= 0), (cone[2][1][1] >= 0)] #2-size list of booleans
                        connectionAnglesAllowed = [((abs(radDiff(cone[2][0][2], self.car.orient)) < self.pathFirstLineCarAngleDeltaMax) if connectionsFilled[0] else False), ((abs(radDiff(cone[2][1][2], self.car.orient)) < self.pathFirstLineCarAngleDeltaMax) if connectionsFilled[1] else False)]
                        ## it's important to note that the distance calculated by distanceToCone() is between the center of the car and the cone, and therefore not the shortest path, or real distance to the car (a cone next to a wheel will have a higher distance than a cone next to the middle of the car, which is illogical)
                        ## this illogical distance can still be used to help filter out candidates, but for an accurate strength-rating, distanceToCar() (a function of the raceCar class) should be used
                        coneOverlapsCar, distToCar = self.car.distanceToCar(cone[1])
                        #print("evaluating "+("right" if (LorR==1) else "left")+" cone:", cone[0], connectionsFilled, connectionAnglesAllowed, coneOverlapsCar, distToCar)
                        if(not (connectionsFilled[0] or connectionsFilled[1])):
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
                            coneCandidateStrength *= 1.5-abs(radDiff(cone[2][(0 if connectionAnglesAllowed[0] else 1)][2], self.car.orient))/self.pathFirstLineCarAngleDeltaMax  #high angle delta, low strength. Linear. worst>0.5 best<1.5
                            ## this following check makes sure the pathline is perpendicular to the car
                            conePerpAngle = 0 #init var
                            if(connectionsFilled[0] and connectionsFilled[1]):
                                conePerpAngle = radMidd(cone[2][0][2], cone[2][1][2]) #note: radMidd() has inputs (lowBound, upBound), so for right cones this will give an angle that points AWAY from the car, and for left cones it points towards the car (both in the same direction if they're paralel)
                            else:
                                connectionToUse = (1 if connectionsFilled[1] else 0)
                                conePerpAngle = radRoll(cone[2][connectionToUse][2] + (np.pi*(0.5 if (connectionToUse == 0) else -0.5))) #see note on claculation with double connection
                            coneCandidateStrength *= 1.5-min(abs(radDiff(conePerpAngle, carPerpAngle))/self.pathConnectionMaxAngleDelta, 1)
                            ## using existing chosen firstCone, only works for the right cone (this is an unequal check, so i hate it, but it does work to make sure the first line is straight (not diagonal))
                            if(LorR == 1): #TO BE IMPROVED, but i dont know quite how yet
                                leftFirstConeConnectionsFilled = [(firstCone[0][2][0][1] >= 0), (firstCone[0][2][1][1] >= 0)]
                                leftPerpAngle = 0 #init var
                                if(leftFirstConeConnectionsFilled[0] and leftFirstConeConnectionsFilled[1]):
                                    leftPerpAngle = radMidd(firstCone[0][2][0][2], firstCone[0][2][1][2])
                                else:
                                    connectionToUse = (1 if leftFirstConeConnectionsFilled[1] else 0)
                                    leftPerpAngle = radRoll(firstCone[0][2][connectionToUse][2] + (np.pi*(0.5 if (connectionToUse == 0) else -0.5)))
                                tempPathWidth, tempPathAngle = distAngleBetwPos(firstCone[0][1], cone[1])
                                coneCandidateStrength *= 1.5-min(abs(radDiff(tempPathAngle, leftPerpAngle))/self.pathConnectionMaxAngleDelta, 1)
                                #you could also even do distance, but whatever
                            if(coneCandidateStrength > highestStrength):
                                highestStrength = coneCandidateStrength
                                bestCandidateIndex = i
                    if((bestCandidateIndex < 0) or (highestStrength <= 0) or (len(firstConeCandidates[LorR]) == candidatesDiscarded)):
                        print("it seems no suitible candidates for first "+("right" if (LorR==1) else "left")+" cone were found at all... bummer.", len(firstConeCandidates[LorR]), candidatesDiscarded, bestCandidateIndex, highestStrength)
                        return(False, [])
                    ## if the code makes it here, a suitable first cone has been selected.
                    #print("first "+("right" if (LorR==1) else "left")+" cone found!", highestStrength, bestCandidateIndex, len(firstConeCandidates[LorR]), candidatesDiscarded)
                    firstCone[LorR] = firstConeCandidates[LorR][bestCandidateIndex][2]
                    firstConeIndexInArray[LorR] = firstConeCandidates[LorR][bestCandidateIndex][1] #could be eliminated in favor of pointers?
            
            ## angle checks and swithing connections if possible
            for LorR in range(2):
                firstConeConnectionIndex = 1 #try to use the 'front' connection by default
                firstConnectionsFilled = [firstCone[LorR][2][0][1] >= 0, firstCone[LorR][2][1][1] >= 0]
                if(not (firstConnectionsFilled[0] or firstConnectionsFilled[1])): #if it has no conenctions at all (this SHOULD NEVER HAPPEN, because distanceToCone() filters out unconnected cones, but might as well check
                    #first cone is unconnected
                    print("no connections on "+("right" if (LorR==1) else "left")+" firstCone:", firstCone[LorR])
                    return(False)
                elif(not firstConnectionsFilled[firstConeConnectionIndex]): #if it only has a 'back' connection, just move the back one to the front
                    #first, switch connection data to make preferable (front) the only valid one
                    #print("whipping lastLeft (1):", lastLeftCone[2])
                    tempConVal = firstCone[LorR][2][0] #one of these is an empty connection
                    firstCone[LorR][2][0] = firstCone[LorR][2][1]
                    firstCone[LorR][2][1] = tempConVal
                    self.logFileChanged = True #set flag
                    #then check the angle of that connection. If it is too far off from the car angle then something is terribly wrong (or 
                    if(abs(radDiff(firstCone[LorR][2][firstConeConnectionIndex][2], self.car.orient)) > self.pathFirstLineCarAngleDeltaMax):
                        print("only first "+("right" if (LorR==1) else "left")+" connection angle larger than allowed:", firstConeConnectionIndex, round(np.rad2deg(firstCone[LorR][2][firstConeConnectionIndex][2]), 2), round(np.rad2deg(self.car.orient), 2), round(np.rad2deg(abs(radDiff(firstCone[LorR][2][firstConeConnectionIndex][2], self.car.orient))),2))
                        return(False)
                elif(firstConnectionsFilled[intBoolInv(firstConeConnectionIndex)]): #if it has both connections:
                    if(abs(radDiff(firstCone[LorR][2][firstConeConnectionIndex][2], self.car.orient)) > self.pathFirstLineCarAngleDeltaMax):
                        if(abs(radDiff(firstCone[LorR][2][intBoolInv(firstConeConnectionIndex)][2], self.car.orient)) > self.pathFirstLineCarAngleDeltaMax):
                            print("second left angle also larger than allowed", round(np.rad2deg(firstCone[LorR][2][intBoolInv(firstConeConnectionIndex)][2]), 2), round(np.rad2deg(self.car.orient), 2), round(np.rad2deg(abs(radDiff(firstCone[LorR][2][intBoolInv(firstConeConnectionIndex)][2], self.car.orient))),2))
                            return(False)
                        else: #first angle was large, but second angle wasnt, just switch the connections around and we're good to go
                            #print("whipping lastLeft (2):", lastLeftCone[2])
                            tempConVal = firstCone[LorR][2][0] #one of these is an empty connection
                            firstCone[LorR][2][0] = firstCone[LorR][2][1]
                            firstCone[LorR][2][1] = tempConVal
                            self.logFileChanged = True #set flag
                ## else do nothing, everything is allready good and there's no need to worry
            ## and now just put the first cones into the pathlist
            pathWidth, lineAngle = distAngleBetwPos(firstCone[0][1], firstCone[1][1])
            carAngle = radRoll(lineAngle + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
            centerPoint = [firstCone[1][1][0] + (firstCone[0][1][0]-firstCone[1][1][0])/2, firstCone[1][1][1] + (firstCone[0][1][1]-firstCone[1][1][1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
            self.pathList.append([centerPoint, [lineAngle, carAngle], pathWidth, [firstCone[0][0], firstCone[0][1], firstConeIndexInArray[0]], [firstCone[1][0], firstCone[1][1], firstConeIndexInArray[1]], 69.420])
        
        else: #if len(pathList) > 0
            lastPathLine = self.pathList[-1] # -1 gets the last item in list, you could also use (len(pathList)-1)
            coneLists = [self.leftConeList, self.rightConeList];  lastCone = [];  lastConeConnectionIndex = [];  lastConePerpAngle = [];  prospectCone = [];  prospectConeConnectionIndex = [];  
            for LorR in range(2):
                lastCone.append(coneLists[LorR][lastPathLine[3+LorR][2]])
                lastConeConnectionIndex.append(1) #try to use the 'front' connection by default
                lastConnectionsFilled = [lastCone[LorR][2][0][1] >= 0, lastCone[LorR][2][1][1] >= 0]
                #most of the code below is sanity checks, but some of it is for the purpouses of flipping connections to fit the 'back','front' model. This can be done differently, by putting it in the connectCone() function, for example. Threre might be some useless redundancy in the code below, but fuck it, it works (for now)
                if(not lastConnectionsFilled[lastConeConnectionIndex[LorR]]): #if it doesnt have a connected cone on the 
                    #print("no preferable (front) connection on lastLeftCone")
                    if(not lastConnectionsFilled[intBoolInv(lastConeConnectionIndex[LorR])]):
                        print("no connections on lastCone["+("right" if (LorR==1) else "left")+"] (impossible):", lastCone[LorR])
                        return(False)
                    else:
                        if(findIndexBy3DEntry(self.pathList, 3+LorR, 0, lastCone[LorR][2][intBoolInv(lastConeConnectionIndex[LorR])][0]) >= 0): #check if that isnt already in pathlist
                            ## if it is, then we just stop here. no more path generation can be done for now
                            print("single lastCone["+("right" if (LorR==1) else "left")+"] connection already in pathList (path at end of cone line, make more connections)")
                            return(False)
                        else: #if not, then the 'back' connection is the next (prospect) one, and this (last) cone has it all backwards. Switch the connection data for this cone around
                            #print("whipping lastCone["+("right" if (LorR==1) else "left")+"] (3):", lastLeftCone[2])
                            tempConVal = lastCone[LorR][2][0]
                            lastCone[LorR][2][0] = lastCone[LorR][2][1]
                            lastCone[LorR][2][1] = tempConVal
                            self.logFileChanged = True #set flag
                #super safety check for first-pathLine code
                if(len(self.pathList) == 1): #now check both angles again, just to be sure:
                    if(abs(radDiff(lastCone[LorR][2][lastConeConnectionIndex[LorR]][2], self.car.orient) > self.pathFirstLineCarAngleDeltaMax)):
                        print("post correction first "+("right" if (LorR==1) else "left")+" angle large:", lastConeConnectionIndex[LorR], round(np.rad2deg(lastCone[LorR][2][lastConeConnectionIndex[LorR]][2]), 2), round(np.rad2deg(self.car.orient), 2), round(np.rad2deg(abs(radDiff(lastCone[LorR][2][lastConeConnectionIndex[LorR]][2], self.car.orient))),2))
                        if(((lastConnectionsFilled[intBoolInv(lastConeConnectionIndex[LorR])])) and (abs(radDiff(lastCone[LorR][2][intBoolInv(lastConeConnectionIndex[LorR])][2], self.car.orient) > self.pathFirstLineCarAngleDeltaMax))):
                            print("post correction second angle also large", round(np.rad2deg(lastCone[LorR][2][intBoolInv(lastConeConnectionIndex[LorR])][2]), 2), round(np.rad2deg(self.car.orient), 2), round(np.rad2deg(abs(radDiff(lastCone[LorR][2][intBoolInv(lastConeConnectionIndex[LorR])][2], self.car.orient))),2))
                        return(False)
                lastConePerpAngle.append(radMidd(lastCone[LorR][2][LorR][2], lastCone[LorR][2][intBoolInv(LorR)][2]) if (lastCone[LorR][2][intBoolInv(lastConeConnectionIndex[LorR])][1] >= 0) else radRoll(lastCone[LorR][2][lastConeConnectionIndex[LorR]][2] + (np.pi*(0.5 if (lastConeConnectionIndex[LorR]==LorR) else -0.5)))) #note: addition or subtraction of half pi is a bit strange, dont worry about it :)
                #note: currently i am using radInv() when calculating angle delta, because angles are from left cone to right cone, but if you reverse radMidd(lastRightCone[2][1][2], lastRightCone[2][0][2]) to be radMidd(lastRightCone[2][0][2], lastRightCone[2][1][2]) it will give an inverted angle already. less human, more efficient
                #and now the prospect cones
                prospectCone.append(coneLists[LorR][lastCone[LorR][2][lastConeConnectionIndex[LorR]][1]])
            
            #check if you've gone full circle
            if(((prospectCone[0][0] == self.pathList[0][3][0]) and (prospectCone[1][0] == self.pathList[0][4][0])) \
               or ((lastCone[0][0] == self.pathList[0][3][0]) and (prospectCone[1][0] == self.pathList[0][4][0])) \
               or ((prospectCone[0][0] == self.pathList[0][3][0]) and (lastCone[1][0] == self.pathList[0][4][0]))):
                print("path full circle (by default)")
                self.pathFullCircle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            
            prospectConeConnectionIndex = [];  prospectConePerpAngle = []
            for LorR in range(2):
                prospectConeConnectionIndex.append(0 if (prospectCone[LorR][2][0][0] == lastCone[LorR][0]) else (1 if (prospectCone[LorR][2][1][0] == lastCone[LorR][0]) else -1)) #match connection. In simple, regular situations you could assume that the 'front' connection of the lastCone is the 'back' connection of prospectCone, but this is not a simple system, now is it :)
                if(prospectConeConnectionIndex[LorR] == -1):
                    print("BIG issue: lastCone["+("right" if (LorR==1) else "left")+"] pointed to this prospect cone, but this prospect cone does not point back", lastCone[LorR][2], prospectCone[LorR][2])
                elif(prospectConeConnectionIndex[LorR] == 1): #prospect cone has its connections switched around (lastLeftCone's 'front' should connect to prospectLeftCone's 'back')
                    #print("whipping prospect left:", prospectLeftCone[2])
                    tempConVal = prospectCone[LorR][2][0]
                    prospectCone[LorR][2][0] = prospectCone[LorR][2][1]
                    prospectCone[LorR][2][1] = tempConVal
                    prospectConeConnectionIndex[LorR] = 0
                    self.logFileChanged = True #set flag
                
                prospectConePerpAngle.append(radMidd(prospectCone[LorR][2][LorR][2], prospectCone[LorR][2][intBoolInv(LorR)][2]) if (prospectCone[LorR][2][intBoolInv(prospectConeConnectionIndex[LorR])][1] >= 0) else radRoll(prospectCone[LorR][2][prospectConeConnectionIndex[LorR]][2] + (np.pi*(0.5 if (prospectConeConnectionIndex[LorR] == LorR) else -0.5))))
                #note: currently i am using radInv() when calculating angle delta, because angles are from left cone to right cone, but if you reverse radMidd(lastRightCone[2][1][2], lastRightCone[2][0][2]) to be radMidd(lastRightCone[2][0][2], lastRightCone[2][1][2]) it will give an inverted angle already. less human, more efficient
            
            # self.debugLines = [] #clear debugLines
            # for LorR in range(2):
            #     self.debugLines.append([1, self.realToPixelPos(lastCone[LorR][1]), [4, lastConePerpAngle[LorR]], 1+LorR])
            #     self.debugLines.append([1, self.realToPixelPos(prospectCone[LorR][1]), [4, prospectConePerpAngle[LorR]], 1+LorR])
            
            ## all of could really be in a forloop of some kind, but fuck it; manual it is
            strengths = [1,1,1] #4 possible path lines, one of which already exists (between lastLeftCone and lastRightCone), so calculate the strengths for the remaining three possible pathlines
            pathWidths = [0,0,0];   pathAngles = [0,0,0]
            allCones = lastCone + prospectCone  #combine lists
            allPerpAngles = lastConePerpAngle + prospectConePerpAngle #combine lists
            maxStrengthIndex = -1; maxStrengthVal = -1;  winningCone = [None, None]
            for i in range(3):
                pathWidths[i], pathAngles[i] = distAngleBetwPos(allCones[(0 if (i==0) else 2)][1], allCones[(1 if (i==1) else 3)][1]) #last left to next right (for left (CCW) corners, where one left cone (lastLeftCone) connects to several right cones  (lastRightCone AND prospectRightCone))
                strengths[i] *= 1.5-min(pathWidths[i]/self.pathConnectionThreshold, 1) #strength based on distance, the threshold just determines minimum score, distance can be larger than threshold without math errors
                strengths[i] *= 1.5-min(abs(radDiff(pathAngles[i], allPerpAngles[(0 if (i==0) else 2)]))/self.pathConnectionMaxAngleDelta, 1) #strength based on angle delta from lastLeftConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
                strengths[i] *= 1.5-min(abs(radDiff(radInv(pathAngles[i]), allPerpAngles[(1 if (i==1) else 3)]))/self.pathConnectionMaxAngleDelta, 1) #strength based on angle delta from prospectRightConePerpAngle, the maxAngleDelta just determines minimum score, angle can be larger than threshold without math errors
                if(strengths[i] >= maxStrengthVal):
                    maxStrengthVal = strengths[i]
                    maxStrengthIndex = i
                    winningCone[0] = allCones[(0 if (i==0) else 2)];  winningCone[1] = allCones[(1 if (i==1) else 3)]
            
            print("path found:", maxStrengthIndex, "at strength:", round(maxStrengthVal, 2))
            carAngle = radRoll(pathAngles[maxStrengthIndex] + (np.pi/2)) # angle is from left cone to right, so 90deg (pi/2 rad) CCW rotation is where the car should go
            ## the next section could especially benefit from a forloop, as none of these values are in lists/arrays and they absolutely could be. At least it is slightly legible, i guess
            winningConeIndex = []
            if(maxStrengthIndex == 0):
                winningConeIndex.append(lastPathLine[3][2]);  winningConeIndex.append(lastCone[1][2][lastConeConnectionIndex[1]][1])
            elif(maxStrengthIndex == 1):
                winningConeIndex.append(lastCone[0][2][lastConeConnectionIndex[0]][1]);  winningConeIndex.append(lastPathLine[4][2])
            else: #(maxStrengthIndex == 2)
                winningConeIndex.append(lastCone[0][2][lastConeConnectionIndex[0]][1]);  winningConeIndex.append(lastCone[1][2][lastConeConnectionIndex[1]][1])
            #check if you've gone full circle
            if((winningCone[0][0] == self.pathList[0][3][0]) and (winningCone[1][0] == self.pathList[0][4][0])):
                print("path full circle (from winning cones)")
                self.pathFullCircle = True
                return(False) #technically, no new pathLine was added, but it does feel a little wrong to output the same value as errors at such a triumphant moment in the loop. 
            else:
                centerPoint = [winningCone[1][1][0] + (winningCone[0][1][0]-winningCone[1][1][0])/2, winningCone[1][1][1] + (winningCone[0][1][1]-winningCone[1][1][1])/2]  # [xpos + half Xdelta, yPos + half Ydelta]
                self.pathList.append([centerPoint, [pathAngles[maxStrengthIndex], carAngle], pathWidths[maxStrengthIndex], [winningCone[0][0], winningCone[0][1], winningConeIndex[0]], [winningCone[1][0], winningCone[1][1], winningConeIndex[1]], maxStrengthVal])
        return(True)
    
    def addCone(self, leftOrRight, pos, coneData=[], connections=[[-1,-1,0.0,0.0,0.0],[-1,-1,0.0,0.0,0.0]], connectNewCone=True, reconnectOverlappingCone=False): #note: left=False, right=True
        isNewCone = True; indexInLRlist=0; returnLeftOrRight=leftOrRight; #init vars
        overlapsCone, overlappingConeLeftOrRight, overlappingConeIndex, overlappingCone = self.overlapConeCheck(pos) #check if the new cone overlaps with an existing cone
        if(overlapsCone): #if the new cone overlaps an existing cone
            isNewCone = False
            indexInLRlist = overlappingConeIndex
            returnLeftOrRight = overlappingConeLeftOrRight
            if(reconnectOverlappingCone): #really just for mouse-based UI, clicking on an existing cone will make it attempt to connect
                self.connectCone(overlappingCone[0], overlappingCone[1], returnLeftOrRight, indexInLRlist, overlappingCone[2], 1, True, True) #fill in (ID, pos, leftOrRight, index in left/right, currentConnections, preferred_connection_index)
            returnConePointer = (self.rightConeList if returnLeftOrRight else self.leftConeList)[indexInLRlist]
        else:
            isNewCone = True
            newConnections = [[subItem for subItem in item] for item in connections]
            if(connectNewCone):
                hasConnected, newConnectionData = self.connectCone(self.newConeID, pos, leftOrRight, len(self.rightConeList) if leftOrRight else len(self.leftConeList), connections, 0, False, True) #little tricky: technically the left/right list index that the new cone is in doesnt exist yet, but it is about to be added to the end, so current list length = index. MAY NOT work if multithreaded
                if(hasConnected): ## newConnectionData structure: [ID, index, angle, dist, strength, connection_index_inputCone, connection_index_winnerCone]
                    #newConnections[newConnectionData[5]] = [item for item in newConnectionData[0:5]]
                    newConnections[newConnectionData[5]] = newConnectionData[0:5]
            ## now append the list
            listToAppend = (self.rightConeList if leftOrRight else self.leftConeList) #this saves a simgle line of code, totally worth it
            listToAppend.append([self.newConeID, pos, newConnections, self.coneDataCopy(coneData)])
            indexInLRlist = len(listToAppend)-1
            if(self.logging):
                self.logCone(leftOrRight, self.newConeID, pos, newConnections, coneData)
            self.newConeID += 1
            returnConePointer = listToAppend[indexInLRlist]
        return(isNewCone, indexInLRlist, returnLeftOrRight, returnConePointer) #return some useful data (note: returnConePointer contains all other data, but fuck it)
    
    def addCar(self, pos=[12.0, 4.0], orient=0.0, color=[50,200,50]): #note: the starting pos is just any number, should probably be 0
        if(self.car is None):
            self.car = raceCar([pos[0], pos[1]], orient, color=[color[0], color[1], color[2]]) #copy array, sothat cars dont have pointer to the same (default) array
            return(True)
        else: #if there's already a car on the track
            return(False)
    
    def setFinishCone(self, leftOrRight, pos, coneDataInput=[CD_FINISH]): #note: left=False, right=True
        coneData = self.coneDataCopy(coneDataInput)
        # global CD_FINISH
        # coneData.append(CD_FINISH)
        #check if the requested position already has a cone
        addConeResult = self.addCone(leftOrRight, pos, coneData, connectNewCone=False, reconnectOverlappingCone=False) #attempt to add new cone, if a cone is already at that position, addCone() will return that info
        returnedCone = addConeResult[3]
        if(self.finishLinePos[1 if (addConeResult[2]) else 0] is None): #cant have more than 1 finish cone per side
            if(not addConeResult[0]): #if this is True, a new cone was added
                print("setting finish on existing cone with ID:", returnedCone[0], ("(right)" if addConeResult[2] else "(left)"))
                for coneDataEntry in coneData: #for all coneData identifiers
                    if not (coneDataEntry in returnedCone[3]): #if it's not already in there
                        returnedCone[3].append(coneDataEntry)
                self.logFileChanged = True #set flag
            self.finishLinePos[1 if (addConeResult[2]) else 0] = returnedCone #i'm not sure it's kosher to use booleans as integers of 1 or 0 in python, so i made an extra little if-statement
            return(True)
        else:
            print(("right," if addConeResult[2] else "left,"), "finish cone already set")
            return(False)


#------------------------------------------------------------------------------------------------------------------------- everything from this point is for visualization ---------------------------------------------

#drawing funtions
class pygameDrawer:
    def __init__(self, window, drawSize=(1200,600), drawOffset=(0,0), viewOffset=(0,0), carCamOrient=0, sizeScale=30, startWithCarCam=False, invertYaxis=True):
        self.window = window #pass on the window object (pygame)
        self.drawSize = (int(drawSize[0]),int(drawSize[1])) #width and height of the display area (does not have to be 100% of the window)
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1])) #draw position offset, (0,0) is topleft
        self.viewOffset = [float(viewOffset[0]), float(viewOffset[1])] #'camera' view offsets, changing this affects the real part of realToPixelPos()
        self.carCamOrient = carCamOrient #orientation of the car (and therefore everything) on the screen. 0 is towards the right
        self.sizeScale = sizeScale #pixels per meter
        self.carCam = startWithCarCam #it's either carCam (car-centered cam, with rotating but no viewOffset), or regular cam (with viewOffset, but no rotating)
        self.invertYaxis = invertYaxis #pygame has pixel(0,0) in the topleft, so this just flips the y-axis when drawing things
        
        self.bgColor = [50,50,50] #grey
        
        self.finishLineColor = [255,40,0]
        self.finishLineWidth = 2 #pixels wide
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneLineWidth = 2 #pixels wide
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        #self.pathCenterPixelDiam = 
        
        self.floatingCone = [] #[ [[xpixel,ypixel], color] ] #note: x and y are in pixels, not meters. Translation to meters happens on final placement
        
        #the drawing stuff:
        self.carColor = [50,200,50]
        #polygon stuff (to be replaced by sprite?)
        self.carPointRadius = None #will be calculated once the car is drawn for the first time
        self.carPointAngle = None #will be calculated once the car is drawn for the first time
        
        self.movingViewOffset = False
        self.prevViewOffset = self.viewOffset
        self.movingViewOffsetMouseStart = [0,0]
        
        self.debugLines = [] #[[lineType, pos, pos/angles, color_index (0-2)], ] #third entry is: (lineType==0: straight line from two positions), (lineType==1: straight line from pos and [radius, angle]), (lineType==2: arc from pos and [radius, startAngle, endAngle])
        self.debugLineColors = [[255,0,255],[255,255,255],[0,255,0], [255,160,255]] #purple, white, green, pink
        self.debugLineWidth = 3
    
    #pixel conversion functions (the most important functions in here)
    def pixelsToRealPos(self, pixelPos):
        if(self.carCam):
            dist = 0; angle = 0; #init var
            if(self.invertYaxis):
                dist, angle = distAngleBetwPos([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2], [pixelPos[0], self.drawOffset[1]+(self.drawOffset[1]+self.drawSize[1])-pixelPos[1]]) #get distance to, and angle with respect to, center of the screen (car)
            else:
                dist, angle = distAngleBetwPos([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2], pixelPos) #get distance to, and angle with respect to, center of the screen (car)
            return(distAnglePosToPos(dist/self.sizeScale, radRoll(angle+self.car.orient-self.carCamOrient), self.car.pos)) #use converted dist, correctly offset angle & the real car pos to get a new real point
        else:
            if(self.invertYaxis):
                return([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((self.drawSize[1]-pixelPos[1]+self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]])
            else:
                return([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((pixelPos[1]-self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]])
    
    def realToPixelPos(self, realPos):
        if(self.carCam):
            dist, angle = distAngleBetwPos(self.car.pos, realPos) #get distance to, and angle with respect to, car
            shiftedPixelPos = distAnglePosToPos(dist*self.sizeScale, radRoll(angle-self.car.orient+self.carCamOrient), (self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2)) #calculate new (pixel) pos from the car pos, at the same distance, and the angle, plus the angle that the entire scene is shifted
            if(self.invertYaxis):
                return([shiftedPixelPos[0], self.drawOffset[1]+((self.drawOffset[1]+self.drawSize[1])-shiftedPixelPos[1])]) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return(shiftedPixelPos)
        else:
            if(self.invertYaxis):
                return([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], self.drawSize[1]-((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]]) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], ((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]])
    
    #check if things need to be drawn at all
    def isInsideWindowPixels(self, pixelPos):
        return((pixelPos[0] < (self.drawSize[0] + self.drawOffset[0])) and (pixelPos[0] > self.drawOffset[0]) and (pixelPos[1] < (self.drawSize[1] + self.drawOffset[1])) and (pixelPos[1] > self.drawOffset[1]))
    
    def isInsideWindowReal(self, realPos):
        return(self.isInsideWindowPixels(self.realToPixelPos(realPos))) #not very efficient, but simple
    
    #drawing functions
    def background(self):
        self.window.fill(self.bgColor, (self.drawOffset[0], self.drawOffset[1], self.drawSize[0], self.drawSize[1])) #dont fill entire screen, just this pygamesim's area (allowing for multiple sims in one window)
    
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
        if((self.finishLinePos[0] is not None) and (self.finishLinePos[1] is not None)):
            pygame.draw.line(self.window, self.finishLineColor, self.realToPixelPos(self.finishLinePos[0][1]), self.realToPixelPos(self.finishLinePos[1][1]), self.finishLineWidth)
    
    def drawCar(self):
        #drawing is currently done by calculating the position of the corners and drawing a polygon with those points. Not efficient, not pretty, just fun
        if(self.car is not None):
            if(self.isInsideWindowReal(self.car.pos)):
                if(self.carPointRadius is None):
                    self.carPointRadius = (((self.car.width**2)+(self.car.length**2))**0.5)/2 #Pythagoras
                    self.carPointAngle = np.arctan2(self.car.width, self.car.length) #this is used to make corner point for polygon
                polygonPoints = []
                # polygonPoints.append(simSelf.realToPixelPos([np.cos(self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
                # polygonPoints.append(simSelf.realToPixelPos([np.cos(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(np.pi-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
                # polygonPoints.append(simSelf.realToPixelPos([np.cos(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(np.pi+self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
                # polygonPoints.append(simSelf.realToPixelPos([np.cos(-self.pointAngle+self.orient) * self.pointRadius + self.pos[0], np.sin(-self.pointAngle+self.orient) * self.pointRadius + self.pos[1]]))
                offsets = [[np.cos(self.carPointAngle+self.car.orient) * self.carPointRadius, np.sin(self.carPointAngle+self.car.orient) * self.carPointRadius],
                            [np.cos(np.pi-self.carPointAngle+self.car.orient) * self.carPointRadius, np.sin(np.pi-self.carPointAngle+self.car.orient) * self.carPointRadius]]
                polygonPoints.append(self.realToPixelPos([self.car.pos[0] + offsets[0][0], self.car.pos[1] + offsets[0][1]])) #front left
                polygonPoints.append(self.realToPixelPos([self.car.pos[0] + offsets[1][0], self.car.pos[1] + offsets[1][1]])) #back left
                polygonPoints.append(self.realToPixelPos([self.car.pos[0] - offsets[0][0], self.car.pos[1] - offsets[0][1]])) #back right
                polygonPoints.append(self.realToPixelPos([self.car.pos[0] - offsets[1][0], self.car.pos[1] - offsets[1][1]])) #front right
                pygame.draw.polygon(self.window, self.carColor, polygonPoints) #draw car
                #arrow drawing (not needed, just handy to indicate direction of car)
                arrowPoints = [self.realToPixelPos(self.car.pos), polygonPoints[1], polygonPoints[2]] #not as efficient as using the line below, but self.pos can vary
                oppositeColor = [255-self.carColor[0], 255-self.carColor[1], 255-self.carColor[1]]
                pygame.draw.polygon(self.window, oppositeColor, arrowPoints) #draw arrow
    
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
                    nearbyConeList = self.distanceToConeSquared(overlappingConeData[1], [self.rightConeList if overlappingConeLeftOrRight else self.leftConeList], False, True, [overlappingConeData[0]], self.coneConnectionThresholdSquared, EXCL_DUBL_CONN, [])
                    overlappingConePixelPos = self.realToPixelPos(overlappingConeData[1])
                    for cone in nearbyConeList:
                        pygame.draw.line(self.window, coneColor, overlappingConePixelPos, self.realToPixelPos(cone[2][1]), int(self.coneLineWidth/2))
                else:
                    #pygame.draw.circle(self.window, coneColor, [int(self.floatingCone[0][0]), int(self.floatingCone[0][1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
                    conePos = ASA(-(conePixelDiam/2), self.floatingCone[0]) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                    pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
                    if(drawPossibleConnections):
                        nearbyConeList = self.distanceToConeSquared(self.pixelsToRealPos(self.floatingCone[0]), [self.rightConeList if self.floatingCone[1] else self.leftConeList], False, True, [], self.coneConnectionThresholdSquared, EXCL_DUBL_CONN, [])
                        # ## debug
                        # nearbyConeList = self.distanceToCone(self.pixelsToRealPos(self.floatingCone[0]), [self.rightConeList if self.floatingCone[1] else self.leftConeList], DONT_SORT, True, [], self.coneConnectionThreshold, EXCL_DUBL_CONN, [], 0.0, someAngles)
                        # someAngles = [-self.coneConnectionHighAngleDelta, self.coneConnectionHighAngleDelta]
                        # self.debugLines = []
                        # self.debugLines.append([1, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[0]], 0]) #line from floating cone at someAngles[0] radians with a length of coneConnectionThreshold
                        # self.debugLines.append([1, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[1]], 0]) #line from floating cone at someAngles[1] radians with a length of coneConnectionThreshold
                        # self.debugLines.append([2, self.floatingCone[0], [self.coneConnectionThreshold, someAngles[0], someAngles[1]], 0]) #arc centered at floating cone with radius coneConnectionThreshold, startAngle being someAngles[0] radians and endAngle being someAngles[1] radians
                        # ## end debug
                        for cone in nearbyConeList:
                            pygame.draw.line(self.window, coneColor, self.floatingCone[0], self.realToPixelPos(cone[2][1]), int(self.coneLineWidth/2))
    
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
    
    def updateViewOffset(self, mousePos=None):
        if(self.movingViewOffset):
            if(mousePos is None):
                mousePos = pygame.mouse.get_pos()
            mouseDelta = [] #init var
            if(self.invertYaxis):
                mouseDelta = [float(mousePos[0] - self.movingViewOffsetMouseStart[0]), float(self.movingViewOffsetMouseStart[1] - mousePos[1])]
            else:
                mouseDelta = [float(mousePos[0] - self.movingViewOffsetMouseStart[0]), float(mousePos[1] - self.movingViewOffsetMouseStart[1])]
            self.viewOffset[0] = self.prevViewOffset[0] + (mouseDelta[0]/self.sizeScale)
            self.viewOffset[1] = self.prevViewOffset[1] + (mouseDelta[1]/self.sizeScale)
    
    def redraw(self):
        self.updateViewOffset()
        self.background()
        self.drawCones(True) #boolean parameter is whether to draw lines between connected cones (track bounds) or not
        self.drawPathLines(True, True) #boolean parameters are whether to draw the lines between cones (not the line the car follows) and whether to draw circles (conesized ellipses) on the center points of path lines respectively
        self.drawFinishLine()
        self.drawCar()
        self.drawFloatingCone(True, False)
        self.drawDebugLines()
        if(self.logging):
            textimg = pygame.font.Font(None, 21).render(self.logfilename, 1, [255-self.bgColor[0], 255-self.bgColor[1], 255-self.bgColor[2]], self.bgColor)
            self.window.blit(textimg, [self.drawOffset[0] + 5,self.drawOffset[1] + 5])
        if(self.logging and (pygame.time.get_ticks() > self.rewriteLogTimer) and self.logFileChanged):
            self.rewriteLogfile()
        #this section SHOULDN'T be in redraw(), but instead in some form of general update(), but as there currently isn't one yet, and it's only needed for one thing (manual driving), i put it here
        if(self.car is not None):
            self.car.updatePos()
    
    def updateWindowSize(self, drawSize = [1200, 600], drawOffset = [0,0], sizeScale=-1, autoMatchSizeScale=True):
        if(sizeScale > 0):
            self.sizeScale = sizeScale
        elif(autoMatchSizeScale):
            self.sizeScale = min(drawSize[0]/self.drawSize[0], drawSize[1]/self.drawSize[1]) * self.sizeScale #auto update sizeScale to match previous size
        self.drawSize = (int(drawSize[0]), int(drawSize[1]))
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1]))



class pygamesimLocal(coneConnecter, pygameDrawer):
    def __init__(self, window, car=None, drawSize=(1200,600), drawOffset=(0,0), viewOffset=[0,0], carCamOrient=0, sizeScale=30, startWithCarCam=False, invertYaxis=True, importConeLogFilename='', logging=True, logname="coneLog"):
        coneConnecter.__init__(self, car, importConeLogFilename, logging, logname)
        pygameDrawer.__init__(self, window, drawSize, drawOffset, viewOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)

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
oldWindowSize = []

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
                pygamesimInput.setFinishCone(False, pygamesimInput.pixelsToRealPos(pos))
                pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
            else:
                pygamesimInput.addCone(False, pygamesimInput.pixelsToRealPos(pos), connectNewCone=(pygame.key.get_pressed()[pygame.K_LSHIFT]), reconnectOverlappingCone=True) #place a new cone, and connect if shift is held
    if(button==3): #right mouse button
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
                pygamesimInput.addCone(True, pygamesimInput.pixelsToRealPos(pos), connectNewCone=(pygame.key.get_pressed()[pygame.K_LSHIFT]), reconnectOverlappingCone=True) #place a new cone, and connect if shift is held
    elif(button==2): #middle mouse button
        if(buttonDown): #mouse pressed down
            if(not pygamesimInput.carCam):
                pygame.event.set_grab(1)
                pygame.mouse.set_system_cursor(pygame.SYSTEM_CURSOR_HAND)
                pygamesimInput.movingViewOffset = True
                pygamesimInput.movingViewOffsetMouseStart = pygame.mouse.get_pos()
                pygamesimInput.prevViewOffset = (pygamesimInput.viewOffset[0], pygamesimInput.viewOffset[1])
        else:           #mouse released
            pygame.event.set_grab(0)
            pygame.mouse.set_system_cursor(pygame.SYSTEM_CURSOR_ARROW)
            pygamesimInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
            pygamesimInput.movingViewOffset = False

def handleKeyPress(pygamesimInput, keyDown, key, eventToHandle):
    if(key==pygame.K_f): # f
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
    elif(key==pygame.K_r): # r
        if(keyDown):
            pygamesimInput.makePath()
            # doesNothing = 0
            # while(pygamesimInput.makePath()): #stops when path can no longer be advanced
            #     doesNothing += 1  # "python is so versitile, you can do anything" :) haha good joke
    elif(key==pygame.K_l): # l
        if(keyDown):
            pygamesimInput.rewriteLogfile()
    elif(key==pygame.K_c): # c
        if(keyDown):
            if(pygamesimInput.car is not None):
                pygamesimInput.carCam = not pygamesimInput.carCam
                if(pygamesimInput.carCam and pygamesimInput.movingViewOffset): #if you switched to carCam while you were moving viewOffset, just stop moving viewOffset (same as letting go of MMB)
                    pygame.event.set_grab(0)
                    pygame.mouse.set_system_cursor(pygame.SYSTEM_CURSOR_ARROW)
                    pygamesimInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
                    pygamesimInput.movingViewOffset = False
            else:
                print("can't switch to car-centered cam, there is no car")

def currentPygamesimInput(pygamesimInputList, mousePos=None, demandMouseFocus=True): #if no pos is specified, retrieve it using get_pos()
    if(len(pygamesimInputList) > 1):
        if(mousePos is None):
            mousePos = pygame.mouse.get_pos()
        global pygamesimInputLast
        if(pygame.mouse.get_focused() or (not demandMouseFocus)):
            for pygamesimInput in pygamesimInputList:
                # localBoundries = [[pygamesimInput.drawOffset[0], pygamesimInput.drawOffset[1]], [pygamesimInput.drawSize[0], pygamesimInput.drawSize[1]]]
                # if(((mousePos[0]>=localBoundries[0][0]) and (mousePos[0]<(localBoundries[0][0]+localBoundries[1][0]))) and ((mousePos[1]>=localBoundries[0][1]) and (mousePos[0]<(localBoundries[0][1]+localBoundries[1][1])))):
                if(pygamesimInput.isInsideWindowPixels(mousePos)):
                    pygamesimInputLast = pygamesimInput
                    return(pygamesimInput)
        if(pygamesimInputLast is None): #if this is the first interaction
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
                localOldSize = [pygamesimInput.drawSize[0], pygamesimInput.drawSize[1]]
                localOldDrawPos = [pygamesimInput.drawOffset[0], pygamesimInput.drawOffset[1]]
                localNewSize = [int((localOldSize[0]*correctedSize[0])/oldWindowSize[0]), int((localOldSize[1]*correctedSize[1])/oldWindowSize[1])]
                localNewDrawPos = [int((localOldDrawPos[0]*correctedSize[0])/oldWindowSize[0]), int((localOldDrawPos[1]*correctedSize[1])/oldWindowSize[1])]
                pygamesimInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
        oldWindowSize = window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.WINDOWEVENT):
        if(eventToHandle.event == 6): #in SDL, SDL_WINDOWEVENT_SIZE_CHANGED is 6
            newSize = window.get_size()
            if((oldWindowSize[0] != newSize[0]) or (oldWindowSize[1] != newSize[1])): #if new size is actually different
                print("video resize from", oldWindowSize, "to", newSize)
                correctedSize = [newSize[0], newSize[1]]
                for pygamesimInput in pygamesimInputList:
                    localOldSize = [pygamesimInput.drawSize[0], pygamesimInput.drawSize[1]]
                    localOldDrawPos = [pygamesimInput.drawOffset[0], pygamesimInput.drawOffset[1]]
                    localNewSize = [int((localOldSize[0]*correctedSize[0])/oldWindowSize[0]), int((localOldSize[1]*correctedSize[1])/oldWindowSize[1])]
                    localNewDrawPos = [int((localOldDrawPos[0]*correctedSize[0])/oldWindowSize[0]), int((localOldDrawPos[1]*correctedSize[1])/oldWindowSize[1])]
                    pygamesimInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
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
        handleKeyPress(currentPygamesimInput(pygamesimInputList, None, True), eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
    
    elif(eventToHandle.type == pygame.MOUSEWHEEL): #scroll wheel (zooming / rotating)
        simToScale = currentPygamesimInput(pygamesimInputList, None, True)
        if(pygame.key.get_pressed()[pygame.K_LCTRL] and simToScale.carCam): #if holding (left) CTRL while in carCam mode, rotate the view
            simToScale.carCamOrient += (eventToHandle.y * np.pi/16)
        else:
            simToScale.sizeScale += eventToHandle.y #zooming

def handleAllWindowEvents(pygamesimInput): #input can be pygamesim object, 1D list of pygamesim objects or 2D list of pygamesim objects
    pygamesimInputList = []
    if(type(pygamesimInput) is (pygamesimLocal or pygamesimLocal)): #if it's actually a single input, not a list
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
    #the manual keyboard driving (tacked on here, because doing it with the event system would require more variables, and this is temporary anyway)
    carToDrive = currentPygamesimInput(pygamesimInputList, demandMouseFocus=False).car #get the active sim within the window
    if(carToDrive is not None):
        pressedKeyList = pygame.key.get_pressed()
        speedAccelVal = 0.025
        steerAccelVal = 0.005
        #first for speed
        if(pressedKeyList[pygame.K_UP]): #accelerate button
            carToDrive.speed += speedAccelVal #accelerate
        elif(pressedKeyList[pygame.K_DOWN]): #brake/reverse button
            if(carToDrive.speed > (speedAccelVal*3)): #positive speed
                carToDrive.speed -= speedAccelVal * 3 #fast brake
            else:                               #near-zero or negative speed
                carToDrive.speed -= speedAccelVal * 0.5 #reverse accelerate
        else:                           #neither buttons
            if(carToDrive.speed > speedAccelVal): #positive speed
                carToDrive.speed -= speedAccelVal/2 #slow brake
            elif(carToDrive.speed < -speedAccelVal): #negative speed
                carToDrive.speed += speedAccelVal #brake
            else:                           #near-zero speed
                carToDrive.speed = 0
        carToDrive.speed = max(-10, min(10, carToDrive.speed)) #limit speed
        #now for steering
        if(pressedKeyList[pygame.K_LEFT] and (not pressedKeyList[pygame.K_RIGHT])):
            carToDrive.steering += steerAccelVal
        elif(pressedKeyList[pygame.K_RIGHT] and (not pressedKeyList[pygame.K_LEFT])):
            carToDrive.steering -= steerAccelVal
        else:
            if(carToDrive.steering > steerAccelVal):
                carToDrive.steering -= steerAccelVal*2.5
            elif(carToDrive.steering < -steerAccelVal):
                carToDrive.steering += steerAccelVal*2.5
            else:
                carToDrive.steering = 0
        carToDrive.steering = max(-np.pi/5, min(np.pi/5, carToDrive.steering)) #limit speed





if __name__ == '__main__':
    pygameInit()
    
    sim1 = pygamesimLocal(window, raceCar()) #just a basic class object with all default attributes
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
    
    
    while windowKeepRunning:
        handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        sim1.redraw()
        pygame.display.flip() #draw display
    
    print("closing logging file(s)...")
    sim1.closeLog()
    pygameEnd()