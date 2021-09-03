import numpy as np



def get_angle_between(obj_1, obj_2, obj_2_angle): #get angle between 2 positions, with respect to the angle of obj_2_angle in the global coordinate system
    """get angle between 2 positions (2-sized arrays/lists), shifted by another angle (like car.angle)"""
    return(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle)


# Array Scalar Multiplication and Addition (numpy probably also has these, but whatever)
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
degMiddOne = lambda lowBound, upBound : (degRoll(lowBound) + (degDiff(lowBound, upBound)/2) + (180 if (degDiff(lowBound, upBound) < 0) else 0))
degMidd = lambda lowBound, upBound : degRoll(degMiddOne(lowBound, upBound))

radDiff = lambda angleOne, angleTwo : radRoll(angleTwo-angleOne)  #recommend using abs(radDiff()), but currently, it returns the value required to get from angleOne to angleTwo, so (pi/2, -pi/4) = -3pi/4
radMiddOne = lambda lowBound, upBound : (radRoll(lowBound) + (radDiff(lowBound, upBound)/2) + (np.pi if (radDiff(lowBound, upBound) < 0) else 0))
radMidd = lambda lowBound, upBound : radRoll(radMiddOne(lowBound, upBound))

simpleRange = lambda angle, lowBound, upBound : ((lowBound <= angle) and (angle <= upBound))

degRangeOne = lambda angle, lowBound, upBound, offset : simpleRange(degRoll(angle-offset), degRoll(lowBound-offset), degRoll(upBound-offset)) #offset values to make lowBound a negative number between (-180, 0) and upBound between (0, 180) and offset input angle the same, to make the check simple
degRange = lambda angle, lowBound, upBound : degRangeOne(degRoll(angle), degRoll(lowBound), degRoll(upBound), degMiddOne(lowBound, upBound))
degInv = lambda angle : degRoll(angle + 180)

radRangeOne = lambda angle, lowBound, upBound, offset : simpleRange(radRoll(angle-offset), radRoll(lowBound-offset), radRoll(upBound-offset)) #offset values to make lowBound a negative number between (-pi, 0) and upBound between (0, pi) and offset input angle the same, to make the check simple
radRange = lambda angle, lowBound, upBound : radRangeOne(radRoll(angle), radRoll(lowBound), radRoll(upBound), radMiddOne(lowBound, upBound))
radInv = lambda angle : radRoll(angle + np.pi)


def get_norm_angle_between(obj_1, obj_2, obj_2_angle): #same as get_angle_between but rollover safe (you should probably still use radDiff when comparing angles, though)
    """get angle between 2 positions (2-sized arrays/lists), shifted by another angle (like car.angle), output is normalized to between -pi and pi"""
    return(radRoll(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle))

#note: distAngleBetwPos(obj_1, obj_2)[1]-obj_2_angle  =  get_angle_between(obj_1, obj_2, obj_2_angle)

def distAngleBetwPos(posOne, posTwo): #returns distance and angle between 2 positions
    """get distance and angle between 2 positions (2-sized arrays/lists)"""
    funcPosDelta = [posTwo[0]-posOne[0], posTwo[1]-posOne[1]]
    funcDistance = 0 #var init
    funcAngle = np.arctan2(funcPosDelta[1], funcPosDelta[0]) 
    if(abs(funcPosDelta[0]) < 0.0001): #sin(angle) can be 0, which results in divide by 0 errors.
        funcDistance = abs(funcPosDelta[1])
    elif(abs(funcPosDelta[1]) < 0.0001): #floating point error, alternatively you could check the angle
        funcDistance = abs(funcPosDelta[0])
    else:
        funcDistance = funcPosDelta[1]/np.sin(funcAngle)  #soh
        #funcDistance = funcPosDelta[0]/np.cos(funcAngle)  #cah
    return(np.array([funcDistance, funcAngle])) #return an np.array because the alternative is a tuple, which is (needlessly) immutable

def distSqrdBetwPos(posOne, posTwo): #returns distance^2 between 2 positions (useful for efficient distance thresholding)
    """get distance squared between 2 positions (2-sized arrays/lists), useful for efficient distance thresholding (compare to threshold squared)"""
    return((posTwo[0]-posOne[0])**2 + (posTwo[1]-posOne[1])**2)  #A^2 + B^2 = C^2

# def distPowBetwPos(posOne, posTwo): #returns distance between 2 positions (using ** operator specifically)
#     """get distance and angle between 2 positions (2-sized arrays/lists), uses square-root, inefficient!"""
#     return(distSqrdBetwPos(posOne, posTwo)**0.5) #just square root of the squared-distance (C instead of C^2 in A^2+B^2=C^2)

def distAnglePosToPos(funcRadius, funcAngle, funcPos): #returns a new pos given an angle, distance and starting pos
    """get position that is the entered distance and angle away from the entered position"""
    return(np.array([funcPos[0] + funcRadius * np.cos(funcAngle), funcPos[1] + funcRadius * np.sin(funcAngle)]))

def vectorProjectDist(posOne, posTwo, angleToProjectOnto): #returns the x and y distance after rotating by angleToProjectOnto from posOne
    """get x and y distance between two positions in a coordinate system that is rotated by the entered angle"""
    #approach (loosely) derived from vector math (not my (thijs) area of maximum expertise)
    posDeltas = [posTwo[0] - posOne[0], posTwo[1] - posOne[1]]
    cosAndSin = [np.cos(angleToProjectOnto), np.sin(angleToProjectOnto)]
    return(np.array([posDeltas[0]*cosAndSin[0] + posDeltas[1]*cosAndSin[1], posDeltas[1]*cosAndSin[0] - posDeltas[0]*cosAndSin[1]]))
    # # system shifting approach
    # hypotenuse, oldAngle = distAngleBetwPos(posOne, posTwo)
    # return(np.cos(oldAngle-angleToProjectOnto)*hypotenuse, np.sin(oldAngle-angleToProjectOnto)*hypotenuse)
    ##to be clear, this function is mostly for Car.distanceToCar(), where posOne is the Car.position and angleToProjectOnto is Car.angle
    ##this makes it sothat you can do easy comparisons 

#finding things in lists
def findIndexBy2DEntry(listToSearch, indexToCompare, valueToFind): #finds matching entry in 2D lists and returns index
    """find the index (of the outer list) that has an entry at the entered index (of the inner list) equal to the entered value (basically, finding entries in 2D lists)"""
    for i in range(len(listToSearch)):
        if(listToSearch[i][indexToCompare] == valueToFind):
            return(i)
    return(-1) #not found

def findIndexBy3DEntry(listToSearch, firstIndexToCompare, secondIndexToCompare, valueToFind): #finds matching entry in 3D lists and returns index
    """find the index (of the outer-most list) that has an entry at the entered indexes (of the 2nd and 3rd dimension lists) equal to the entered value (basically, finding entries in 3D lists)"""
    for i in range(len(listToSearch)):
        if(listToSearch[i][firstIndexToCompare][secondIndexToCompare] == valueToFind):
            return(i)
    return(-1) #not found

def findMinIndex(inputList): #finds smallest value in 1D list and returns index
    """find the index that holds the minimum value, as well as that value (builtin min() only returns value, not index)"""
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

def findMaxIndex(inputList): #finds biggest value in 1D list and returns index
    """find the index that holds the maximum value, as well as that value (builtin max() only returns value, not index)"""
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

#finding things in lists of classes
def findIndexByClassAttr(listToSearch, attrName: str, valueToFind): #finds matching attribute in lists of class objects and returns index
    """find the index (in a list of class objects) where the attribute with the entered name equals the entered value"""
    for i in range(len(listToSearch)):
        if(getattr(listToSearch[i], attrName) == valueToFind):
            return(i)
    return(-1) #not found


def findMinAttrIndex(inputList, attrName): #finds smallest attribute in 1D list of classes and returns index
    """find the index that holds the minimum (attribute) value, as well as that value"""
    if(len(inputList) > 0):
        returnIndex = 0
        minVal = getattr(inputList[0], attrName)
        for i in range(len(inputList)):
            val = getattr(inputList[i], attrName)
            if(val < minVal):
                minVal = val
                returnIndex = i
        return(returnIndex, minVal)
    else:
        return(-1, 0.0)

def findMaxAttrIndex(inputList, attrName): #finds biggest attribute in 1D list of classes and returns index
    """find the index that holds the maximum (attribute) value, as well as that value"""
    if(len(inputList) > 0):
        returnIndex = 0
        maxVal = getattr(inputList[0], attrName)
        for i in range(len(inputList)):
            val = getattr(inputList[i], attrName)
            if(val > maxVal):
                maxVal = val
                returnIndex = i
        return(returnIndex, maxVal)
    else:
        return(-1, 0.0)

def average(listOfValues):
    """get average value of list (empty list returns 0)"""
    if(len(listOfValues) > 0): #avoid divide by 0
        return(sum(listOfValues)/len(listOfValues))
    else:
        #print("averaging 0 values!?")
        return(0.0)

#i didn't realize python lists have a builtin 'sort()' which does exactly this
# def sortValues(listOfValues, upwards): #sort a list of values (int, float, etc.) from min to max or the other way around
#     """sort the entered list (does NOT return NEW list) by value, either upwards or downwards"""
#     for i in range(len(listOfValues)):
#         for j in range(i, len(listOfValues)):
#             if(((listOfValues[j] < listOfValues[i]) and upwards) or ((listOfValues[j] > listOfValues[i]) and (not upwards))):
#                 tempVal = listOfValues[i]
#                 listOfValues[i] = listOfValues[j]
#                 listOfValues[j] = tempVal
#     return(listOfValues)

#removed in favor of copy.deepcopy()
# def deepCopy(source): #can copy any list, variable or class (copy values, not pointers)
#     #print(type(source))
#     if(source.__class__.__module__ == 'builtins'): #python builtin types, like <int> and <list> and such
#         if((type(source) is list) or (type(source) is bytearray)):
#             returnList = source.__class__() #make new list/bytearray (same class as source)
#             for entry in source:
#                 returnList.append(deepCopy(entry))
#             return(returnList)
#         else:
#             return(source)
#     else: #custom classes (__module__ returns something like '__main__')
#         returnObject = source.__class__() #make new instance of same class as source
#         # alternatively, you might be able to retrieve an attribute name list with: getattr(getattr(getattr(source, '__init__'), '__code__'), 'co_names')
#         for attrName in dir(source): #dir(class) returs a list of all class attributes
#             if((not attrName.startswith('_')) and (not callable(getattr(source, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
#                 print(attrName)
#                 setattr(returnObject, attrName, deepCopy(getattr(source, attrName))) #copy attribute
#         return(returnObject)

