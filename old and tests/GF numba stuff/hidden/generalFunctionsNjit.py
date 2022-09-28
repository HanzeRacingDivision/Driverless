from numba import njit#, prange #njit is for compilation (same as @jit(nopython=True)), prange is for parallel forloops
#from numba import float32, float64
import numpy as np


@njit
def get_angle_between(obj_1, obj_2, obj_2_angle): #get angle between 2 positions, with respect to the angle of obj_2_angle in the global coordinate system
    """get angle between 2 positions (2-sized arrays/lists), shifted by another angle (like car.angle)
        numba compiled!"""
    return(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle)


## angle rollover functions (for 2d positioning systems, given that arctan2 outputs between (-pi, pi))
@njit #([float32(float32), float64(float64)]) #numba allows (encourages?) you to specify the input/output types. that could make compilation faster(?)
def degRoll(angle):
    """return the angle (degrees) considering (-180, 180) rollover
        numba compiled!"""
    tempAngle = ((angle % -360.0) if (angle < 0) else (angle % 360.0))
    return((tempAngle % -180.0) if (tempAngle > 180.0) else ((tempAngle % 180.0) if (tempAngle < -180.0) else tempAngle))

@njit
def radRoll(angle):
    """return the angle (radians) considering (-pi, pi) rollover
        numba compiled!"""
    tempAngle = ((angle % (-2*np.pi)) if (angle < 0) else (angle % (2*np.pi)))
    return((tempAngle % -np.pi) if (tempAngle > np.pi) else ((tempAngle % np.pi) if (tempAngle < -np.pi) else tempAngle))

## angle math functions (that incorporate rollover)
@njit
def degDiff(angleOne, angleTwo):
    """get (rollover safe) difference between 2 angles in degrees.
        recommend using abs(radDiff()), but currently, 
         it returns the value required to get from angleOne to angleTwo, 
         so (90, -45) = -135
        numba compiled!"""
    return(degRoll(angleTwo-angleOne))

@njit
def degMidd(lowBound, upBound):
    """get the angle in the middle of (the shortest angle between) two angles (degrees)
        numba compiled!"""
    return(degRoll(degRoll(lowBound) + (degDiff(lowBound, upBound)/2) + (180.0 if (degDiff(lowBound, upBound) < 0) else 0)))

@njit
def radDiff(angleOne, angleTwo):
    """get (rollover safe) difference between 2 angles in radians.
        recommend using abs(radDiff()), but currently, 
         it returns the value required to get from angleOne to angleTwo, 
         so (pi/2, -pi/4) = -3pi/4
        numba compiled!"""
    return(radRoll(angleTwo-angleOne))

@njit
def radMidd(lowBound, upBound):
    """get the angle in the middle of (the shortest angle between) two angles (radians)
        numba compiled!"""
    return(radRoll(radRoll(lowBound) + (radDiff(lowBound, upBound)/2) + (np.pi if (radDiff(lowBound, upBound) < 0) else 0)))
    
@njit
def simpleRange(angle, lowBound, upBound):
    return((lowBound <= angle) and (angle <= upBound))

@njit
def degRange(angle, lowBound, upBound):
    offset = (degRoll(lowBound) + (degDiff(lowBound, upBound)/2) + (180.0 if (degDiff(lowBound, upBound) < 0) else 0))
    return(simpleRange(degRoll(angle-offset), degRoll(lowBound-offset), degRoll(upBound-offset)))

@njit
def degInv(angle):
    return(radRoll(angle + 180.0))

@njit
def radRange(angle, lowBound, upBound):
    offset = (radRoll(lowBound) + (radDiff(lowBound, upBound)/2) + (np.pi if (radDiff(lowBound, upBound) < 0) else 0))
    return(simpleRange(radRoll(angle-offset), radRoll(lowBound-offset), radRoll(upBound-offset)))

@njit
def radInv(angle):
    return(radRoll(angle + np.pi))

@njit
def get_norm_angle_between(obj_1, obj_2, obj_2_angle): #same as get_angle_between but rollover safe (you should probably still use radDiff when comparing angles, though)
    """get angle between 2 positions (2-sized arrays/lists), shifted by another angle (like car.angle), output is normalized to between -pi and pi
        numba compiled!"""
    return(radRoll(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle))

#note: distAngleBetwPos(obj_1, obj_2)[1]-obj_2_angle  =  get_angle_between(obj_1, obj_2, obj_2_angle)

@njit
def distAngleBetwPos(posOne: np.ndarray, posTwo: np.ndarray): #returns distance and angle between 2 positions
    """get distance and angle between 2 positions (2-sized arrays/lists)
        numba compiled!"""
    funcPosDelta = posTwo-posOne #IMPORTANT: only works on np.arrays
    returnData = np.zeros(2)
    returnData[1] = np.arctan2(funcPosDelta[1], funcPosDelta[0])
    if(abs(funcPosDelta[0]) < 0.0001): #sin(angle) can be 0, which results in divide by 0 errors.
        returnData[0] = abs(funcPosDelta[1])
    elif(abs(funcPosDelta[1]) < 0.0001): #floating point error, alternatively you could check the angle
        returnData[0] = abs(funcPosDelta[0])
    else:
        returnData[0] = funcPosDelta[1]/np.sin(returnData[1])  #soh
        #funcDistance = funcPosDelta[0]/np.cos(returnData[1])  #cah
    return(returnData)

@njit
def distSqrdBetwPos(posOne, posTwo): #returns distance^2 between 2 positions (useful for efficient distance thresholding)
    """get distance squared between 2 positions (2-sized arrays/lists), useful for efficient distance thresholding (compare to threshold squared)
        numba compiled!"""
    return((posTwo[0]-posOne[0])**2 + (posTwo[1]-posOne[1])**2)  #A^2 + B^2 = C^2
    #return(np.sum((posTwo-posOne)**2)) #slower

@njit
def distAnglePosToPos(funcRadius, funcAngle, funcPos): #returns a new pos given an angle, distance and starting pos
    """get position that is the entered distance and angle away from the entered position
        numba compiled!"""
    return(np.array([funcPos[0] + funcRadius * np.cos(funcAngle), funcPos[1] + funcRadius * np.sin(funcAngle)]))

@njit
def vectorProjectDist(posOne: np.ndarray, posTwo: np.ndarray, angleToProjectOnto: np.float32): #returns the x and y distance after rotating by angleToProjectOnto from posOne
    """get x and y distance between two positions in a coordinate system that is rotated by the entered angle
        numba compiled!"""
    #approach (loosely) derived from vector math (not my (thijs) area of maximum expertise)
    posDeltas = [posTwo[0] - posOne[0], posTwo[1] - posOne[1]]
    cosAndSin = [np.cos(angleToProjectOnto), np.sin(angleToProjectOnto)]
    return(np.array([posDeltas[0]*cosAndSin[0] + posDeltas[1]*cosAndSin[1], posDeltas[1]*cosAndSin[0] - posDeltas[0]*cosAndSin[1]]))
    # # system shifting approach
    # hypotenuse, oldAngle = distAngleBetwPos(posOne, posTwo)
    # return(np.cos(oldAngle-angleToProjectOnto)*hypotenuse, np.sin(oldAngle-angleToProjectOnto)*hypotenuse)
    ##to be clear, this function is mostly for Car.distanceToCar(), where posOne is the Car.position and angleToProjectOnto is Car.angle
    ##this makes it sothat you can do easy comparisons 

@njit
def findMinIndex(inputList: np.ndarray): #similar to np.argmax()
    """find the index that holds the minimum value, as well as that value (builtin min() only returns value, not index)
        numba compiled!"""
    if(len(inputList) > 0):
        returnIndex = 0
        minVal = inputList[0]
        for i in range(len(inputList)):
            if(inputList[i] < minVal):
                minVal = inputList[i]
                returnIndex = i
        return(returnIndex, minVal)
    else:
        return(-1, 0.0)

@njit
def findMaxIndex(inputList: np.ndarray): #similar to np.argmax()
    """find the index that holds the maximum value, as well as that value (builtin max() only returns value, not index)
        numba compiled!"""
    if(len(inputList) > 0):
        returnIndex = 0
        maxVal = inputList[0]
        for i in range(len(inputList)):
            if(inputList[i] > maxVal):
                maxVal = inputList[i]
                returnIndex = i
        return(returnIndex, maxVal)
    else:
        return(-1, 0.0)



#note: numpy as has an average() function, but (strangely enough) it's slower (and returns NaN when len()==0). I also found np.mean, havent tested it yet
@njit
def average(listOfValues: np.ndarray):
    """get average value of list (empty list returns 0)
        numba compiled!"""
    if(len(listOfValues) > 0): #avoid divide by 0
        return(np.sum(listOfValues)/len(listOfValues))
    else:
        #print("averaging 0 values!?")
        return(0.0)

def compileAll(verbose=False):
    ## precompile things by running them once
    print("precompiling njit generalFunctions...")
    import time
    compileStartTime = time.time()
    
    get_angle_between(np.array([0.5, 2.5]), np.array([2.5, 0.5]), 0.78)
    degRoll(721.0)
    radRoll(np.pi*4.125)
    degDiff(35.0, 135.0)
    degMidd(35.0, 135.0)
    radDiff(np.pi/5, np.pi-0.1)
    radMidd(np.pi/5, np.pi-0.1)
    simpleRange(5.0, 1.0, 9.0); simpleRange(170.0, -80, 14)
    degRange(405.0, 331.0, 480.0); degRange(-112.0, 331.0, -361.0)
    degInv(-741.0)
    radRange(2.1*np.pi, np.pi*1.9, 2.9*np.pi); radRange(-0.8*np.pi, 1.9*np.pi, -2*np.pi)
    radInv(3.1*np.pi)
    get_norm_angle_between(np.array([0.5, 2.5]), np.array([2.5, 0.5]), 0.78)
    distAngleBetwPos(np.array([0.5, 2.5]), np.array([2.5, 0.5]))
    distSqrdBetwPos(np.array([0.5, 2.5]), np.array([2.5, 0.5]))
    distAnglePosToPos(14.01, np.pi/5.2, np.array([0.5, 2.5]))
    vectorProjectDist(np.array([0.5, 2.5]), np.array([2.5, 0.5]), -np.pi/2.2)
    findMinIndex(np.random.rand(100))
    findMaxIndex(np.random.rand(100))
    average(np.random.rand(100))
    print("generalFunctions njit compilation done! (took", round(time.time()-compileStartTime,1), "seconds)")
    if(verbose):
        def inspect(compiledFunction):
            print(compiledFunction.__name__, compiledFunction.signatures, type(compiledFunction.signatures[0][0]))
            compiledFunction.inspect_types(file)
        ## verbose output of njit functions (to file)
        print("verbose output of generalFunctions njit compilation:")
        file = open("generalFunctions njit verbose output.txt", "w+")
        inspect(get_angle_between)
        inspect(degRoll)
        inspect(radRoll)
        inspect(degDiff)
        inspect(degMidd)
        inspect(radDiff)
        inspect(radMidd)
        inspect(simpleRange)
        inspect(degRange)
        inspect(degInv)
        inspect(radRange)
        inspect(radInv)
        inspect(get_norm_angle_between)
        inspect(distAngleBetwPos)
        inspect(distSqrdBetwPos)
        inspect(distAnglePosToPos)
        inspect(vectorProjectDist)
        inspect(findMinIndex)
        inspect(findMaxIndex)
        inspect(average)
        file.close()

if __name__ == "__main__":
    compileAll()