import numpy as np
import matplotlib.pyplot as plt #for the graphs
import math


points = np.array([[10.0, 10.0],
                   [-8.0, 4.0],
                   [-5.0, -1.0]])
trueTarget = np.array([1.23, 0.456])

trueRadii = np.array([np.hypot(*(trueTarget-point)) for point in points])
measurementError = np.random.random(trueRadii.shape)
badRadii = trueRadii - measurementError
print(badRadii)
print(measurementError)

# def hypotonuse(*args): #a manual (without math library) N dimensional pythagorean
#     return(sum([arg**2 for arg in args])**0.5)
#     #return(math.hypot(xDif, yDif, zDif))

def errorsAbs(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray):
    """calculates the sum of measurement errors at a given target position
        input format: target: (dim), origins: (N, dim), measuredRadii: (N), 
         where N is the number of circles to intersect and dim is the number of spacial dimensions (usually 2 or 3)
         IMPORTANT: this function REQUIRES numpy arrays (at least for target and origins)"""
    # # posDif = np.empty_like(origins)
    # posDif = target - origins #for numpy arrays, a 1D array - a 2D array makes it subtract the 1D array from all entries of the 2D array
    # calculatedRadii = np.empty_like(measuredRadii)
    errors = np.empty_like(measuredRadii)
    for i in range(len(origins)):
        # # #posDif[i] = np.array([target[0]-origins[i][0], target[1]-origins[i][1]]) #only 2D
        # # posDif[i] = target - origins[i] #ONLY WORKS WITH NUMPY ARRAYS!
        # #calculatedRadii[i] = hypotonuse(*posDif[i])
        # calculatedRadii[i] = math.hypot(*posDif[i])
        # errors[i] = abs(calculatedRadii[i] - measuredRadii[i])
        errors[i] = abs(math.hypot(*(target - origins[i])) - measuredRadii[i])
    return(errors)

def errorsSquared(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray):
    """calculates the sum of squared measurement errors at a given target position
        input format: target: (dim), origins: (N, dim), measuredRadii: (N), 
         where N is the number of circles to intersect and dim is the number of spacial dimensions (usually 2 or 3)
         IMPORTANT: this function REQUIRES numpy arrays (at least for target and origins)"""
    # # posDif = np.empty_like(origins)
    # posDif = target - origins #for numpy arrays, a 1D array - a 2D array makes it subtract the 1D array from all entries of the 2D array
    # calculatedRadii = np.empty_like(measuredRadii)
    errors = np.empty_like(measuredRadii)
    for i in range(len(origins)):
        # # #posDif[i] = np.array([target[0]-origins[i][0], target[1]-origins[i][1]]) #only 2D
        # # posDif[i] = target - origins[i] #ONLY WORKS WITH NUMPY ARRAYS!
        # #calculatedRadii[i] = hypotonuse(*posDif[i])
        # calculatedRadii[i] = math.hypot(*posDif[i])
        # errors[i] = (calculatedRadii[i] - measuredRadii[i])**2
        errors[i] = (math.hypot(*(target - origins[i])) - measuredRadii[i])**2
    return(errors)


# def errorsAbsPartialDer(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray, axis: int):
#     """calculates the sum of measurement errors at a given target position
#         input format: target: (dim), origins: (N, dim), measuredRadii: (N), 
#           where N is the number of circles to intersect and dim is the number of spacial dimensions (usually 2 or 3)
#           IMPORTANT: this function REQUIRES numpy arrays (at least for target and origins)"""
#     # # posDif = np.empty_like(origins)
#     # calculatedRadii = np.empty_like(measuredRadii)
#     # posDif = target - origins #for numpy arrays, a 1D array - a 2D array makes it subtract the 1D array from all entries of the 2D array
#     deltaError = np.empty_like(measuredRadii)
#     for i in range(len(origins)):
#         #posDif = np.array([target[0]-origins[i][0], target[1]-origins[i][1]]) #only 2D
#         posDif = target - origins[i] #ONLY WORKS WITH NUMPY ARRAYS!
#         #calculatedRadii = hypotonuse(*posDif[i])
#         calculatedRadii = math.hypot(*posDif[i])
        
#         distDif = calculatedRadii - measuredRadii[i]
#         if(abs(distDif) < 0.0001):
#             print("sumOfErrorsAbsPartialDerX2DFast avoiding 0 / 0 = 0")
#             deltaError[i] = 0.0
#         else:
#             deltaError[i] = (posDif[axis] * distDif) / (calculatedRadii * abs(distDif))
#     return(deltaError)

def errorsSquaredPartialDer(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, axis: int):
    """calculates the sum of measurement errors at a given target position
        input format: target: (dim), origins: (N, dim), measuredRadii: (N), 
         where N is the number of circles to intersect and dim is the number of spacial dimensions (usually 2 or 3)
         IMPORTANT: this function REQUIRES numpy arrays (at least for target and origins)"""
    # # posDif = np.empty_like(origins)
    # calculatedRadii = np.empty_like(measuredRadii)
    # posDif = target - origins #for numpy arrays, a 1D array - a 2D array makes it subtract the 1D array from all entries of the 2D array
    deltaError = np.empty_like(measuredRadii)
    for i in range(len(origins)):
        #posDif = np.array([target[0]-origins[i][0], target[1]-origins[i][1]]) #only 2D
        posDif = target - origins[i] #ONLY WORKS WITH NUMPY ARRAYS!
        #calculatedRadii = hypotonuse(*posDif[i])
        calculatedRadii = math.hypot(*posDif[i])
        
        distDif = calculatedRadii - measuredRadii[i]
        deltaError[i] = 2.0 * (posDif[axis] * distDif) / calculatedRadii
    return(deltaError)

def changeOneAxis(value, array, axis):
    changedArray = array.copy()
    changedArray[axis] = value
    return(changedArray)

def findMinimumErrorsAbsAllAxis(initialGuess: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray):
    """find the position to minimize sum of abs error (should work for any (2+) dimensional space)"""
    errorAtMinimum = 0.0
    posAtMinimum = initialGuess.copy()
    for i in range(len(initialGuess)):
        posAtMinimum, errorAtMinimum = findMinimumErrorsAbs(posAtMinimum, origins, measuredRadii, i)
    return(posAtMinimum, errorAtMinimum)

def findMinimumErrorsAbs(initialGuess: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray, axis: int):
    """for abs error, there is an inflection point where it normally crosses zero, but (due to abs()) changes direction.
        This means the lowest point in the sum of abs error is found at one of these inflection points.
        this function calculates the location of the inflection points, and the error value at that point, to find the minimal error"""
    valuesAtPossibleMinima = np.empty((len(measuredRadii)*2, 2))
    for i in range(len(measuredRadii)):
        sumOfSquaredPerpendicularComponents = 0.0
        for j in range(len(initialGuess)-1): # all of this is to make it work in 2D as well as 3D, it's still just pythagorean
            notAxis = (axis+1+j)%len(initialGuess) #the indices of all axis that arent the entered one
            sumOfSquaredPerpendicularComponents += (origins[i][notAxis]-initialGuess[notAxis])**2
        possibleMinimaSquared = measuredRadii[i]**2 - sumOfSquaredPerpendicularComponents # A^2 = Z^2 - B^2 + C^2 + D^2 + E^2 ...etc, pythagorean
        if(possibleMinimaSquared < 0.0): ## i don't want to sqrt() a negative number.
            #print("avoiding NaN:", possibleMinimaSquared, measuredRadii[i])
            valuesAtPossibleMinima[i*2][1] = 1000000.0 #functional infinity
            valuesAtPossibleMinima[i*2 + 1][1] = 1000000.0 #functional infinity
        else:
            possibleMinima = np.sqrt(possibleMinimaSquared) # A = sqrt(C**2 - B**2)
            valuesAtPossibleMinima[i*2][0] = origins[i][axis]-possibleMinima
            #posAtPossibleMinima = changeOneAxis(valuesAtPossibleMinima[i*2][0], initialGuess, axis)
            posAtPossibleMinima = initialGuess.copy()
            posAtPossibleMinima[axis] = valuesAtPossibleMinima[i*2][0]
            valuesAtPossibleMinima[i*2][1] = np.sum(errorsAbs(posAtPossibleMinima, origins, measuredRadii))
            valuesAtPossibleMinima[i*2 + 1][0] = origins[i][axis]+possibleMinima
            posAtPossibleMinima[axis] = valuesAtPossibleMinima[i*2 + 1][0]
            valuesAtPossibleMinima[i*2 + 1][1] = np.sum(errorsAbs(posAtPossibleMinima, origins, measuredRadii))
            #print("possibleMinima:", i, possibleMinimaSquared, possibleMinima, valuesAtPossibleMinima[i*2], valuesAtPossibleMinima[i*2 + 1])
    minimumIndex = np.argmin(valuesAtPossibleMinima[:,1]) #unfortunately, there is no numpy function for returning a minimum index while ignoring NaN. 
    ## (i don't feel like writing my own forloop to return the minimum index ignoring NaNs)
    #print("predicting minimum:", minimumIndex, valuesAtPossibleMinima[minimumIndex][0], valuesAtPossibleMinima[minimumIndex][1])
    posAtMinima = initialGuess.copy()
    posAtMinima[axis] = valuesAtPossibleMinima[minimumIndex][0]
    #posAtMinima = changeOneAxis(valuesAtPossibleMinima[minimumIndex][0], initialGuess, axis)
    return(posAtMinima, valuesAtPossibleMinima[minimumIndex][1])


def findMinimumErrorsSquaredAllAxis(initialGuess: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray):
    """find the position to minimize sum of abs error (should work for any (2+) dimensional space)"""
    errorAtMinimum = 0.0
    posAtMinimum = initialGuess.copy()
    for i in range(len(initialGuess)):
        posAtMinimum, errorAtMinimum = findMinimumErrorsSquared(posAtMinimum, origins, measuredRadii, i)
    return(posAtMinimum, errorAtMinimum)

def secant(func, absErrorThresh, maxItt, lastOutput, lastInput, nextInput, itt=0):
    """the secant method is very similar to the Newton-Raphson,
        please just google it for a real explenation.
        input parameters: (func: the function to find the zero-crossing for, absErrorThresh: if abs(func()) output drops below this threshold then secant stops,
                           maxItt: maximum number of itterations before forced stop (if absErrorThresh is not reached), lastOutput: func() at last input,
                           lastInput: most recent (or first) input to func(), nextInput: the next input to func() (first value is any arbirtary value)
        this function works recursively, so at large maxItt's you might run into python errors. Secant isn't meant for many itterations, check absErrorThresh and maxItt."""
    if(itt >= maxItt):
        return(nextInput)
    nextOutput = func(nextInput)
    if(absErrorThresh < abs(nextOutput)):
        a = (nextOutput - lastOutput) / (nextInput - lastInput)
        b = nextOutput - (a * nextInput)
        zeroInput = -b / a
        #zeroOutput = func(zeroInput)
        return(secant(func, absErrorThresh, maxItt, nextOutput, nextInput, zeroInput, itt + 1))
    else:
        return(nextInput)

def findMinimumErrorsSquared(initialGuess: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray, axis: int):
    """for squared error, the formula is NEARLY parabolic, and so the derivative is NEARLY straight.
        because of this, a very short secant(ish) zero-crossing-finding algorithem is used:
            a straight line between 2 arbirtary points is calculated, this line's zero-crossing position is calculated,
            repeat this process a few times, to really hone in on the exact zero-crossing"""
    firstAxisStep = initialGuess[axis] + 0.1 #i'm pretty sure this doesn't matter at all.
    secantFunc = lambda secantInput : np.sum(errorsSquaredPartialDer(changeOneAxis(secantInput, initialGuess, axis), origins, measuredRadii, axis))
    posAtMinima = changeOneAxis(secant(secantFunc, 0.001, 3, secantFunc(initialGuess[axis]), initialGuess[axis], firstAxisStep), initialGuess, axis)
    return(posAtMinima, np.sqrt(np.sum(errorsSquared(posAtMinima, origins, measuredRadii))))

# badPos = trueTarget + np.array([0.09865, -0.12345])
# returned = findMinimumErrorsAbs(badPos, points, badRadii, 0)
# print("returned:", returned)
# returned = findMinimumErrorsAbs(returned[0], points, badRadii, 1)
# print("returned:", returned)
# # returned = findMinimumErrorsAbsAllAxis(returned[0], points, badRadii, 0) #prove that the minima still hold up
# # print("returned:", returned)


badPos = trueTarget + np.array([0.09865, -0.12345])
print("badPos:", badPos)
returned = findMinimumErrorsSquared(badPos, points, badRadii, 0)
print("returned:", returned)
returned = findMinimumErrorsSquared(returned[0], points, badRadii, 1)
print("returned:", returned)
# returned = findMinimumErrorsAbs(returned[0], points, badRadii, 0) #prove that the X minima still holds up
# print("returned:", returned)
# returned = findMinimumErrorsAbs(returned[0], points, badRadii, 1)
# print("returned:", returned)



# N = 100
# graphX = np.empty(N)
# graphOne = np.empty(N)
# #graphTwo = np.empty(N)
# graphTwo = np.empty((N, len(badRadii)))
# for i in range(N):
#     offsetTarget = trueTarget.copy(); offsetTarget[0] += ((i-((N-1)/2))/N)*3.0
#     graphX[i] = offsetTarget[0]
#     # graphOne[i] = sumOfErrorsAbs2D(offsetTarget, points, badRadii)[0]
#     # graphTwo[i] = sumOfErrorsSquared2D(offsetTarget, points, badRadii)[0]
#     graphTwo[i] = errorsSquared(offsetTarget, points, badRadii)[0]
#     graphOne[i] = np.sum(graphTwo[i])
#     if(i > 1):
#         for j in range(len(badRadii)):
#             if((graphTwo[i][j] > graphTwo[i-1][j]) and (graphTwo[i-1][j] < graphTwo[i-2][j])):
#                 print("found zero-crossing:", offsetTarget[0], )

# plt.plot(graphX, graphOne)
# names = ["sum of error"]
# for i in range(len(badRadii)):
#     plt.plot(graphX, graphTwo[:,i])
#     names.insert(i+1, "individual error "+str(i))
# #plt.plot(graphX, np.zeros(len(graphX)))
# plt.legend(names)
# plt.show()


# #N = 100
# graphOneDer = np.empty(N)
# #graphTwoDer = np.empty(N)
# graphTwoDer = np.empty((N, len(badRadii)))
# for i in range(N):
#     offsetTarget = trueTarget.copy(); offsetTarget[0] += ((i-((N-1)/2))/N)*3.0
#     #graphOneDer[i] = sumOfErrorsAbsPartialDerX2D(offsetTarget, points, badRadii, 0)[0]
#     #graphTwoDer[i] = sumOfErrorsSquaredPartialDerX2D(offsetTarget, points, badRadii, 0)[0]
#     graphTwoDer[i] = errorsSquaredPartialDer(offsetTarget, points, badRadii, 0)[0]
#     graphOneDer[i] = np.sum(graphTwoDer[i])
#     # if(i > 0):
#     #     if((graphOneDer[i] - graphOneDer[i-1]) > 0.5):
#     #         print("found large increment:", offsetTarget[0])
#     #         if((graphOneDer[i] > 0.0) and (graphOneDer[i-1] < 0.0)):
#     #             print("found minima?:",  offsetTarget[0], graphOneDer[i])

# plt.plot(graphX, graphOneDer)
# names = ["sum of derivatives", "zero"]
# for i in range(len(badRadii)):
#     plt.plot(graphX, graphTwoDer[:,i])
#     names.insert(i+1, "individual der. "+str(i))
# plt.plot(graphX, np.zeros(len(graphX)))
# plt.legend(names)
# plt.show()


## newton-rahpson method
# newtRaphMax = 100
# secondGraphX = []
# idealXgraphOne = []
# idealX = trueTarget.copy() + np.array([0.2, 0.3])
# for i in range(newtRaphMax):
#     FxDer = sumOfErrorsAbsPartialDerX2D(idealX, points, badRadii)[0]
#     if(abs(FxDer) > 0.0001):
#         Fx = sumOfErrorsAbs2D(idealX, points, badRadii)[0]
        
#         secondGraphX.append(idealX[0])
#         idealXgraphOne.append(Fx)
        
#         update = (Fx / FxDer)
#         #print(update)
#         idealX[0] = idealX[0] - update
#         if(abs(update) < 0.001):
#             print("found minima:", i, idealX, Fx)
#             break
#     else:
#         print("avoiding divide by 0:", i, idealX)
#         break

# thirdGraphX = []
# idealXgraphTwo = []
# idealXalso = trueTarget.copy() + np.array([0.2, 0.3])
# for i in range(newtRaphMax):
#     FxDer = sumOfErrorsSquaredPartialDerX2D(idealXalso, points, badRadii)[0]
#     if(abs(FxDer) > 0.0001):
#         Fx = sumOfErrorsSquared2D(idealXalso, points, badRadii)[0]
        
#         thirdGraphX.append(idealXalso[0])
#         idealXgraphTwo.append(Fx)
        
#         update = (Fx / FxDer)
#         #print(update)
#         idealXalso[0] = idealXalso[0] - update
#         if(abs(update) < 0.001):
#             print("also found minima:", i, idealXalso, Fx)
#             break
#     else:
#         print("also avoiding divide by 0:", i, idealXalso)
#         break

## secant method
# secantMax = 100
# thirdGraphX = []
# idealXgraphTwo = []
# idealXalso = trueTarget.copy() + np.array([0.2, 0.3])
# xPrev = trueTarget.copy() + np.array([-0.2, -0.3])
# FxPrev = sumOfErrorsSquared2D(xPrev, points, badRadii)[0]
# for i in range(secantMax):
#     Fx = sumOfErrorsSquared2D(idealXalso, points, badRadii)[0]
    
#     thirdGraphX.append(idealXalso[0])
#     idealXgraphTwo.append(Fx)
    
#     if(abs(Fx - FxPrev) < 0.001):
#         print("also found minima because Fx == FxPrev:", i, idealXalso, Fx)
#         break
#     update = Fx * ((idealXalso[0] - xPrev[0]) / (Fx - FxPrev))
#     #print("test:", update, Fx, (idealXalso[0] - xPrev[0]))
#     xPrev[0] = idealXalso[0]
#     FxPrev = Fx
#     idealXalso[0] = idealXalso[0] - update
#     if(abs(update) < 0.001):
#         print("also found minima:", i, idealXalso, Fx)
#         break

# plt.plot(graphX, graphOne)
# plt.plot(graphX, graphTwo)
# plt.legend(["sum of abs(error)", "sum of (error)^2"])
# plt.show()


# plt.plot(secondGraphX)
# plt.show()
# plt.plot(graphX, graphTwo)
# plt.plot(thirdGraphX, idealXgraphTwo)
# plt.show()