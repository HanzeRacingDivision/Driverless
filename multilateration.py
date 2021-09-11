## "multilateration" is the process of finding where 3+ circles (directionless distance measurements) intersect
## this method attempts to minimize X, Y (and Z) error by finding the minima in the error-over-X graph (a.k.a. where the (partial) derivative of X (with respect to summed sensor error) equals 0)
import math
import numpy as np

##TBD: if you somehow knew sensor error (or an estimate of it), you could compute a position that minimizes weighted sensor error.
##      like, if some sensors are known to suck, then the magnitude of their error matters less.


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
        errors[i] = abs(math.hypot(*(target - origins[i])) - measuredRadii[i]) * measurementGains[i]
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
        errors[i] = ((math.hypot(*(target - origins[i])) - measuredRadii[i])**2) * measurementGains[i]
        #errors[i] = ((math.hypot(*(target - origins[i])) - measuredRadii[i]) * measurementGains[i])**2  #i think it makes more sense to multiply the squared eror with the gain, but who knows
    return(errors)


def errorsAbsPartialDer(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray, axis: int):
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
        #calculatedRadii = hypotonuse(*posDif)
        calculatedRadii = math.hypot(*posDif)
        
        distDif = calculatedRadii - measuredRadii[i]
        if(abs(distDif) < 0.0001):
            print("sumOfErrorsAbsPartialDerX2DFast avoiding 0 / 0 = 0")
            deltaError[i] = 0.0
        else:
            deltaError[i] = ((posDif[axis] * distDif) / (calculatedRadii * abs(distDif))) * measurementGains[i]
    return(deltaError)

def errorsSquaredPartialDer(target: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray, axis: int):
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
        #calculatedRadii = hypotonuse(*posDif)
        calculatedRadii = math.hypot(*posDif)
        
        distDif = calculatedRadii - measuredRadii[i]
        deltaError[i] = 2.0 * ((posDif[axis] * distDif) / calculatedRadii) * measurementGains[i]
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
        posAtMinimum, errorAtMinimum = findMinimumErrorsAbs(posAtMinimum, origins, measuredRadii, measurementGains, i)
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
            valuesAtPossibleMinima[i*2][1] = np.sum(errorsAbs(posAtPossibleMinima, origins, measuredRadii, measurementGains))
            valuesAtPossibleMinima[i*2 + 1][0] = origins[i][axis]+possibleMinima
            posAtPossibleMinima[axis] = valuesAtPossibleMinima[i*2 + 1][0]
            valuesAtPossibleMinima[i*2 + 1][1] = np.sum(errorsAbs(posAtPossibleMinima, origins, measuredRadii, measurementGains))
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
        posAtMinimum, errorAtMinimum = findMinimumErrorsSquared(posAtMinimum, origins, measuredRadii, measurementGains, i)
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
    secantFunc = lambda secantInput : np.sum(errorsSquaredPartialDer(changeOneAxis(secantInput, initialGuess, axis), origins, measuredRadii, measurementGains, axis))
    posAtMinima = changeOneAxis(secant(secantFunc, 0.001, 3, secantFunc(initialGuess[axis]), initialGuess[axis], firstAxisStep), initialGuess, axis)
    return(posAtMinima, np.sqrt(np.sum(errorsSquared(posAtMinima, origins, measuredRadii, measurementGains))))


def multilaterate(initialGuess: np.ndarray, origins: np.ndarray, measuredRadii: np.ndarray, measurementGains: np.ndarray):
    """find the best approximation of the intersection between several circles (defined by origins and radii).
        can be used to find a position estimate for a sensor system that gives directionless distance measurements
        requires initialGuess, but i'm not sure how much it matters"""
    #return(findMinimumErrorsAbsAllAxis(initialGuess, origins, measuredRadii, measurementGains))    #my own creation
    return(findMinimumErrorsSquaredAllAxis(initialGuess, origins, measuredRadii, measurementGains)) #proven-to-work 'least squares' approach