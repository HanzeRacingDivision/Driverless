import numpy as np
import math

origins = np.array([[ 10.0,  10.0],
                    [ 10.0, -10.0],
                    [-10.0, -10.0],
                    [-10.0,  10.0]])

shift = np.array([2.0, 3.0])
origins += shift
print(origins)

initialGuess = shift + np.array([2.34, 0.0])

result = initialGuess.copy()

JacobUnitVects = np.empty((len(origins), len(initialGuess)))
vectLens = np.empty(len(origins))
for j in range(len(origins)):
    JacobUnitVects[j] = origins[j]-result #get a vector that points from the target position to a landmark/origin
    vectLens[j] = math.hypot(*JacobUnitVects[j])
    if(vectLens[j] > 0.00001):
        JacobUnitVects[j] /= vectLens[j] #make into unit vector (normalize length to 1.0)
    else:
        print("target position perfectly aligned with a landmark?:", result, origins[j])

print(JacobUnitVects)
sumOfjacobians = np.sum(JacobUnitVects, 0)
print(sumOfjacobians)
print(np.sqrt(np.abs(sumOfjacobians[0])) / np.sqrt(np.sum()