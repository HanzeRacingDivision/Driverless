from numba import njit, prange
import numpy as np
import time


import generalFunctions as GF


@njit
def randomPositions(n, scale, zeroCenter=True):
    if(zeroCenter):
        return((np.random.rand(n,2)-0.5)*scale)
    else:
        return(np.random.rand(n,2)*scale)

@njit
def randomDegAngles(n,m, zeroCenter=True):
    if(zeroCenter):
        return((np.random.rand(n,m)-0.5)*360.0)
    else:
        return(np.random.rand(n,m)*360.0)


@njit
def randomRadAngles(n,m, zeroCenter=True):
    twoPi = (np.pi*2)
    if(zeroCenter):
        return((np.random.rand(n,m)-0.5)*twoPi)
    else:
        return(np.random.rand(n,m)*twoPi)


# class someClass:
#     def __init__(self, val):
#         self.val = val

# def classList(n):
#     output = np.empty(n, dtype=someClass)
#     for i in range(n):
#         output[i] = someClass(i)
#     return(output)

# def classClassList(n,m):
#     output = np.empty(n, dtype=np.ndarray)
#     for i in range(n):
#         output[i] = classList(m)
#     return(output)


#@njit(parallel=False)
def doThing(argOne: np.ndarray):
    output = np.empty(len(argOne))
    for i in prange(len(output)):
        #output[i] = np.average(argOne[i]) #doesnt support njit (?)
        output[i] = GF.average(argOne[i])
    return(output)

N = 100000

posList = randomDegAngles(N,10)
times = np.zeros(10)

times[0] = time.time_ns()
result = doThing(posList)
times[1] = time.time_ns()
# print(posList)
# print(result)
# print()
posListTwo = randomDegAngles(N,10)
times[2] = time.time_ns()
result = doThing(posListTwo)
times[3] = time.time_ns()
# print(posListTwo)
# print(result)
# print()
posListTwo = randomDegAngles(N,10)
times[4] = time.time_ns()
result = doThing(posListTwo)
times[5] = time.time_ns()
# print(posListTwo)
# print(result)
# print()
print((times[1]-times[0]) / 1000000)
print((times[3]-times[2]) / 1000000)
print((times[5]-times[4]) / 1000000)









# @njit
# def appendLists(n):
#     lst = []
#     for i in range(n):
#         lst.append(i**2)
#     return lst

# @njit
# def fillInLists(n):
#     lst = np.empty(n)
#     for i in range(n):
#         lst[i] = i**2
#     return lst

# #@njit(parallel=False)
# def doThing(n):
#     output = np.empty(n)
#     for i in prange(n):
#         #output = len(appendLists(10000))
#         output = len(fillInLists(10000)) #about 72 times faster
#     return(output)

# N = 1000

# times = np.zeros(10)

# times[0] = time.time_ns()
# result = doThing(N)
# times[1] = time.time_ns()
# result = doThing(N)
# times[2] = time.time_ns()
# result = doThing(N)
# times[3] = time.time_ns()
# print((times[1]-times[0]) / 1000000)
# print((times[2]-times[1]) / 1000000)
# print((times[3]-times[2]) / 1000000)








# #@njit(parallel=False)
# def doThing(argOne: np.ndarray):
#     output = np.empty(len(argOne)-1)
#     #output = np.empty((len(argOne)-1,2))
#     for i in prange(len(output)):
#         #output[i] = (argOne[i+1][0] - argOne[i][0])**2 + (argOne[i+1][1] - argOne[i][1])**2
#         output[i] = GF.distSqrdBetwPos(argOne[i], argOne[i+1])
#         #output[i] = GF.vectorProjectDist(argOne[i], argOne[i+1], 1.1) #uncomment 2D output list!
#         #output[i] = GF.distAngleBetwPos(argOne[i], argOne[i+1]) #uncomment 2D output list!
#     return(output)


# N = 1000000

# posList = randomPositions(N, 10)
# times = np.zeros(10)

# times[0] = time.time_ns()
# result = doThing(posList)
# times[1] = time.time_ns()
# # print(posList)
# # print(result)
# # print()
# posListTwo = randomPositions(N, 10)
# times[2] = time.time_ns()
# result = doThing(posListTwo)
# times[3] = time.time_ns()
# # print(posListTwo)
# # print(result)
# # print()
# posListTwo = randomPositions(N, 10)
# times[4] = time.time_ns()
# result = doThing(posListTwo)
# times[5] = time.time_ns()
# # print(posListTwo)
# # print(result)
# # print()
# print((times[1]-times[0]) / 1000000)
# print((times[3]-times[2]) / 1000000)
# print((times[5]-times[4]) / 1000000)