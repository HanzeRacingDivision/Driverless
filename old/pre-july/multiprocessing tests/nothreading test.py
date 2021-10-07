# single_threaded.py
import time
from threading import Thread
from numba import njit
import numpy as np

COUNT = 100*1000*1000

@njit
def countdown(n):
    output = np.zeros(n)
    while n>0:
        n -= 1
        output[n] = n
    return(output)

print(len(countdown(10)))

start = time.time()
countdown(COUNT)
end = time.time()

print('Time taken in seconds -', end - start)
