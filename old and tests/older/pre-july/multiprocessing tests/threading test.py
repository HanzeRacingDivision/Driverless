# multi_threaded.py
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

t1 = Thread(target=countdown, args=(COUNT//2,))
t2 = Thread(target=countdown, args=(COUNT//2,))

start = time.time()
t1.start()
t2.start()
t1.join()
t2.join()
end = time.time()

print('Time taken in seconds -', end - start)
