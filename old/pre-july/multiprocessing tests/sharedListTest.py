import multiprocessing as MP
from multiprocessing import shared_memory as MultiMem
import numpy as np
import time

print("loaded sharedMemTest", __name__ == '__main__')

class someClass:
    def __init__(self, one):
        self.one = one
    
    def __del__(self):
        print("deleting someClass:", self)

a = [someClass(5), someClass(6)]
#a = ["word", 5, 6.7]
#print('a', a, a.nbytes)

#mainMemName = "thisj" #shared memory name (for main thread only)



def testFunc(lock, memName):
    print("testFunc", memName)
    try:
        another = MultiMem.ShareableList(name=memName) #size argument probably doesnt apply, but whatever
        print("another:", len(another))
        #with lock:
        #c[0].one = 1000
        print('preC')
        print('c', another[0])
    finally:
        try:
            another.shm.close()
        except:
            print("didnt close shared memory from thread")


if __name__ == '__main__': ##IMPORTANT: every multiprocessing thread will import THIS (the instigating) file, so without this check it will try to run all code twice(/infinitely)
    # try:
    #     another = MultiMem.SharedMemory(mainMemName)
    #     another.close()
    #     another.unlink()
    #     print("closed leftover sharedmem")
    # except:
    #     print("didnt close leftover sharedmem")
    
    try:
        ## init
        print("main init", len(a))
        sharedMem = MultiMem.ShareableList(a) #note: size will (in the threads that use it) as integer scalar of 4096 (ram chuck size or HDD (formatting) chunk size?)
        memLock = MP.Lock()
        #with memLock:
        print('b', sharedMem, sharedMem[0])
        print('Btyp', type(sharedMem[0]))
        time.sleep(1)
        
        ## make process
        P = MP.Process(target=testFunc, args=(memLock, sharedMem.shm.name))
        P.start()
        # P.join()
        time.sleep(3)
        print('b', sharedMem, sharedMem[0])
        
    finally:
        try:
            P.join()
        except:
            print("couldn't join P")
        try:
            sharedMem.shm.close()
            sharedMem.shm.unlink()
        except:
            print("DIDNT CLOSE 'sharedMem'")