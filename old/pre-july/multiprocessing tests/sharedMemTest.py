import multiprocessing as MP
from multiprocessing import shared_memory as MultiMem
import numpy as np
import time

print("loaded sharedMemTest", __name__ == '__main__')

lidarPacket = np.dtype([('RPM', np.uint16),
                        ('startAngle', np.uint16),
                        ('endAngle', np.uint16),
                        ('timestamp', np.float64),
                        ('measurements', np.uint16, (8,)),
                        ('reservedData', np.uint8, (8,)),
                        ('dataFilled', np.int16),
                        ('CRCpassed', bool)])

mainMemName = "" #shared memory name (for main thread only)



def testFunc(lock, memName, memSize, arrayShape):
    print("testFunc", memName, arrayShape)
    try:
        another = MultiMem.SharedMemory(memName, size=memSize) #size argument probably doesnt apply, but whatever
        print("another:", len(another.buf), memSize)
        with lock:
            c = np.ndarray(arrayShape, dtype=lidarPacket, buffer=another.buf[0:memSize]) #only the dtype needs to be manually set, the shape can be done automatically
            print('c', c[0]['RPM'])
            c[0]['RPM'] = 5
            c[1] = c[0]
            c[0]['RPM'] = 10
            print('c', c[0]['RPM'])
    finally:
        try:
            another.close()
        except:
            print("didnt close shared memory from thread")


if __name__ == '__main__': ##IMPORTANT: every multiprocessing thread will import THIS (the instigating) file, so without this check it will try to run all code twice(/infinitely)
    try:
        another = MultiMem.SharedMemory(mainMemName)
        another.close()
        another.unlink()
        print("closed leftover sharedmem")
    except:
        print("didnt close leftover sharedmem")
    
    try:
        ## init
        maxPacketArrayLen = np.uint16(100) #constant
        print("main init", maxPacketArrayLen)
        listInitializer = np.zeros(maxPacketArrayLen, dtype=lidarPacket)
        sharedMem = MultiMem.SharedMemory(None, True, size=listInitializer.nbytes) #note: size will (in the threads that use it) as integer scalar of 4096 (ram chuck size or HDD (formatting) chunk size?)
        b = np.ndarray(maxPacketArrayLen, dtype=lidarPacket, buffer=sharedMem.buf)
        b[:] = listInitializer[:] #zero out array
        memLock = MP.Lock()
        with memLock:
            print('b', b[0]['RPM'], b[1]['RPM'])
            #print('Btyp', b.dtype)
        time.sleep(1)
        
        ## make process
        P = MP.Process(target=testFunc, args=(memLock, sharedMem.name, b.nbytes, b.shape))
        P.start()
        # P.join()
        time.sleep(3)
        print('b', b[0]['RPM'], b[1]['RPM'])
        
    finally:
        try:
            P.join()
        except:
            print("couldn't join P")
        try:
            sharedMem.close()
            sharedMem.unlink()
        except:
            print("DIDNT CLOSE 'sharedMem'")