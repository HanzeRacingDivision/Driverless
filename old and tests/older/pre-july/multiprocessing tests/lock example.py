import time
import multiprocessing as MP

aLock = MP.Lock()

def procFunc(procLock):
    print("proc init")
    while(True):
        procLockStart = time.time()
        with procLock:
            print("proc lock get time:", round(time.time()-procLockStart,3))
            time.sleep(1)
        time.sleep(0.005)

if __name__ == "__main__":
    try:
        print("main init")
        P = MP.Process(target=procFunc, args=(aLock, ))
        P.start()
        while(True):
            procLockStart = time.time()
            with aLock:
                print("main lock get time:", round(time.time()-procLockStart,3))
                time.sleep(1)
            time.sleep(0.005)
    finally:
        try:
            P.join()
        except:
            print("couldn't join P")