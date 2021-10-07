import time
import multiprocessing as MP

aQueue = MP.Queue()

def procFunc(procQueue):
    print("proc init")
    while(True):
        if(not procQueue.empty()):
            print("proc got:", procQueue.get())
            procQueue.put("pong")
            print("proc queue empty?:", procQueue.empty())
            if(not procQueue.empty()):
                print("proc SECOND got:", procQueue.get())
        time.sleep(1)

if __name__ == "__main__":
    try:
        print("main init")
        P = MP.Process(target=procFunc, args=(aQueue, ))
        P.start()
        time.sleep(0.5)
        while(True):
            procLockStart = time.time()
            if(not aQueue.empty()):
                print("main got:", aQueue.get())
                print("main queue empty?:", aQueue.empty())
                if(not aQueue.empty()):
                    print("main SECOND got:", aQueue.get())
            aQueue.put("ping")
            time.sleep(1)
    finally:
        try:
            P.join()
        except:
            print("couldn't join P")