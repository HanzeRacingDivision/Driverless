import time
import multiprocessing as MP

connOne, connTwo = MP.Pipe(True)


def procFunc(procPipe):
    print("proc init")
    while(True):
        if(procPipe.poll()):
            print("proc got:", procPipe.recv())
            procPipe.send("pong")
            print("proc after recv poll:", procPipe.poll())
            if(procPipe.poll()):
                print("proc SECOND got:", procPipe.recv())
        time.sleep(1)

if __name__ == "__main__":
    try:
        print("main init")
        P = MP.Process(target=procFunc, args=(connTwo, ))
        P.start()
        time.sleep(0.5)
        while(True):
            procLockStart = time.time()
            if(connOne.poll()):
                print("main got:", connOne.recv())
                print("main after recv poll:", connOne.poll())
                if(connOne.poll()):
                    print("main SECOND got:", connOne.recv())
            connOne.send("ping")
            time.sleep(1)
    finally:
        try:
            P.join()
        except:
            print("couldn't join P")