from camsense_X1 import camsense_X1, sharedMemArray
import multiprocessing as MP
import time

def callbackFunc(lidarSelf, newPacket, pointsAdded): #example callback function
    ##if((newPacket is not None) and (pointsAdded > 0)):
    #print(newPacket['startAngle']) #do note: this code will be run about 50packets/Rotation * 5rotations/sec = 250 times per second, so try to make this as quick as possible (like an interrupt function)
    if(lidarSelf.callbackExtraArg is not None):
        #if(newPacket['dataFilled'] == 8): #only on full packets
        lidarSelf.callbackExtraArg.put(("packet", newPacket, time.time()))

def processFunc(processQueue: MP.Queue, processDataLock: MP.Lock, lidarInitArgs):
    print("processFunc init:", lidarInitArgs)
    printTimer = time.time()
    try:
        lidar = camsense_X1(*lidarInitArgs)
        print("MPvars init:", lidar.startMultiProcessVars("liDaRam", processDataLock)) #returns True on success
        lidar.postParseCallback = callbackFunc
        lidar.callbackExtraArg = processQueue
        print(lidar)
        processQueue.put(("lidarData", lidar.getSharedMemName()))
        
        while(True):
            lidar.run() #needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
            
            # rightNow = time.time()
            # if((rightNow - printTimer) > 1.0):
            #     with lidar.dataLock:
            #         print(lidar.spinning, lidar.lidarDataLen[0], lidar.rotationCount[0], round(lidar.RPM(),1))
            #     printTimer = rightNow
    finally: #required for correct serial port closing (otherwise, you need to unplug and replug device every time)
        try:
            lidar._serialPort.close()
            print("closed lidar serial port")
        except:
            print("coudln't close lidar serial port")
        try:
            lidar.closeSharedMem()
        except:
            print("couldnt close lidar sharedMem")


if __name__ == '__main__': ##IMPORTANT: every multiprocessing thread will import THIS (the instigating) file, so without this check it will try to run all code twice(/infinitely)
    try:
        lidarInitArgs = ('COM7',) #tuple of arguments
        processQueue = MP.Queue();   processDataLock = MP.Lock()
        P = MP.Process(target=processFunc, args=(processQueue, processDataLock, lidarInitArgs))
        P.start()
        lidarData = None;  sharedMem = None;  gotLidarData=False #init vars
        while(True):
            if(gotLidarData):
                startTime = time.time()
                if(not processQueue.empty()):
                    gottenThing = processQueue.get()
                    endTime = time.time()
                    if((endTime-startTime)>0.001):
                        print("time:", endTime-startTime)
            else:
                if(not processQueue.empty()):
                    gottenThing = processQueue.get()
                    print("got:", gottenThing)
                    if((((gottenThing[0] == "lidarData") and (len(gottenThing)==2)) if (type(gottenThing[0]) is str) else False) if (type(gottenThing) is tuple) else False):
                        print("got memName:", gottenThing[1])
                        lidarData, sharedMem = sharedMemArray(gottenThing[1])
                        gotLidarData = True
                else:
                    print("waiting for lidarData")
                time.sleep(2)
    finally:
        # try:
        #     sharedMem.close()
        # except:
        #     print("DIDNT CLOSE 'sharedMem'")
        try:
            P.join()
        except:
            print("couldn't join P")