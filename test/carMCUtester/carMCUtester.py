
import time
import numpy as np

from Map import Map
# import GF.generalFunctions as GF

import carMCUclass  as RC

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)


if __name__ == "__main__":
    try:
        masterMap = Map()
        masterMap.car = RC.realCar()
        
        connToSlaves, connToMaster = MP.Pipe(False) #disable duplex (for now)
        masterSharedMem = SM.makeNewSharedMem('driverless', 10*1000*1000) #note: max (pickled) object size is HALF of this size, due to the double-buffering
        
        connToCar, connToCarMaster = MP.Pipe(True) #a two-way pipe to the carMCUserialProcess (which (encodes/decodes and) passes it on to/from the serial connection)
        carSerial = RC.carMCUserialProcess(connToCarMaster, masterSharedMem.passToProcess, comPort="COM4", autoFind=False, autoreconnect=True)
        carSerial.start()
        
        while(True):
            loopStart = masterMap.clock()
            masterMap.car.desired_steering = np.sin((loopStart * 0.25) * 2*np.pi) * Map.Car.maxSteeringAngle
            connToCar.send((masterMap.car.desired_velocity, masterMap.car.desired_steering))
            if(connToCar.poll()):
                carData = None #init var
                while(connToCar.poll()):
                    carData = connToCar.recv()
                    #you can update for every feedback, or you can update for only the newest 
                print("received from car:  t:", carData[0], "steer:", np.rad2deg(carData[1]), "trvld:", carData[2])
                timestamp, steeringAngle, newTotalDist = carData #(unpack tuple)
                dTime = timestamp - masterMap.car.lastFeedbackTimestamp
                dDist = newTotalDist - masterMap.car.totalDistTraveled
                masterMap.car.lastFeedbackTimestamp = timestamp
                masterMap.car.totalDistTraveled = newTotalDist
                masterMap.car.steering = steeringAngle
                ## note: SLAM should replace most of this stuff, so the lack of real filetering here is temporary
                if(dTime <= 0.0001):
                    print("bad timestamp?!", dTime)
                else:
                    #masterMap.car.velocity = (masterMap.car.velocity + (dDist/dTime))/2 #i don't trust the (low-resolution) rotary encoder on the car enough to get the right velocity without some averaging)
                    masterMap.car.velocity = dDist/dTime
                    masterMap.car.update(dTime, dDist) #to be replaced by SLAM?
            
            pickleSize = masterSharedMem.pickle(masterMap)
            
            loopEnd = masterMap.clock() #this is only for the 'framerate' limiter (time.sleep() doesn't accept negative numbers, this solves that)
            if((loopEnd-loopStart) < 0.015): #60FPS limiter (optional)
                time.sleep(0.0155-(loopEnd-loopStart))
            elif((loopEnd-loopStart) > 0.2):
                print("main process running slow", 1/(loopEnd-loopStart))
    finally:
        try:
            connToCar.close()
            connToCarMaster.close()
            print("(main) carSerial pipe closing done")
        except: 
            print("couldn't close carSerial pipes")
        try:
            carSerial.stopLoop()
            print("(main) carSerial stopping done")
        except:
            try:
                if(carSerial.is_alive()):
                    carSerial.terminate()
                    carSerial.join(3)
                    print("(main) carSerial stopping maybe done")
            finally:
                if(carSerial.is_alive()):
                    print("couldn't stopLoop carSerial")