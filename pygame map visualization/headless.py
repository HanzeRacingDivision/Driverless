from Map import Map
import map_loader as ML
import coneConnecting as CC
import pathFinding    as PF
import pathPlanningTemp as PP
import simulatedCar   as SC
import carMCUclass    as RC
import mapTransSock   as MS

import time
#import numpy as np

import threading as thr

import sys #used for importing files (map_loader) from commandline (DOS run argument)


class pygamesimHeadless(Map, ML.mapLoader, CC.coneConnecter, PF.pathFinder, PP.pathPlanner):
    def __init__(self):
        Map.__init__(self) #init map class
        ML.mapLoader.__init__(self)
        
        #self.clockSet(SC.simClockExample) #an altered clock, only for simulations where the speed is faster/slower than normal  #DEPRICATED
        #self.clock = your custom clock function here
        self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
        #self.car = RC.realCar(self.clock, comPort='COM8')
        
        CC.coneConnecter.__init__(self)
        PF.pathFinder.__init__(self)
        PP.pathPlanner.__init__(self)
        #initialize constants (about which parts are present)
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerData()



sim1 = pygamesimHeadless()

if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
    print("found sys.argv[1], attempting to import:", sys.argv[1])
    sim1.load_map(sys.argv[1], sim1)

timeSinceLastUpdate = sim1.clock()
print("printing serial ports:")
[print(entry.name) for entry in RC.serial.tools.list_ports.comports()]
print("done printing ports.")
print()

try:
    mapSender = MS.mapTransmitterSocket(host='', port=65432, objectWithMap=sim1)
    mapSender.mapSendInterval = 0.2 #start safe, you can bring this number down if the connection is good (FPS = 1/this)
    threadKeepRunning = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    autoMapSend = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    UIreceive = [True] #an argument (functional pointer) shared between the main and mapSockThread and main thread
    mapSockThread = thr.Thread(target=mapSender.runOnThread, name="mapSockThread", args=(threadKeepRunning, autoMapSend, UIreceive), daemon=True)
    mapSockThread.start()
    ##note: mapSender.manualSendBuffer is a list to which you can append things (any object) to be sent to the connected client (if any)
    
    while threadKeepRunning[0]: #stop evrything if mapSockThread stops
        rightNow = sim1.clock()
        dt = rightNow - timeSinceLastUpdate
        
        if((sim1.car.pathFolData.auto) if (sim1.pathPlanningPresent and (sim1.car.pathFolData is not None)) else False):
            sim1.calcAutoDriving()
            # if(sim1.car.pathFolData.laps >= 1): #stop after a set number of laps
            #     sim1.car.pathFolData.targetVelocity = 0.0;  sim1.car.desired_velocity = 0.0;  #sim1.car.pathFolData.auto = False
            #     sim1.car.pathFolData.laps = 0 #reset counter, to allow the car to start driving again (by upping targetVelocity)
            sim1.car.sendSpeedAngle(sim1.car.desired_velocity, sim1.car.desired_steering) #(spam) send instruction (or simulate doing so)
        sim1.car.getFeedback() #run this to parse serial data (or simulate doing so)
        sim1.car.update(dt)
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        
        ## after all the important stuff:
        FPSrightNow = sim1.clock() #this is only for the framerate limiter (time.sleep() doesn't accept negative numbers, this solves that)
        if((FPSrightNow-rightNow) < 0.015): #60FPS limiter (optional)
            time.sleep(0.0155-(FPSrightNow-rightNow))

# except KeyboardInterrupt:
#     print("main thread keyboard interrupt")
# except Exception as excep:
#     print("main thread exception:", excep)
finally:
    try:
        mapSender.manualClose()
    except:
        print("manualClose() failed")
    try:
        threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
        mapSockThread.join(2)
        print("mapSockThread still alive?:", mapSockThread.is_alive())
    except Exception as excep:
        print("couldn't stop thread?:", excep)
    try:  #alternatively:  if(type(sim1.car) is RC.realCar):
        sim1.car.disconnect()
    except Exception as excep:
        print("failed to run car.disconnect():", excep)