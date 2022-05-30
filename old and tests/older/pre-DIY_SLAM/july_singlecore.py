
## this version changes the structure to split the (pickle-able) Map object and the functions that interact with it
## drawDriverless.py is split into a drawing file (drawDriverless.py) and a UI file, pygameUI.py
## map_loader, coneConnecting, pathFinding and (now) pathPlanningTemp were static classes (no variables), so these are now used as such (no longer integrated into map object)
##  note: i think you could also pull some functions out of the base Map class (make them global), but i'm not sure that actually gets pickled at all
## the 3D (camera FPV) rendering class is also added to drawDriverless.py



from Map import Map

import map_loader as ML
# import coneConnecting as CC
# import pathFinding    as PF
import pathPlanningTemp as PP

import simulatedCar as SC

import drawDriverless as DD
import pygameUI       as UI

#import mapTransSock   as MS

#import time
#import numpy as np
import sys

# import multiprocessing as MP
# def drawProcessTarget(sharedMemName, sharedMemLock):
#     do MP stuff...

class mapObjectClass(Map):
    def __init__(self):
        Map.__init__(self) #init map class
        
        self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
        #enable/disable components here
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerCarData()
            self.pathFolData = PP.pathPlannerMapData()

resolution = [1280, 720]

if __name__ == "__main__":
    try:
        mapObject = mapObjectClass()
        DD.pygameInit(resolution)
        drawer = DD.pygameDrawer(mapObject, DD.window, resolution)
        drawer3D = DD.pygameDrawer3D(mapObject, DD.window, resolution)
        #coneConnec = CC.coneConnecter(mapObject)
        #pathFind = PF.pathFinder(mapObject)
        ##mapSender = MS.mapTransmitterSocket(host='', port=65432, objectWithMap=mapObject)
        
        if(sys.argv[1].endswith(ML.mapLoader.fileExt) if ((type(sys.argv[1]) is str) if (len(sys.argv) > 1) else False) else False): #a long and convoluted way of checking if a file was (correctly) specified
            print("found sys.argv[1], attempting to import:", sys.argv[1])
            ML.mapLoader.load_map(sys.argv[1], mapObject)
        
        timeSinceLastUpdate = mapObject.clock()
        while DD.windowKeepRunning:
            rightNow = mapObject.clock()
            UI.handleAllWindowEvents(drawer) #handle all window events like key/mouse presses, quitting and most other things
            
            if((mapObject.car.pathFolData.auto) if (mapObject.pathPlanningPresent and (mapObject.car.pathFolData is not None)) else False):
                PP.calcAutoDriving(mapObject)
            mapObject.car.simulateFeedback()
            mapObject.car.update(rightNow - timeSinceLastUpdate) #to be replaced by SLAM?
            
            if(drawer.extraViewMode):
                drawer3D.redraw()
            else:
                drawer.redraw()
            DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
            
            timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
    finally:
        try:
            DD.pygameEnd() #correctly shut down pygame window
        except:
            print("couldn't run pygameEnd()")
        # try:
        #     sharedMem.close()
        #     sharedMem.unlink()
        # except:
        #     print("couldn't close/unlink sharedMem")
        # try:
        #     drawProcess.join()
        # except:
        #     print("couldn't join lidarProcess")