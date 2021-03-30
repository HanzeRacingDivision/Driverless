from Map import Map
import coneConnecting as CC
import pathFinding    as PF
import drawDriverless as DD
import pathPlanningTemp as PP
import simulatedCar   as SC
import carMCUclass    as RC

import time
import numpy as np

from copy import deepcopy

def copyExtractMap(classWithMapParent): #copy ONLY the map class attributes from any (child) class into a new map object
    returnObject = Map() #make new instance of same class as source
    for attrName in dir(returnObject): #dir(class) returs a list of all class attributes
            if((not attrName.startswith('_')) and (not callable(getattr(returnObject, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
                setattr(returnObject, attrName, deepcopy(getattr(classWithMapParent, attrName))) #copy attribute
    return(returnObject)

class pygamesimLocal(CC.coneConnecter, PF.pathFinder, PP.pathPlanner, DD.pygameDrawer):
    def __init__(self, window, drawSize=(1200,600), drawOffset=(0,0), viewOffset=[0,0], carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True, importConeLogFilename='', logging=True, logname="coneLog"):
        Map.__init__(self) #init map class
        self.car = SC.simCar() #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        #self.car = RC.realCar(comPort='COM8')
        CC.coneConnecter.__init__(self, importConeLogFilename, logging, logname)
        PF.pathFinder.__init__(self)
        PP.pathPlanner.__init__(self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, viewOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        #self.carPolygonMode = False #use the fancy car sprite (default)
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerData()
        
        #self.mapList = [copyExtractMap(self)]



DD.pygameInit()
sim1 = pygamesimLocal(DD.window) #just a basic class object with all default attributes

timeSinceLastUpdate = time.time()
#mapSaveTimer = time.time()
print("printing serial ports:")
[print(entry.name) for entry in RC.serial.tools.list_ports.comports()]
print("done printing ports.")
print()


while DD.windowKeepRunning:
    rightNow = time.time()
    dt = rightNow - timeSinceLastUpdate
    DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
    
    if((sim1.car.pathFolData.auto) if sim1.pathPlanningPresent else False):
        sim1.calcAutoDriving()
        sim1.car.sendSpeedAngle(sim1.car.desired_velocity, sim1.car.desired_steering) #(spam) send instruction (or simulate doing so)
        sim1.car.getFeedback() #run this to parse serial data (or simulate doing so)
        #print(sim1.car.velocity)
    sim1.car.update(dt)
    
    sim1.redraw()
    DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
    
    # if((time.time()-mapSaveTimer)>0.25):
    #     mapSaveTimer = time.time()
    #     sim1.mapList.append(copyExtractMap(sim1))
    #     if(len(sim1.mapList) > 40):
    #         sim1.mapList.pop(0)
    #     print((time.time()-mapSaveTimer)*1000)
    
    timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
    
    rightNow = time.time() #this is only for the framerate limiter (time.sleep() doesn't accept negative numbers, this solves that)
    if((rightNow-timeSinceLastUpdate) < 0.015): #60FPS limiter
        time.sleep(0.016-(rightNow-timeSinceLastUpdate))

DD.pygameEnd()
try:  #alternatively:  if(type(sim1.car) is RC.realCar):
    sim1.car.disconnect()
except Exception as excep:
    print("failed to run car.disconnect():", excep)
