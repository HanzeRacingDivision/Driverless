from Map import Map
import map_loader as ML
import coneConnecting as CC
import pathFinding    as PF
import pathPlanningTemp as PP
import simulatedCar   as SC
import carMCUclass    as RC
import drawDriverless as DD
import mapTransSock   as MS

import time
#import numpy as np

import multiprocessing as MP

class pygamesimLocal(Map, ML.mapLoader, CC.coneConnecter, PF.pathFinder, PP.pathPlanner, DD.pygameDrawer):
    def __init__(self, window, drawSize=(700,350), drawOffset=(0,0), carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        Map.__init__(self) #init map class
        ML.mapLoader.__init__(self)
        
        #self.clockSet(SC.simClockExample) #an altered clock, only for simulations where the speed is faster/slower than normal  #DEPRICATED
        #self.clock = your custom clock function here
        self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
        #self.car = RC.realCar(self.clock, comPort='COM8')
        
        CC.coneConnecter.__init__(self)
        PF.pathFinder.__init__(self)
        PP.pathPlanner.__init__(self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        self.isRemote = False #tell the drawing class to apply UI elements locally
        
        #self.carPolygonMode = True #if you dont want to use the car sprite, set this to true (but if the sprite wasnt loaded this will be used automatically)
        
        if(self.pathPlanningPresent):
            self.car.pathFolData = PP.pathPlannerData()
        
        #self.mapList = [copyExtractMap(self)]