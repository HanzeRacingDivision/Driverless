from mapClassTemp import Map
import coneConnecting as CC
import pathFinding as    PF
import drawDriverless as DD
import time

class pygamesimLocal(CC.coneConnecter, CC.pathFinder, DD.pygameDrawer):
    def __init__(self, window, drawSize=(1200,600), drawOffset=(0,0), viewOffset=[0,0], carCamOrient=0, sizeScale=30, startWithCarCam=False, invertYaxis=True, importConeLogFilename='', logging=True, logname="coneLog"):
        Map.__init__(self) #init map class
        CC.coneConnecter.__init__(self, importConeLogFilename, logging, logname)
        PF.pathFinder.__init__(self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, viewOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = True
        self.pathPlanningPresent = False
        self.SLAMPresent = False



DD.pygameInit()
sim1 = pygamesimLocal(DD.window) #just a basic class object with all default attributes

timeSinceLastUpdate = time.time()

while DD.windowKeepRunning:
    DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
    
    rightNow = time.time()
    sim1.car.update(rightNow - timeSinceLastUpdate)
    timeSinceLastUpdate = rightNow
    
    sim1.redraw()
    DD.frameRefresh() #not done in redraw() to accomodate multi-sim options

DD.pygameEnd()