from Map import Map
import coneConnecting as CC
import pathFinding    as PF
import pathPlanningTemp as PP
import simulatedCar   as SC
import drawDriverless as DD


# import time
# def simClock(clockStart):
#     return((time.time()-clockStart)*4)
global discreteClock, discreteClockStep
discreteClock = 0
discreteClockStep = 0.05
def simClock(clockStart):
    return(discreteClock)


class pygamesimLocal(CC.coneConnecter, PF.pathFinder, PP.pathPlanner, DD.pygameDrawer):
    def __init__(self, window, drawSize=(700,350), drawOffset=(0,0), carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        Map.__init__(self) #init map class
        
        self.clockSet(simClock) #an altered clock, only for simulations where the speed is faster/slower than normal
        self.car = SC.simCar(self.clock) #simCar has Map.Car as a parent class, so all regular Car stuff will still work
        
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

resolution = [1200, 600]

DD.pygameInit(resolution)
sim1 = pygamesimLocal(DD.window, resolution)

timeSinceLastUpdate = sim1.clock()

try:
    while DD.windowKeepRunning:
        discreteClock += discreteClockStep
        
        rightNow = sim1.clock()
        dt = rightNow - timeSinceLastUpdate
        DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        
        if((sim1.car.pathFolData.auto) if (sim1.pathPlanningPresent and (sim1.car.pathFolData is not None)) else False):
            sim1.calcAutoDriving()
            sim1.car.sendSpeedAngle(sim1.car.desired_velocity, sim1.car.desired_steering) #(spam) send instruction (or simulate doing so)
        sim1.car.getFeedback() #run this to parse serial data (or simulate doing so)
        sim1.car.update(dt)
        
        sim1.redraw()
        DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        
except KeyboardInterrupt:
    print("main thread keyboard interrupt")
except Exception as excep:
    print("main thread exception:", excep)
finally:
    DD.pygameEnd() #correctly shut down pygame window
