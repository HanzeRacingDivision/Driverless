from Map import Map
#import map_loader as ML #not needed(?)
import coneConnecting as CC #used for UI cone placement
import pathPlanningTemp as PP #used for rendering splines
import drawDriverless as DD
import mapRecvSock    as MS

import time

import threading as thr

import sys #used to pass arguments to the program when started from command line

class pygamesimRemote(Map, CC.coneConnecter, PP.pathPlanner, MS.mapReceiverSocket, DD.pygameDrawer):
    def __init__(self, host, port, window, drawSize=(700,350), drawOffset=(0,0), carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        Map.__init__(self)
        CC.coneConnecter.__init__(self)
        PP.pathPlanner.__init__(self)
        MS.mapReceiverSocket.__init__(self, host, port, self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = False #note: in remote mode, this has no effect (in the current version at least)
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        self.isRemote = True #tell the drawing class to apply UI elements remotely
        self.remoteUIsender = self #mapReceiverSocket class is part of 'self'
        
        #self.carPolygonMode = True #if you dont want to use the car sprite, set this to true (but if the sprite wasnt loaded this will be used automatically)
        
        self.threadKeepRunning = [True]
        try:
            self.mapSockThread = thr.Thread(target=self.runOnThread, name="mapSockThread", args=(self.threadKeepRunning, ), daemon=True)
            self.mapSockThread.start()
        except KeyboardInterrupt:
            print("pygamesimRemote init thread keyboard interrupt")
            try:
                self.threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
                self.mapSockThread.join(2)
                print("thread still alive?:", self.mapSockThread.is_alive())
            except Exception as excep:
                print("couldn't stop thread?:", excep)
        except Exception as excep:
            print("pygamesimRemote init thread  exception:", excep)
            try:
                self.threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
                self.mapSockThread.join(2)
                print("thread still alive?:", self.mapSockThread.is_alive())
            except Exception as excep:
                print("couldn't stop thread?:", excep)


resolution = [1200, 600]
host = 'pi4thijs.local'
port = 65432
if(len(sys.argv)>1):
    for i, arg in enumerate(sys.argv[1::]):
        if(i==0):
            print("setting host to:", arg)
            host = arg
        elif(i==1):
            print("setting port to: int(" + arg + ")")
            port = int(arg)
        elif(i==2):
            if(arg.startswith(("[","(")) and arg.endswith(("]",")"))):
                print("setting resolution to:", arg)
                res = arg.strip("([])").split(",")
                if(len(res) == 2):
                    resolution = [int(res[0]), int(res[1])]
                else:
                    print('failed to set resolution, as "'+arg+'" did not split nicely:', res)

DD.pygameInit(resolution)
sim1 = pygamesimRemote(host, port, DD.window, resolution)

try:
    timeSinceLastUpdate = time.time()
    while DD.windowKeepRunning:
        rightNow = time.time()
        DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        
        if(DD.windowKeepRunning): #the pygame.QUIT event may have set this to False at this point
            DD.windowKeepRunning = sim1.threadKeepRunning[0] #stop window if thread stopped
        
        if(sim1.pathPlanningPresent): #recalculate splines every frame, because why not (this one doesnt need to be all that efficient)
            sim1.makeBoundrySplines()
            sim1.makePathSpline()
        
        sim1.redraw()
        DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        rightNow = time.time() #this is only for the framerate limiter (time.sleep() doesn't accept negative numbers, this solves that)
        if((rightNow-timeSinceLastUpdate) < 0.015): #60FPS limiter
            time.sleep(0.0155-(rightNow-timeSinceLastUpdate))

# except KeyboardInterrupt:
#     print("main thread keyboard interrupt")
# except Exception as excep:
#     print("main thread exception:", excep, excep.args)
finally:
    try:
        sim1.threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
        sim1.mapSockThread.join(3)
        print("thread still alive?:", sim1.mapSockThread.is_alive())
    except Exception as excep:
        print("couldn't stop thread?:", excep)
    DD.pygameEnd() #correctly shut down pygame window
