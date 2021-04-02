from Map import Map
import coneConnecting as CC #used for UI cone placement
import pathPlanningTemp as PP #used for rendering splines
import drawDriverless as DD
import mapRecvSock    as MS

import time

import threading as thr

class pygamesimRemote(CC.coneConnecter, PP.pathPlanner, MS.mapReceiverSocket, DD.pygameDrawer):
    def __init__(self, host, port, window, drawSize=(700,350), drawOffset=(0,0), viewOffset=[0,0], carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        Map.__init__(self)
        CC.coneConnecter.__init__(self)
        PP.pathPlanner.__init__(self)
        MS.mapReceiverSocket.__init__(self, host, port, self)
        DD.pygameDrawer.__init__(self, self, window, drawSize, drawOffset, viewOffset, carCamOrient, sizeScale, startWithCarCam, invertYaxis)
        #tell the drawing class which parts are present
        self.coneConnecterPresent = True
        self.pathFinderPresent = False #note: in remote mode, this has no effect (in the current version at least)
        self.pathPlanningPresent = True
        self.SLAMPresent = False
        
        self.isRemote = True
        self.remoteUIsender = self #mapReceiverSocket class is part of 'self'
        
        #self.carPolygonMode = True #if you dont want to use the car sprite, set this to true (but if the sprite wasnt loaded this will be used automatically)
        
        #self.mapList = [copyExtractMap(self)]
        
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

DD.pygameInit(resolution)
sim1 = pygamesimRemote('127.0.0.1', 65432, DD.window, resolution)

try:
    #sim1.manualConnect()
    
    timeSinceLastUpdate = time.time()
    while DD.windowKeepRunning:
        rightNow = time.time()
        DD.handleAllWindowEvents(sim1) #handle all window events like key/mouse presses, quitting and most other things
        
        #sim1.manualReceive(True) #this way, framerate is limited to sending rate
        if(DD.windowKeepRunning): #the pygame.QUIT event may have set this to False at this point
            DD.windowKeepRunning = sim1.threadKeepRunning[0] #stop window if thread stopped
        
        if(sim1.pathPlanningPresent): #recalculate splines
            sim1.makeBoundrySplines()
            sim1.makePathSpline()
        
        sim1.redraw()
        DD.frameRefresh() #not done in redraw() to accomodate multi-sim options
        
        # if((rightNow-mapSaveTimer)>0.25):
        #     sim1.mapList.append(copyExtractMap(sim1))
        #     if(len(sim1.mapList) > 40):
        #         sim1.mapList.pop(0)
        #     #print((time.time()-mapSaveTimer)*1000)
        #     mapSaveTimer = rightNow
        
        timeSinceLastUpdate = rightNow #save time (from start of loop) to be used next time
        rightNow = time.time() #this is only for the framerate limiter (time.sleep() doesn't accept negative numbers, this solves that)
        if((rightNow-timeSinceLastUpdate) < 0.015): #60FPS limiter
            time.sleep(0.0155-(rightNow-timeSinceLastUpdate))

except KeyboardInterrupt:
    print("main thread keyboard interrupt")
except Exception as excep:
    print("main thread exception:", excep, excep.args)
finally:
    try:
        sim1.threadKeepRunning[0] = False #signal the Thread function to stop its while() loop(s) (the list is just a manual boolean pointer (hack))
        sim1.mapSockThread.join(3)
        print("thread still alive?:", sim1.mapSockThread.is_alive())
    except Exception as excep:
        print("couldn't stop thread?:", excep)
    try:
        sim1.manualClose()
    except:
        print("couldn't manualClose()")
    DD.pygameEnd() #correctly shut down pygame window
