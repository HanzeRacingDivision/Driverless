from Map import Map

# import map_loader as ML  #note: imported in __main__
# import coneConnecting as CC
# import pathFinding    as PF
# import pathPlanningTemp as PP

# import simulatedCar as SC
#import carMCUclass  as RC

# import drawDriverless as DD
# import pygameUI       as UI

#import mapTransSock   as MS

#import phantom        as PH

import time
#import numpy as np
import sys #used for cmdline arguments

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)

class masterMapClass(Map): #for pickle to make the same objects in different sketches, 
    pass


class drawProcess(MP.Process):
    """a multiprocessing pygameDrawer"""
    def __init__(self, connToMaster, sharedMemInit, resolution, asynchronous=True):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.sharedMemInit = sharedMemInit
        self.resolution = resolution
        self.asynchronous = asynchronous
        
        self.keepalive = MP.Semaphore() # this (functional) boolean stops the while() loop in run()
    
    def run(self):
        #self.keepalive.release() #for running this process again after it was ended
        try:
            import drawDriverless as DD
            import pygameUI       as UI
            import phantom        as PH
            slaveSharedMem = SM.sharedMemReadWrapper(*self.sharedMemInit)
            initMap = slaveSharedMem.readObj(self.keepalive) #wait for the first map object to come in (with no timeout?)
            DD.pygameInit(self.resolution)
            drawer = DD.pygameDrawer(initMap, DD.window, self.resolution)
            drawer.isRemote = True
            drawer3D = DD.pygameDrawer3D(initMap, DD.window, self.resolution)
            while((self.keepalive.get_value()) and DD.windowKeepRunning):
                loopStart = time.time()
                try:
                    if(self.asynchronous):
                        if(slaveSharedMem.poll()): #if a new map is available
                            newMap = slaveSharedMem.readObj(self.keepalive) #get master map
                            drawer.mapToDraw = newMap
                            drawer3D.mapToDraw = newMap
                    else:
                        newMap = slaveSharedMem.readNewObj(self.keepalive) #get master map (blocking)
                        drawer.mapToDraw = newMap
                        drawer3D.mapToDraw = newMap
                except Exception as excep:
                    print("unpickling exception?:", excep, excep.args)
                
                if(drawer.extraViewMode):
                    drawer3D.redraw()
                else:
                    drawer.redraw()
                DD.frameRefresh()
                
                phantomMap = PH.phantomClass()
                UI.handleAllWindowEvents(drawer, phantomMap) #handle all window events like key/mouse presses, quitting and most other things
                
                if(phantomMap._hasData_):
                    self.connToMaster.send(phantomMap) #if there were any UI changes
                    if(self.asynchronous):
                        slaveSharedMem.lastFrameCounter = slaveSharedMem.frameCounter.value #assume you have the latest map object
                        drawer.mapToDraw = slaveSharedMem.readNewObj(self.keepalive) #block untill a new master map is received
                        drawer3D.mapToDraw = drawer.mapToDraw
                
                loopEnd = time.time()
                if((loopEnd-loopStart)>0.5):
                    print("drawer running slow", 1/(loopEnd-loopStart))
        finally:
            print("drawProcess ending")
            try:
                DD.pygameEnd() #correctly shut down pygame window
            except:
                print("couldn't run pygameEnd()")
    
    def stopLoop(self, alsoJoin=True, joinTimeout=5):
        if(self.keepalive.get_value()):
            self.keepalive.acquire()
        if(alsoJoin and self.is_alive()):
            self.join(joinTimeout)
            if(self.is_alive()):
                print("stopLoop join failed, process still alive!")
            else:
                self.keepalive.release() #setup for next start (not needed)


if __name__ == "__main__":
    try:
        resolution = [1280, 720]
        host = '192.168.137.193'
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
        
        ## mapRecvSock init
        import mapRecvSock    as MS
        import socket
        import pickle
        mapReceiver = MS.mapReceiverSocket(host, port)
        phantomMapBytes = None
        keepReceiving = True
        
        ## drawer init
        connToSlaves, connToMaster = MP.Pipe(False) #disable duplex (for now)
        masterSharedMem = SM.makeNewSharedMem('remoteViewer', 10*1000*1000) #note: max (pickled) object size is HALF of this size, due to the double-buffering
        drawer = drawProcess(connToMaster, masterSharedMem.passToProcess, resolution, False)
        drawer.start()
        
        while((drawer.is_alive()) and keepReceiving):
            try:
                print("connecting to server...")
                mapReceiver.mapSock.connect((mapReceiver.host, mapReceiver.port))
                with mapReceiver.mapSock: #python likes this(?)
                    print("connected to:", mapReceiver.mapSock.getpeername())
                    # firstPacket = mapReceiver.mapSock.recv(999999) #receive any straddler-data
                    # print("first packet received:", len(firstPacket))
                    failureCounter = 0
                    failureCountDecrementTimer = time.time()
                    lastMapImportTime = time.time()
                    while((drawer.is_alive()) and keepReceiving):     ################## main loop
                        loopStart = time.time()
                        receivedBytes = mapReceiver.manualReceiveBytes() #get map data (blocking)
                        #print("receivedBytes:", len(receivedBytes))
                        if(len(receivedBytes) > 0):
                            masterSharedMem.writePickled(receivedBytes)
                        else:
                            failureCounter += 1
                        
                        if(failureCounter > mapReceiver.maxFailureCount):
                            print("too many faillures")
                            raise(Exception("faillureCounter"))
                            # keepReceiving = False
                            # #mapReceiver.initSuccess = False
                            # return()
                        if((time.time() - failureCountDecrementTimer) > mapReceiver.failureCountDecrementInterval):
                            failureCountDecrementTimer = time.time()
                            if(failureCounter > 0):
                                print("decrementing failureCounter:", failureCounter)
                                failureCounter -= 1
                            #failureCounter = max(failureCounter-1, 0)
                        
                        if(connToSlaves.poll()): #check drawProcess
                            phantomMapBytes = connToSlaves.recv_bytes() #get phantomMap object from drawProcess (get it as pickled bytes, to avoid needless re-pickling)
                            ## send phantomMap to host
                        if(phantomMapBytes is not None):
                            #print("sending", len(phantomMapBytes), "bytes from phantomMapBytes")
                            if(len(phantomMapBytes) > 999999):
                                print("phantomMapBytes too large:", len(phantomMapBytes))
                            else:
                                packetSizeHeader = str(len(phantomMapBytes)).rjust(mapReceiver.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                                mapReceiver.mapSock.sendall(packetSizeHeader + phantomMapBytes)
                            phantomMapBytes = None
                        
                        loopEnd = time.time()
                        if((loopEnd-loopStart) > 0.5):
                            print("main process running slow", 1/(loopEnd-loopStart))
            except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                errorResolved = False
                if(excep.args[0] == 10038): #'an operation was performed by something that is not a socket'
                    print("mapReceiverSocket.runOnThread(): mapSock is not socket?, attempting reInit...")
                    if(mapReceiver.reInit()):
                        print("mapSock not socket -> reInit success, continuing...")
                        errorResolved = True
                if(not errorResolved):
                    print("mapReceiverSocket.runOnThread(): major socket exception:", excep)
                    try:
                        mapReceiver.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    keepReceiving = False
                    mapReceiver.initSuccess = False
            except KeyboardInterrupt:
                print("mapReceiverSocket.runOnThread(): keyboardInterrupt! closing everything")
                try:
                    mapReceiver.mapSock.close()
                except Exception as excep:
                    print("couldn't close mapSock after keyboardInterrupt:", excep)
                keepReceiving = False
                mapReceiver.initSuccess = False
            except Exception as excep:
                errorResolved = False
                if((excep.args[0] == "faillureCounter") if (type(excep.args[0]) is str) else False):
                    print("simple failureCount overflow, ignoring exception (reconnecting)...")
                    errorResolved = True
                if(not errorResolved):
                    print("mapReceiverSocket.runOnThread(): other exception:", excep)
                    try:
                        mapReceiver.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    keepReceiving = False
                    mapReceiver.initSuccess = False
            
    finally:
        print("main ending")
        try:
            connToSlaves.close()
            connToMaster.close()
        except: 
            print("couldn't close map pipes")
        try:
            drawer.stopLoop()
        except:
            try:
                drawer.join(3)
            finally:
                print("couldn't stopLoop drawer")
    print("main ended")

