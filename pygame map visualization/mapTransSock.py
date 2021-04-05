##this sketch has functions for sending Map objects to a remote PC to be drawn
##this will be the network server, and the drawing PC will be the client.
##you could do it the other way around, but this allows for constant settings on this side, and variable settings on the drawing side, which makes sense to me (Thijs)

## TBD: different (default) ports for maps with different classes present (coneConnecting, pathPlanning, SLAM, etc) ?

import socket
import pickle
import time
import threading as thr

import generalFunctions as GF #only used for cone deletion (UI parser)

## this is used to shrink the sent object to only include (important) map objects, no drawing data
from Map import Map #only used in copyExtractMap
from copy import deepcopy #only used in copyExtractMap
def deepCopyExtractMap(classWithMapParent): #copy ONLY the map class attributes from any (child) class into a new map object
    returnObject = Map() #make new instance of same class as source
    for attrName in dir(returnObject): #dir(class) returs a list of all class attributes
            if((not attrName.startswith('_')) and (not callable(getattr(returnObject, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
                setattr(returnObject, attrName, deepcopy(getattr(classWithMapParent, attrName))) #deepcopy attribute
    return(returnObject)

def shallowCopyExtractMap(classWithMapParent): #copy ONLY the map class attributes from any (child) class into a 'new' map object (consisting of pointers)
    returnObject = Map() #make new instance of same class as source
    for attrName in dir(returnObject): #dir(class) returs a list of all class attributes
            if((not attrName.startswith('_')) and (not callable(getattr(returnObject, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
                setattr(returnObject, attrName, getattr(classWithMapParent, attrName)) #copy attribute (pointer)
    return(returnObject)


class mapTransmitterSocket:
    def __init__(self, host='', port=69420, objectWithMap=None, usePacketSizeHeader=True):
        """a class for transmitting Map objects over the network (IPV4 TCP steam) to nearby PCs to draw them on screen (to lighten processing load on car PC)"""
        self.mapSock = None
        self.initSuccess = False
        self.usePacketSizeHeader = usePacketSizeHeader #send a (constant, predetermined length) before each sent object
        self.packetSizeHeaderLength = 6 #constant, must be same on transmitter and receiver
        
        self.host = host
        self.port = port
        
        self.runningOnThread = None
        self.manualSendBuffer = [] #used to pass objects to runOnThread() (to be sent to the client)  note: does not need to be here, could also just be an argument in runOnThread(), but whatever
        
        self.maxPacketFixAttempts = 10 #only for UIreceiver
        
        self.mapSendInterval = 0.25 #avoids spamming
        #self.manualSendInterval = 0.05 #avoids spamming (can be set to 0 if you are very confident) (TBD?)
        self.objectWithMap = objectWithMap
        if(self.objectWithMap is None):
            print("asuming map is in same class as mapTransmitterSocket (only if auto-sending)")
            self.objectWithMap = self
        
        try:
            self.mapSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.mapSock.bind((host, port))
            self.mapSock.listen()
            self.initSuccess = True #if everything went well, set to true
        except KeyboardInterrupt: #only in case the tried code hangs forever
            print("map transmission socket init keyboardInterrupt")
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        except Exception as excep:
            print("map transmission socket init exception:", excep)
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
    
    def __del__(self):
        print("deinitializing mapTransmitterSocket")
        if(self.runningOnThread is not None):
            print("THREAD STILL RUNNING!!!")
        try:
            self.mapSock.close()
        except:
            print("couldn't close() mapSock")
    
    def manualClose(self):
        """close the socket (MUST be done when exiting the program/deinitializing the object)"""
        try:
            self.mapSock.close()
            print("manualClose() success")
        except Exception as closeExcep:
            print("couldn't close mapSock after other exception:", closeExcep)
        self.initSuccess = False
    
    def reInit(self, host=None, port=None):
        self.initSuccess = False
        #if(self.runningOnThread is not None): #note: reInit() may have been called from inside the thread, in an attempt to stay alive
        try:
            self.mapSock.close() #close existing/previous socket
        except:
            print("couldn't close() mapSock")
        time.sleep(0.05) #wait just a little bit (50ms) for extra safety
        
        if(host is not None):
            self.host = host
        if(port is not None):
            self.port = port
        try: #try to re-establish socket
            self.mapSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.mapSock.bind((self.host, self.port))
            self.mapSock.listen()
            self.initSuccess = True #if everything went well, set to true
        except KeyboardInterrupt: #only in case the tried code hangs forever
            print("map transmission socket reInit keyboardInterrupt")
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        except Exception as excep:
            print("map transmission socket reInit exception:", excep)
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        finally:
            return(self.initSuccess) #no need to return, if you just check self.initSuccess afterwards, but why not right
    
    def UIparser(self, instruction):
        """parse the received instruction (like place-cone or start-auto)
            this fulfills the funcion of drawDriverless, but with less(?) CPU/GPU drag"""
        if(self.objectWithMap is None):
            print("(UIreceiver) objectWithMap is None, can't parse UI")
            return(False)
        print("parsing:", instruction)
        if(instruction[0] == 'PLACE'):
            if(len(instruction) < 3): #if the 'immediateConnect' argument was not provided (for whatever silly reason...) assume it's False
                instruction.append(False)
            aNewCone = instruction[1]
            overlaps, overlappingCone = self.objectWithMap.overlapConeCheck(aNewCone.position) #check if there isn't already a cone there
            if(overlaps):
                print("can't enact 'PLACE' instruction, it overlaps with existing cone:", overlappingCone.ID)
                return(False)
            coneListToAppend = (self.objectWithMap.right_cone_list if aNewCone.LorR else self.objectWithMap.left_cone_list)
            coneListToAppend.append(aNewCone)
            if(aNewCone.isFinish):
                if((len(self.objectWithMap.finish_line_cones)<2) and ((self.objectWithMap.finish_line_cones[0].LorR != aNewCone.LorR) if (len(self.objectWithMap.finish_line_cones)>0) else True)):
                    self.objectWithMap.finish_line_cones.append(aNewCone)
                else:
                    print("couldn't set instructed cone as finish line cone (but did place it)!")
            if(instruction[2]):
                if(self.objectWithMap.coneConnecterPresent):
                    self.objectWithMap.connectCone(aNewCone)
                else:
                    print("couldn't connect newly instructed cone, because self.objectWithMap.coneConnecterPresent == False")
            if(self.objectWithMap.pathPlanningPresent):
                    self.objectWithMap.makeBoundrySplines()
        elif(instruction[0] == 'CONNEC'):
            if(self.objectWithMap.coneConnecterPresent):
                overlaps, coneToConnect = self.objectWithMap.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
                if(overlaps and (coneToConnect.ID == instruction[1].ID)):
                    self.objectWithMap.connectCone(coneToConnect)
                    if(self.objectWithMap.pathPlanningPresent):
                        self.objectWithMap.makeBoundrySplines()
                else:
                    print("'CONNEC' instruction couldnt find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
                    return(False)
            else:
                print("ignored 'CONNEC' instruction, because self.objectWithMap.coneConnecterPresent == False")
        elif(instruction[0] == 'SETFIN'):
            overlaps, coneToSetFin = self.objectWithMap.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
            if(overlaps and (coneToSetFin.ID == instruction[1].ID)):
                if((len(self.objectWithMap.finish_line_cones)<2) and ((self.objectWithMap.finish_line_cones[0].LorR != coneToSetFin.LorR) if (len(self.objectWithMap.finish_line_cones)>0) else True)):
                    coneToSetFin.isFinish = True
                    self.objectWithMap.finish_line_cones.append(coneToSetFin)
                else:
                    print("couldn't enact 'SETFIN' instruction, too many finish cones OR", ("right" if coneToSetFin.LorR else "left"),"cone finish already exists")
            else:
                print("'SETFIN' instruction couldnt find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
                return(False)
        elif(instruction[0] == 'DELET'):
            overlaps, coneToDelete = self.objectWithMap.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
            if(overlaps and (coneToDelete.ID == instruction[1].ID)):
                for connectedCone in coneToDelete.connections:
                    if(len(connectedCone.coneConData) > 0): #it's always a list, but an empty one if coneConnecter is not used
                        connectedCone.coneConData.pop((0 if (connectedCone.connections[0].ID == coneToDelete.ID) else 1))
                    connectedCone.connections.pop((0 if (connectedCone.connections[0].ID == coneToDelete.ID) else 1))
                listToRemoveFrom = (self.objectWithMap.right_cone_list if coneToDelete.LorR else self.objectWithMap.left_cone_list)
                listToRemoveFrom.pop(GF.findIndexByClassAttr(listToRemoveFrom, 'ID', coneToDelete.ID))
                if(self.objectWithMap.pathPlanningPresent):
                    self.objectWithMap.makeBoundrySplines()
            else:
                print("'DELET' instruction couldn't find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
                return(False)
        elif(instruction[0] == 'PATH'):
            if(self.objectWithMap.pathFinderPresent):
                if(instruction[1] < 1): #if instructed to path-find as far as possible
                    doesNothing = 0 # a variable to keep python happy
                    while(self.objectWithMap.makePath()): #stops when path can no longer be advanced
                        doesNothing += 1  # "python is so versitile, you can do anything" :) haha good joke
                else: #if instructed to (attempt to) find a certian number of path points
                    for i in range(instruction[1]): #try to make this many path points
                        self.objectWithMap.makePath()
                self.objectWithMap.makePathSpline()
            else:
                print("ignored 'PATH' instruction, because self.objectWithMap.pathFinderPresent == False")
        elif(instruction[0] == 'AUTO'):
            if(len(instruction) != 3):
                print("invalid 'AUTO' instruction, too few arguments")
                return(False)
            if(self.objectWithMap.pathPlanningPresent):
                if(self.objectWithMap.car.pathFolData is not None):
                    self.objectWithMap.car.pathFolData.auto = instruction[1]
                    self.objectWithMap.car.pathFolData.targetVelocity = instruction[2]
                    if(not self.objectWithMap.car.pathFolData.auto):
                        try:
                            self.objectWithMap.car.sendSpeedAngle(0.0, 0.0)
                        except:
                            print("couldn't send stop command to car (sendSpeedAngle) after disabling auto")
                else:
                    print("failed to perform 'AUTO' instruction, as car.pathFolData is None")
            else:
                print("ignored 'AUTO' instruction, because self.objectWithMap.pathPlanningPresent == False")
        elif(instruction[0] == 'FPSADJ'):
            print("ajustding mapsPerSecond to:", instruction[1])
            self.mapSendInterval = min(max(1.0/float(instruction[1]), 0.03), 2.0)
        else:
            print("can't parse unknown instruction:", instruction[0])
            return(False)
        return(True)
    
    def UIreceiver(self, threadKeepRunning, UIreceive, clientSocket):
        """a function to run on a thread, to receive data from an (ALREADY CONNECTED) client (socket).
            this will be run by runOnThread() while a connected client is established
            (in that case, threadKeepRunning is used on BOTH threads to indicate continued functioning)"""
        if(not self.initSuccess):
            print("(UIreceiver) socket not initialized, very incorrect usage")
            return(False)
        if(self.objectWithMap is None):
            print("(UIreceiver) objectWithMap is None, can't parse UI")
            return(False)
        try:
            dummyVar = clientSocket.getpeername() #just try to do something basic, to test the connection
        except Exception as excep:
            print("UIreceiver getpeername failed:", excep)
            return(None)
        while(threadKeepRunning[0] and UIreceive[0]):
            try:
                if(self.usePacketSizeHeader):
                    receivedBytes = clientSocket.recv(self.packetSizeHeaderLength) #get packet size
                    if(len(receivedBytes) == 6):
                        packetSizeString = ""
                        try:
                            packetSizeString = receivedBytes.decode()
                        except:
                            packetSizeString = ""
                            print("UIreceiver: packet header couldn't be decoded, probably out of sync")
                            receivedBytes = b''
                        if((len(packetSizeString) > 0) and (packetSizeString.isnumeric())):
                            packetSize = 0
                            try:
                                packetSize = int(packetSizeString)
                            except:
                                print("UIreceiver: couldn't packetSizeString to int")
                                receivedBytes = b''
                                packetSize = 0
                            if(packetSizeString == (str(packetSize).rjust(self.packetSizeHeaderLength, '0'))): #extra check
                                #print("good packet header:", packetSize)
                                receivedBytes = clientSocket.recv(packetSize)
                                if(len(receivedBytes) < packetSize):
                                    #print("received packet too small!?:", len(receivedBytes), packetSize)
                                    attemptsRemaining = self.maxPacketFixAttempts
                                    while((len(receivedBytes) < packetSize) and (attemptsRemaining>0)):
                                        attemptsRemaining -= 1
                                        remainingBytes = clientSocket.recv(packetSize-len(receivedBytes)) #this should wait/provide delay
                                        receivedBytes += remainingBytes
                                        if((len(receivedBytes) < packetSize) and (attemptsRemaining>0)): #only if the goal hasnt been accomplised
                                            time.sleep(0.010) #wait a tiny bit to let the bytes flow into the (underwater) buffer
                                    # if(len(receivedBytes) == packetSize):
                                    #     print("packet fixed in", self.maxPacketFixAttempts-attemptsRemaining, "attempts", packetSize)
                                    if(len(receivedBytes) != packetSize):
                                        print("UIreceiver: packet NOT FIXED!:", len(receivedBytes), packetSize)
                                        receivedBytes = b'' #avoid pickle exceptions
                            else:
                                print("UIreceiver: bad packet size header?:", receivedBytes, packetSizeString, packetSize, str(packetSize).rjust(self.packetSizeHeaderLength, '0'))
                                receivedBytes = b''
                        else:
                            print("UIreceiver: bad packet size header!:", receivedBytes, packetSizeString)
                            receivedBytes = b''
                    else:
                        print("UIreceiver: no packet size header received:", receivedBytes)
                        receivedBytes = clientSocket.recv(999999)
                else:
                    receivedBytes = clientSocket.recv(999999)
                
                #print("len(receivedBytes):", len(receivedBytes))
                receivedObj = None
                if(len(receivedBytes) > 0):
                    try:
                        receivedObj = pickle.loads(receivedBytes)
                    except Exception as excep:
                        print("UIreceiver: pickle.loads exception:", excep)
                        print("UIreceiver: ploughing on...")
                        receivedObj = None
                    #print("receivedObj:", type(receivedObj), len(pickle.dumps(receivedObj)))
                    if(type(receivedObj) is list):
                        if((type(receivedObj[0]) is str) if (len(receivedObj) >= 2) else False):
                            try:
                                self.UIparser(receivedObj)
                            except Exception as excep:
                                print("UIparser exception:", excep)
                        else:
                            print("UIreceiver: receivedObj is not an INSTRUCTIONAL list!?", receivedObj)
                    else:
                        ## if other objects can be received, parse them here (above here, with an elif())
                        print("UIreceiver: non-list object received:", receivedObj)
                else:
                    print("UIreceiver: missed packet?")
                
            except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                errorResolved = False
                # if(excep.args[0] == ):
                #     print("UIreceiver: , ignoring exception...")
                if(not errorResolved):
                    print("UIreceiver: major socket exception:", excep)
                    UIreceive[0] = False
                    return()
            except KeyboardInterrupt:
                print("UIreceiver: keyboardInterrupt!")
                UIreceive[0] = False
                return()
            except Exception as excep:
                errorResolved = False
                # if((excep.args[0] == "faillureCounter") if (type(excep.args[0]) is str) else False):
                #     print("simple failureCount overflow, ignoring exception (reconnecting)...")
                #     errorResolved = True
                if(not errorResolved):
                    print("UIreceiver: other exception:", excep)
                    UIreceive[0] = False
                    return()
            finally:
                time.sleep(0.2) #you probably won't receive more than 5 instructions per second
    
    def runOnThread(self, threadKeepRunning, autoMapSend, UIreceive):
        """(run this on a thread) manages connection (accepts clients, handles exceptions) and transmits map (or manualSendBuffer) data.
            USAGE: enter 1-sized lists with a boolean (e.g.: 'keepRunning=[True]; autoMapSend=[True]; UIreceive=[True]; runOnThread(keepRunning, autoMapSend, UIreceive)'), 
            to stop the thread, set the first (keepRunning) boolean to False and then run thisThread.join()"""
        if(((type(threadKeepRunning[0]) is not bool) if (len(threadKeepRunning)==1) else True) if (type(threadKeepRunning) is list) else True): #bad usage checking
            print("bad usage of mapTransmitterSocket.runOnThread(), for threadKeepRunning, please enter a 1-sized list with a boolean (python pointer hack)")
            #raise
            return()
        if(((type(autoMapSend[0]) is not bool) if (len(autoMapSend)==1) else True) if (type(autoMapSend) is list) else True): #bad usage checking
            print("bad usage of mapTransmitterSocket.runOnThread(), for autoMapSend, please enter a 1-sized list with a boolean (python pointer hack)")
            threadKeepRunning[0] = False
            return()
        if(((type(UIreceive[0]) is not bool) if (len(UIreceive)==1) else True) if (type(UIreceive) is list) else True): #bad usage checking
            print("bad usage of mapTransmitterSocket.runOnThread(), for UIreceive, please enter a 1-sized list with a boolean (python pointer hack)")
            threadKeepRunning[0] = False
            return()
        if(type(self.manualSendBuffer) is not list):
            print("mapTransmitterSocket.runOnThread(): self.manualSendBuffer is the wrong type, please make it (back to) a list.")
            threadKeepRunning[0] = False
            return()
        if(not self.initSuccess):
            print("mapTransmitterSocket.runOnThread(): socket not initialized, run __init__() first!")
            threadKeepRunning[0] = False
            return()
        self.runningOnThread = thr.current_thread()
        # print("running on thread:", self.runningOnThread)
        # print("main thread:", thr.main_thread())
        # print(self.runningOnThread is thr.main_thread())
        ##if you made it here, all is well and you can start accepting clients on the socket
        tempUIreceive = [UIreceive[0]] #copy UIreceive
        while(threadKeepRunning[0]):
            try:
                print("mapTransmitterSocket.runOnThread(): waiting for client...")
                clientSocket, clientAddress = self.mapSock.accept()
                print("mapTransmitterSocket.runOnThread(): cliend found:", clientAddress)
                time.sleep(1.0) #wait a little while to avoid overloading client (and potentially getting drastically out of sync, especially when sending large packets
                print("starting transmission!")
                if(self.mapSendInterval > 0):
                    mapsPerSecond = 1/self.mapSendInterval #how many Map objects are being sent per second
                    self.manualSendBuffer.append(['FPSREP', int(mapsPerSecond)]) #send the FPS as a UI report
                    print("UI reporting FPS:", mapsPerSecond)
                with clientSocket: #python's favorite way of handling a socket, this should (i think) close it up afterwards
                    if(UIreceive[0]): #if the external (main thread) overlord wants remote UI to be enabled
                        tempUIreceive[0] = True
                        UIreceiverThread = thr.Thread(target=self.UIreceiver, name="UIrecvThread", args=(threadKeepRunning, tempUIreceive, clientSocket), daemon=True)
                        UIreceiverThread.start()
                    lastSendTime = time.time()
                    while(threadKeepRunning[0]):
                        if(autoMapSend[0] and (self.objectWithMap is not None)):
                            rightNow = time.time()
                            if((rightNow - lastSendTime) > self.mapSendInterval): #dont spam
                                #print("PPS:", round(1/(rightNow-lastSendTime), 1))
                                lastSendTime = rightNow
                                bytesToSend = pickle.dumps(shallowCopyExtractMap(self.objectWithMap))
                                
                                ##debug speed measurement
                                # extractStart = time.time()
                                # extractedMap = shallowCopyExtractMap(self.objectWithMap)
                                # pickleStart = time.time()
                                # bytesToSend = pickle.dumps(extractedMap)
                                # pickleEnd = time.time()
                                # # if((pickleStart-extractStart)>0):
                                # #     print("extract:", round(1/(pickleStart-extractStart), 1))
                                # if((pickleEnd-pickleStart)>0):
                                #     print("pickle:", round(1/(pickleEnd-pickleStart), 1))
                                
                                #print("sending map of size:", len(bytesToSend))
                                if(self.usePacketSizeHeader):
                                    if(len(bytesToSend) > 999999):
                                        print("manualSendBuffer entry too large:", len(bytesToSend))
                                    else:
                                        packetSizeHeader = str(len(bytesToSend)).rjust(self.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                                        #print("sending map packet with header:", packetSizeHeader)
                                        clientSocket.sendall(packetSizeHeader + bytesToSend)
                                else:
                                    clientSocket.sendall(bytesToSend)
                        
                        while(len(self.manualSendBuffer) > 0):
                            bytesToSend = pickle.dumps(self.manualSendBuffer[0])
                            print("sending", len(bytesToSend), "bytes from self.manualSendBuffer")
                            if(self.usePacketSizeHeader):
                                if(len(bytesToSend) > 999999):
                                    print("self.manualSendBuffer entry too large:", len(bytesToSend))
                                else:
                                    packetSizeHeader = str(len(bytesToSend)).rjust(self.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                                    #print("sending manual packet with header:", packetSizeHeader)
                                    clientSocket.sendall(packetSizeHeader + bytesToSend)
                            else:
                                clientSocket.sendall(bytesToSend)
                            self.manualSendBuffer.pop(0)
                        
                        if(not UIreceive[0]): #if something outside of this thread, want the UIthread to be shut down (For whatever reason)
                            tempUIreceive[0] = False
                        elif((not tempUIreceive[0]) and UIreceive[0] and threadKeepRunning[0]): #if the thread died, but this thread didn't
                            try: #try to turn UIreceive thread back on
                                if(not UIreceiverThread.is_alive()):
                                    tempUIreceive[0] = True
                                    UIreceiverThread.start()
                                #else:
                                #    print("zombie UIreceiver thread?")
                            except:
                                print("couldn't restart UIreceive thread")
                                tempUIreceive[0] = False
                    if(not threadKeepRunning[0]):
                        print("stopping mapTransmitterSocket.runOnThread() (from accepted client)")
            except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                #print("identifying error:", excep.args[0])
                errorResolved = False
                if(excep.args[0] == 10053):
                    errorResolved = True
                    print("mapTransmitterSocket.runOnThread(): client disconnected, ignoring exception...")
                elif(excep.args[0] == 10054):
                    errorResolved = True
                    print("mapTransmitterSocket.runOnThread(): client forcibly disconnected, ignoring exception...")
                elif(excep.args[0] == 104):
                    errorResolved = True
                    print("mapTransmitterSocket.runOnThread(): connection reset by peer, ignoring exception...")
                elif(excep.args[0] == 32):
                    errorResolved = True
                    print("mapTransmitterSocket.runOnThread(): broken pipe, attempting reInit...")
                    if(self.reInit()):
                        print("broken pipe -> reInit success, continuing...")
                        errorResolved = True
                if(not errorResolved):
                    print("mapTransmitterSocket.runOnThread(): major socket exception:", excep)
                    try:
                        clientSocket.close()
                    except Exception as closeExcep:
                        print("couldn't close clientSocket after major socket exception:", closeExcep)
                    try:
                        self.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    self.initSuccess = False
                    threadKeepRunning[0] = False
                    self.runningOnThread = None
                    return()
            except KeyboardInterrupt:
                print("mapTransmitterSocket.runOnThread(): keyboardInterrupt! closing everything")
                try:
                    clientSocket.close()
                except Exception as excep:
                    print("couldn't close clientSocket after keyboardInterrupt:", excep)
                try:
                    self.mapSock.close()
                except Exception as excep:
                    print("couldn't close mapSock after keyboardInterrupt:", excep)
                self.initSuccess = False
                threadKeepRunning[0] = False
                self.runningOnThread = None
                return()
            except Exception as excep:
                print("mapTransmitterSocket.runOnThread(): other exception:", excep)
                try:
                    clientSocket.close()
                except Exception as closeExcep:
                    print("couldn't close clientSocket after other exception:", closeExcep)
                try:
                    self.mapSock.close()
                except Exception as closeExcep:
                    print("couldn't close mapSock after other exception:", closeExcep)
                self.initSuccess = False
                threadKeepRunning[0] = False
                self.runningOnThread = None
                return()
            finally:
                try: #regardless of whether it was started, try to stop it
                    tempUIreceive[0] = False
                    UIreceiverThread.join(1)
                    print("UIreceiverThread still alive?:", UIreceiverThread.is_alive())
                except:
                    print("couldn't stop UIreceiverThread(?)")


# # testing code
# if __name__ == '__main__':
#     objSender = mapTransmitterSocket('', 65432, None, False)
#     objSender.runOnThread([True], [False], [False]) #send some random data (and then wait for new data forever)