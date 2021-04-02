##this sketch has functions for sending Map objects to a remote PC to be drawn
##this will be the network server, and the drawing PC will be the client.
##you could do it the other way around, but this allows for constant settings on this side, and variable settings on the drawing side, which makes sense to me (Thijs)

## TBD: different (default) ports for maps with different classes present (coneConnecting, pathPlanning, SLAM, etc) ?

import socket
import pickle
import time

## UI instruction decoder/enumerator/parser

## this is used to shrink the sent object to only include (important) map objects, no drawing data
from Map import Map #only used in copyImportMap
def copyImportMap(classWithMapParent, mapToImport): #save all attributes from mapToImport to those by the same name in classWithMapParent
    for attrName in dir(mapToImport): #dir(class) returs a list of all class attributes
        if((not attrName.startswith('_')) and (not callable(getattr(mapToImport, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
            setattr(classWithMapParent, attrName, getattr(mapToImport, attrName)) #copy attribute


class mapReceiverSocket:
    def __init__(self, host, port=69420, objectWithMap=None, usePacketSizeHeader=True):
        """a class for receiving Map objects over the network (IPV4 TCP steam) from (car) PCs to draw them on screen (to lighten processing load on car PC)"""
        self.mapSock = None
        self.initSuccess = False
        self.usePacketSizeHeader = usePacketSizeHeader #send a (constant, predetermined length) before each sent object
        self.packetSizeHeaderLength = 6 #constant, must be same on transmitter and receiver
        
        self.host = host
        self.port = port
        
        self.runningOnThread = None
        self.manualSendBuffer = [] #used to pass objects to runOnThread() (to be sent to the host)
        
        self.maxFailureCount = 10
        self.failureCountDecrementInterval = 1.0
        self.maxPacketFixAttempts = 10
        
        self.objectWithMap = objectWithMap
        
        if(self.objectWithMap is None):
            print("asuming map is in same class as mapReceiverSocket")
            self.objectWithMap = self
        
        try:
            self.mapSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.initSuccess = True #if everything went well, set to true
        except KeyboardInterrupt: #only in case the tried code hangs forever
            print("map receiving socket init keyboardInterrupt")
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        except Exception as excep:
            print("map receiving socket init exception:", excep)
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
    
    def __del__(self):
        print("deinitializing mapReceiverSocket")
        if(self.runningOnThread is not None):
            print("THREAD STILL RUNNING!!!")
        try:
            self.mapSock.close()
        except:
            print("couldn't close() mapSock")
    
    def manualConnect(self, host=None, port=None):
        """start a connection to the host, returns whether is established.
            this method is not recommended if runOnThread is also an option"""
        if(host is not None):
            self.host = host
        if(port is not None):
            self.port = port
        if(not self.initSuccess):
            print("(manualConnect) socket not initialized, run __init__() first!")
            return(False)
        try:
            self.mapSock.connect((self.host, self.port))
            try:
                print("connected to:", self.mapSock.getpeername())
                return(True)
            except:
                print("connected, but couldn't run getpeername() ??")
                return(False)
        except KeyboardInterrupt:
            print("manualConnect stopped by keyboardInterrupt")
            return(False)
        except Exception as excep:
            print("couldn't connect to host:", self.host, "with port", self.port)
            print("reason:", excep)
            return(False)
    
    def manualReceive(self, mapImport=True, objectWithMap=None):
        """(BLOCKING) wait for-, decode- and return incoming objects (normally Maps)
            you can choose not to import incoming object as map, and only return it, by setting argument mapImport=False
            you can change the map (or map-child) object to insert the map by setting argument objectWithMap=(object to import into)"""
        if(objectWithMap is not None):
            self.objectWithMap = objectWithMap
        if(not self.initSuccess):
            print("(manualReceive) socket not initialized, run __init__() first!")
            return(False)
        try:
            dummyVar = self.mapSock.getpeername() #just try to do something basic, to test the connection
        except Exception as excep:
            print("manualReceive failed:", excep)
            return(None)
        
        if(self.usePacketSizeHeader):
            receivedBytes = self.mapSock.recv(self.packetSizeHeaderLength) #get packet size
            if(len(receivedBytes) == 6):
                packetSizeString = ""
                try:
                    packetSizeString = receivedBytes.decode()
                except:
                    packetSizeString = ""
                    print("packet header couldn't be decoded, probably out of sync")
                    receivedBytes = b''
                if((len(packetSizeString) > 0) and (packetSizeString.isnumeric())):
                    packetSize = 0
                    try:
                        packetSize = int(packetSizeString)
                    except:
                        print("couldn't packetSizeString to int")
                        receivedBytes = b''
                        packetSize = 0
                    if(packetSizeString == (str(packetSize).rjust(self.packetSizeHeaderLength, '0'))): #extra check
                        #print("good packet header:", packetSize)
                        receivedBytes = self.mapSock.recv(packetSize)
                        if(len(receivedBytes) < packetSize):
                            #print("received packet too small!?:", len(receivedBytes), packetSize)
                            attemptsRemaining = self.maxPacketFixAttempts
                            while((len(receivedBytes) < packetSize) and (attemptsRemaining>0)):
                                attemptsRemaining -= 1
                                remainingBytes = self.mapSock.recv(packetSize-len(receivedBytes)) #this should wait/provide delay
                                receivedBytes += remainingBytes
                                if((len(receivedBytes) < packetSize) and (attemptsRemaining>0)): #only if the goal hasnt been accomplised
                                    time.sleep(0.005) #wait a tiny bit
                            # if(len(receivedBytes) == packetSize):
                            #     print("packet fixed in", self.maxPacketFixAttempts-attemptsRemaining, "attempts", packetSize)
                            if(len(receivedBytes) != packetSize):
                                print("packet NOT FIXED!:", len(receivedBytes), packetSize)
                                receivedBytes = b'' #avoid pickle exceptions
                    else:
                        print("bad packet size header?:", receivedBytes, packetSizeString, packetSize, str(packetSize).rjust(self.packetSizeHeaderLength, '0'))
                        receivedBytes = b''
                else:
                    print("bad packet size header!:", receivedBytes, packetSizeString)
                    receivedBytes = b''
            else:
                print("no packet size header received:", receivedBytes)
                receivedBytes = self.mapSock.recv(999999)
        else:
            receivedBytes = self.mapSock.recv(999999)
        #print("len(receivedBytes):", len(receivedBytes))
        receivedObj = None
        if(len(receivedBytes) > 0):
            try:
                receivedObj = pickle.loads(receivedBytes)
            except Exception as excep:
                print("pickle.loads exception:", excep)
                print("ploughing on...")
                receivedObj = None
            #print("receivedObj:", type(receivedObj), len(pickle.dumps(receivedObj)))
            if(type(receivedObj) is Map):
                copyImportMap(self.objectWithMap, receivedObj)
            else:
                ## if other objects can be received, parse them here (above here, with an elif())
                print("non-map object received:", receivedObj)
        else:
            print("missed packet?")
            return(None)
    
    def manualSend(self, manualSendBuffer: list):
        """manually send all entries of the entered buffer (input is list of objects to send)"""
        if(type(manualSendBuffer) is not list):
            print("(manualSend) bad argument entered for manualSendBuffer! please enter a list.")
            return()
        if(not self.initSuccess):
            print("(manualSend) socket not initialized, run __init__() first!")
            return()
        try:
            dummyVar = self.mapSock.getpeername() #just try to do something basic, to test the connection
        except Exception as excep:
            print("manualReceive failed:", excep)
            return(None)
        while(len(manualSendBuffer) > 0):
            bytesToSend = pickle.dumps(manualSendBuffer[0])
            #print("sending", len(bytesToSend), "bytes from manualSendBuffer")
            if(self.usePacketSizeHeader):
                if(len(bytesToSend) > 999999):
                    print("manualSendBuffer entry too large:", len(bytesToSend))
                else:
                    packetSizeHeader = str(len(bytesToSend)).rjust(self.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                    self.mapSock.sendall(packetSizeHeader + bytesToSend)
            else:
                self.mapSock.sendall(bytesToSend)
            manualSendBuffer.pop(0)
    
    def manualClose(self):
        """close the socket (MUST be done when exiting the program/deinitializing the object)"""
        try:
            self.mapSock.close()
            print("manualClose() success")
        except Exception as closeExcep:
            print("couldn't close mapSock after other exception:", closeExcep)
        self.initSuccess = False
    
    def reInit(self):
        self.initSuccess = False
        #if(self.runningOnThread is not None): #note: reInit() may have been called from inside the thread, in an attempt to stay alive
        try:
            self.mapSock.close() #close existing/previous socket
        except:
            print("couldn't close() mapSock")
        time.sleep(0.05) #wait just a little bit (50ms) for extra safety
        
        try: #try to re-establish socket
            self.mapSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.initSuccess = True #if everything went well, set to true
        except KeyboardInterrupt: #only in case the tried code hangs forever
            print("map receiving socket reInit keyboardInterrupt")
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        except Exception as excep:
            print("map receiving socket reInit exception:", excep)
            try:
                self.mapSock.close()
            except:
                print("couldn't close() mapSock")
            self.initSuccess = False
        finally:
            return(self.initSuccess) #no need to return, if you just check self.initSuccess afterwards, but why not right
    
    
    def runOnThread(self, threadKeepRunning):
        """(run this on a thread) manages connection (connects to server, handles exceptions) and retrieves & stores map data.
            USAGE: enter a 1-sized list with a boolean (e.g.: 'keepRunning=[True]; runOnThread(keepRunning)'), 
            to stop the thread, set that boolean to False and then run thisThread.join()"""
        if(((type(threadKeepRunning[0]) is not bool) if (len(threadKeepRunning)==1) else True) if (type(threadKeepRunning) is list) else True): #bad usage checking
            print("bad usage of mapReceiverSocket.runOnThread(), please enter a 1-sized list with a boolean (python pointer hack)")
            #raise
            return()
        if(type(self.manualSendBuffer) is not list):
            print("mapReceiverSocket.runOnThread(): self.manualSendBuffer is wrong type(), please enter a list.")
            threadKeepRunning[0] = False
            return()
        if(not self.initSuccess):
            print("mapReceiverSocket.runOnThread(): socket not initialized, run __init__() first!")
            threadKeepRunning[0] = False
            return()
        import threading as thr
        self.runningOnThread = thr.current_thread()
        ##if you made it here, all is well and you can start accepting clients on the socket
        while(threadKeepRunning[0]):
            try:
                print("mapReceiverSocket.runOnThread(): connecting so server...")
                self.mapSock.connect((self.host, self.port))
                with self.mapSock: #python likes this(?)
                    print("connected to:", self.mapSock.getpeername())
                    # if(self.usePacketSizeHeader):
                    #     firstPacket = self.mapSock.recv(999999) #receive any straddler-data
                    #     print("first packet received:", len(firstPacket))
                    failureCounter = 0
                    failureCountDecrementTimer = time.time()
                    lastMapImportTime = time.time()
                    while(threadKeepRunning[0]):
                        if(self.usePacketSizeHeader):
                            receivedBytes = self.mapSock.recv(self.packetSizeHeaderLength) #get packet size
                            if(len(receivedBytes) == 6):
                                packetSizeString = ""
                                try:
                                    packetSizeString = receivedBytes.decode()
                                except:
                                    packetSizeString = ""
                                    print("packet header couldn't be decoded, probably out of sync")
                                    #failureCounter += 1
                                    receivedBytes = b''
                                if((len(packetSizeString) > 0) and (packetSizeString.isnumeric())):
                                    packetSize = 0
                                    try:
                                        packetSize = int(packetSizeString)
                                    except:
                                        print("couldn't packetSizeString to int")
                                        #failureCounter += 1
                                        receivedBytes = b''
                                        packetSize = 0
                                    if(packetSizeString == (str(packetSize).rjust(self.packetSizeHeaderLength, '0'))): #extra check
                                        #print("good packet header:", packetSize)
                                        receivedBytes = self.mapSock.recv(packetSize)
                                        if(len(receivedBytes) < packetSize):
                                            #print("received packet too small!?:", len(receivedBytes), packetSize)
                                            attemptsRemaining = self.maxPacketFixAttempts
                                            while((len(receivedBytes) < packetSize) and (attemptsRemaining>0)):
                                                attemptsRemaining -= 1
                                                remainingBytes = self.mapSock.recv(packetSize-len(receivedBytes)) #this should wait/provide delay
                                                receivedBytes += remainingBytes
                                                if((len(receivedBytes) < packetSize) and (attemptsRemaining>0)): #only if the goal hasnt been accomplised
                                                    time.sleep(0.005) #wait a tiny bit to let the bytes flow into the (underwater) buffer
                                            # if(len(receivedBytes) == packetSize):
                                            #     print("packet fixed in", self.maxPacketFixAttempts-attemptsRemaining, "attempts", packetSize)
                                            if(len(receivedBytes) != packetSize):
                                                print("packet NOT FIXED!:", len(receivedBytes), packetSize)
                                                failureCounter += 1
                                                receivedBytes = b'' #avoid pickle exceptions
                                    else:
                                        print("bad packet size header?:", receivedBytes, packetSizeString, packetSize, str(packetSize).rjust(self.packetSizeHeaderLength, '0'))
                                        #failureCounter += 1
                                        receivedBytes = b''
                                else:
                                    print("bad packet size header!:", receivedBytes, packetSizeString)
                                    #failureCounter += 1
                                    receivedBytes = b''
                            else:
                                print("no packet size header received:", receivedBytes)
                                failureCounter += 1
                                receivedBytes = self.mapSock.recv(999999)
                        else:
                            receivedBytes = self.mapSock.recv(999999)
                        #print("len(receivedBytes):", len(receivedBytes))
                        receivedObj = None
                        if(len(receivedBytes) > 0):
                            try:
                                receivedObj = pickle.loads(receivedBytes)
                            except Exception as excep:
                                print("pickle.loads exception:", excep)
                                print("ploughing on...")
                                failureCounter += 1
                                receivedObj = None
                            #print("receivedObj:", type(receivedObj), len(pickle.dumps(receivedObj)))
                            if(type(receivedObj) is Map):
                                rightNow = time.time()
                                if((rightNow-lastMapImportTime)>0):
                                    print("PPS:", round(1/(rightNow-lastMapImportTime), 1))
                                lastMapImportTime = rightNow
                                copyImportMap(self.objectWithMap, receivedObj)
                            else:
                                ## if other objects can be received, parse them here (above here, with an elif())
                                print("non-map object received:", receivedObj)
                        else:
                            failureCounter += 1
                            print("missed packet?")
                        
                        if(failureCounter > self.maxFailureCount):
                            print("too many faillures")
                            raise(Exception("faillureCounter"))
                            # threadKeepRunning[0] = False
                            # #self.initSuccess = False
                            # self.runningOnThread = None
                            # return()
                        if((time.time() - failureCountDecrementTimer) > self.failureCountDecrementInterval):
                            failureCountDecrementTimer = time.time()
                            if(failureCounter > 0):
                                print("decrementing failureCounter:", failureCounter)
                                failureCounter -= 1
                            #failureCounter = max(failureCounter-1, 0)
                        
                        if(len(self.manualSendBuffer) > 0):
                            bytesToSend = pickle.dumps(self.manualSendBuffer[0])
                            #print("sending", len(bytesToSend), "bytes from manualSendBuffer")
                            if(self.usePacketSizeHeader):
                                if(len(bytesToSend) > 999999):
                                    print("manualSendBuffer entry too large:", len(bytesToSend))
                                else:
                                    packetSizeHeader = str(len(bytesToSend)).rjust(self.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                                    self.mapSock.sendall(packetSizeHeader + bytesToSend)
                            else:
                                self.mapSock.sendall(bytesToSend)
                            self.manualSendBuffer.pop(0)
                        
            except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                errorResolved = False
                if(excep.args[0] == 10038): #'an operation was performed by something that is not a socket'
                    print("mapReceiverSocket.runOnThread(): mapSock is not socket?, attempting reInit...")
                    if(self.reInit()):
                        print("mapSock not socket -> reInit success, continuing...")
                        errorResolved = True
                if(not errorResolved):
                    print("mapReceiverSocket.runOnThread(): major socket exception:", excep)
                    try:
                        self.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    threadKeepRunning[0] = False
                    self.initSuccess = False
                    self.runningOnThread = None
                    return()
            except KeyboardInterrupt:
                print("mapReceiverSocket.runOnThread(): keyboardInterrupt! closing everything")
                try:
                    self.mapSock.close()
                except Exception as excep:
                    print("couldn't close mapSock after keyboardInterrupt:", excep)
                threadKeepRunning[0] = False
                self.initSuccess = False
                self.runningOnThread = None
                return()
            except Exception as excep:
                errorResolved = False
                if((excep.args[0] == "faillureCounter") if (type(excep.args[0]) is str) else False):
                    print("simple failureCount overflow, ignoring exception (reconnecting)...")
                    errorResolved = True
                if(not errorResolved):
                    print("mapReceiverSocket.runOnThread(): other exception:", excep)
                    try:
                        self.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    threadKeepRunning[0] = False
                    self.initSuccess = False
                    self.runningOnThread = None
                    return()


# # testing code
# if __name__ == '__main__':
#     objGetter = mapReceiverSocket('127.0.0.1', 65432, None, False)
#     objGetter.runOnThread([True]) #send some random data (and then wait for new data forever)