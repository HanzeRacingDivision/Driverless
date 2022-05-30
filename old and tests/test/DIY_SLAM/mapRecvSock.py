##this sketch has functions for sending Map objects to a remote PC to be drawn
##this will be the network server, and the drawing PC will be the client.
##you could do it the other way around, but this allows for constant settings on this side, and variable settings on the drawing side, which makes sense to me (Thijs)

## TBD: different (default) ports for maps with different classes present (coneConnecting, pathPlanning, SLAM, etc) ?

import socket
import pickle
import time

## UI instruction decoder/enumerator/parser

# ## this is used to shrink the sent object to only include (important) map objects, no drawing data
# from Map import Map #only used in copyImportMap
# def copyImportMap(classWithMapParent, mapToImport):
#     """save all attributes from mapToImport to those by the same name in classWithMapParent"""
#     for attrName in dir(mapToImport): #dir(class) returs a list of all class attributes
#         if((not attrName.startswith('_')) and (not callable(getattr(mapToImport, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
#             setattr(classWithMapParent, attrName, getattr(mapToImport, attrName)) #copy attribute


class mapReceiverSocket:
    def __init__(self, host, port=69420):
        """a class for receiving Map objects over the network (IPV4 TCP steam) from (car) PCs to draw them on screen (to lighten processing load on car PC)"""
        self.mapSock = None
        self.initSuccess = False
        self.packetSizeHeaderLength = 6 #constant, must be same on transmitter and receiver
        
        self.host = host
        self.port = port
        
        # self.manualSendBuffer = [] #used to pass objects to runOnThread() (to be sent to the host)
        
        self.maxFailureCount = 10
        self.failureCountDecrementInterval = 1.0
        self.maxPacketFixAttempts = 10
        
        # self.mapToUse = mapToUse
        # try:
        #     if(not self.mapToUse.isRemote):
        #         print("warning!, mapReceiverSocket.mapToUse.isRemote = false")
        # except Exception as excep:
        #     print("couldn't check mapReceiverSocket.mapToUse.isRemote", excep, self.mapToUse)
        
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
    
    def manualReceiveBytes(self):
        if(not self.initSuccess):
            print("(manualReceive) socket not initialized, run __init__() first!")
            return(None)
        try:
            dummyVar = self.mapSock.getpeername() #just try to do something basic, to test the connection
        except Exception as excep:
            print("manualReceive failed:", excep)
            return(None)
        
        receivedBytes = self.mapSock.recv(self.packetSizeHeaderLength) #get packet size
        if(len(receivedBytes) == self.packetSizeHeaderLength):
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
        #print("len(receivedBytes):", len(receivedBytes))
        return(receivedBytes)
    
    def manualReceive(self):
        """(BLOCKING) wait for-, decode- and return incoming objects (normally Maps)"""
        receivedBytes = self.manualReceiveBytes()
        
        receivedObj = None
        if(len(receivedBytes) > 0):
            try:
                receivedObj = pickle.loads(receivedBytes)
            except Exception as excep:
                print("pickle.loads exception:", excep)
                print("ploughing on...")
                receivedObj = None
            return(receivedObj)
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
            if(len(bytesToSend) > 999999):
                print("manualSendBuffer entry too large:", len(bytesToSend))
            else:
                packetSizeHeader = str(len(bytesToSend)).rjust(self.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                self.mapSock.sendall(packetSizeHeader + bytesToSend)
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
    
    