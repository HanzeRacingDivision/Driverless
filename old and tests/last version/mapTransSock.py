##this sketch has functions for sending Map objects to a remote PC to be drawn
##this will be the network server, and the drawing PC will be the client.
##you could do it the other way around, but this allows for constant settings on this side, and variable settings on the drawing side, which makes sense to me (Thijs)

## TBD: different (default) ports for maps with different classes present (coneConnecting, pathPlanning, SLAM, etc) ?

import socket
import pickle
import time

#import GF.generalFunctions as GF #only used for cone deletion (UI parser)

import multiprocessing as MP
import sharedMem       as SM  #my own shared memory manager (a double-buffered, one-way, pickle-based (single-object) sharedMemory wrapper)

# ##remote connection UI:
# def remoteInstructionSend(socketToSendFrom, instruction):
#     """send instruction to remove instance (car/host)"""
#     if(socketToSendFrom.runningOnThread is not None):
#         #print("threaded send")
#         socketToSendFrom.manualSendBuffer.append(instruction)
#     else:
#         #print("manual send")
#         try:
#             socketToSendFrom.manualSend([instruction])
#         except Exception as excep:
#             print("remoteInstructionSend manualSend exception:", excep)

class remoteProcess(MP.Process):
    """a multiprocessing mapTransSock"""
    def __init__(self, connToMaster, sharedMemInit, IPhost='', IPport=65432, mapSendInterval=0.2, acceptUIreceive=True):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.sharedMemInit = sharedMemInit
        self.IPhost = IPhost
        self.IPport = IPport
        self.mapSendInterval = mapSendInterval
        self.acceptUIreceive = acceptUIreceive
        
        self.keepalive = MP.Semaphore() # this (functional) boolean stops the while() loop in run()
    
    def run(self):
        #self.keepalive.release() #for running this process again after it was ended
        try:
            import mapTransSock   as MS
            slaveSharedMem = SM.sharedMemReadWrapper(*self.sharedMemInit)
            #initMap = slaveSharedMem.readObj(self.keepalive) #wait for the first map object to come in (with no timeout?)
            mapSender = MS.mapTransmitterSocket(host=self.IPhost, port=self.IPport)
            mapSender.mapSendInterval = self.mapSendInterval #start safe, you can bring this number down if the connection is good (FPS = 1/this)
            if(self.acceptUIreceive): #if the external (main thread) wants remote UI to be enabled
                connToUIreceiver, connToSender = MP.Pipe(False) #disable duplex (for now)
            while(self.keepalive.get_value()):
                try:
                    print("remoteProcess: waiting for client...")
                    clientSocket, clientAddress = mapSender.mapSock.accept()
                    print("remoteProcess: cliend found:", clientAddress)
                    time.sleep(1.0) #wait a little while to avoid overloading client (and potentially getting drastically out of sync, especially when sending large packets
                    print("remoteProcess: starting transmission!")
                    if(mapSender.mapSendInterval > 0): #avoid divide by 0
                        mapsPerSecond = 1/mapSender.mapSendInterval #how many Map objects are being sent per second
                        #mapSender.manualSendBuffer.append(['FPSREP', int(mapsPerSecond)]) #send the FPS as a UI report
                        print("remoteProcess: UI reporting FPS:", mapsPerSecond)
                    with clientSocket: #python's favorite way of handling a socket, this should (i think) close it up afterwards
                        if(self.acceptUIreceive): #if the external (main thread) wants remote UI to be enabled
                            UIreceiver = UIreceiveProcess(self.connToMaster, connToSender, self.keepalive, clientSocket)
                            UIreceiver.start()
                        lastSendTime = time.time()
                        while(self.keepalive.get_value()):     ################## main loop
                            loopStart = time.time()
                            if((loopStart - lastSendTime) > mapSender.mapSendInterval): #dont spam
                                #print("PPS:", round(1/(loopStart-lastSendTime), 1))
                                lastSendTime = loopStart
                                bytesToSend = slaveSharedMem.readNewBytes(self.keepalive)
                                
                                #print("sending map of size:", len(bytesToSend))
                                if(len(bytesToSend) > 999999):
                                    print("bytesToSend too large:", len(bytesToSend))
                                else:
                                    packetSizeHeader = str(len(bytesToSend)).rjust(mapSender.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                                    #print("sending map packet with header:", packetSizeHeader)
                                    clientSocket.sendall(packetSizeHeader + bytesToSend)
                            
                            # while(len(self.manualSendBuffer) > 0):
                            #     bytesToSend = pickle.dumps(self.manualSendBuffer[0])
                            #     #print("sending", len(bytesToSend), "bytes from self.manualSendBuffer")
                            #     if(len(bytesToSend) > 999999):
                            #         print("self.manualSendBuffer entry too large:", len(bytesToSend))
                            #     else:
                            #         packetSizeHeader = str(len(bytesToSend)).rjust(mapSender.packetSizeHeaderLength, '0').encode() #6 bytes (constant length) that indicate the size of the (soon to be) sent object
                            #         #print("sending manual packet with header:", packetSizeHeader)
                            #         clientSocket.sendall(packetSizeHeader + bytesToSend)
                            #     self.manualSendBuffer.pop(0)
                            
                            if(self.acceptUIreceive): #if the external (main thread) wants remote UI to be enabled
                                try:
                                    if(UIreceiver.is_alive()):
                                        if(connToUIreceiver.poll()):
                                            UIreceiverData = connToUIreceiver.recv()
                                            if(UIreceiverData[0] == 'FPSADJ'):
                                                print("ajustding mapsPerSecond to:", UIreceiverData[1])
                                                mapSender.mapSendInterval = min(max(1.0/float(UIreceiverData[1]), 0.03), 2.0)
                                    elif(self.keepalive.get_value()):
                                        ## attempt to recover the UIreceiver process
                                        print("restarting UIreceiverProcess")
                                        if(not (UIreceiver.keepalive.get_value())):
                                            UIreceiver.keepalive.release()
                                        UIreceiver.join(0.5)
                                        UIreceiver = UIreceiveProcess(self.connToMaster, connToSender, self.keepalive, clientSocket)
                                        UIreceiver.start()
                                except Exception as excep:
                                    print("UIreceiver interface error:", excep, excep.args)
                                    time.sleep(0.3)
                            
                            loopEnd = time.time()
                            if((loopEnd-loopStart)>(3*mapSender.mapSendInterval)):
                                print("remoteProcess running slow", 1/(loopEnd-loopStart))
                except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                    print("identifying error:", excep.args[0])
                    errorResolved = False
                    if(excep.args[0] == 10053):
                        errorResolved = True
                        print("remoteProcess: client disconnected, ignoring exception...")
                    elif(excep.args[0] == 10054):
                        errorResolved = True
                        print("remoteProcess: client forcibly disconnected, ignoring exception...")
                    elif(excep.args[0] == 104):
                        errorResolved = True
                        print("remoteProcess: connection reset by peer, ignoring exception...")
                    elif(excep.args[0] == 32):
                        errorResolved = True
                        print("remoteProcess: broken pipe, attempting reInit...")
                        if(mapSender.reInit()):
                            print("broken pipe -> reInit success, continuing...")
                            errorResolved = True
                    if(not errorResolved):
                        print("remoteProcess: major socket exception:", excep)
                        
                        self.keepalive.acquire()
                        try:
                            clientSocket.close()
                        except Exception as closeExcep:
                            print("couldn't close clientSocket after major socket exception:", closeExcep)
                        try:
                            mapSender.mapSock.close()
                        except Exception as closeExcep:
                            print("couldn't close mapSock after other exception:", closeExcep)
                        mapSender.initSuccess = False
                        try:
                            if(UIreceiver.keepalive.get_value()):
                                UIreceiver.keepalive.acquire()
                        except:
                            time.sleep(0.001) #do nothing
                        return()
                    time.sleep(0.25) #a little delay (to make cmdline output less spammy)
                except KeyboardInterrupt:
                    print("remoteProcess: keyboardInterrupt! closing everything")
                    self.keepalive.acquire()
                    try:
                        clientSocket.close()
                    except Exception as excep:
                        print("couldn't close clientSocket after keyboardInterrupt:", excep)
                    try:
                        mapSender.mapSock.close()
                    except Exception as excep:
                        print("couldn't close mapSock after keyboardInterrupt:", excep)
                    mapSender.initSuccess = False
                    try:
                        if(UIreceiver.keepalive.get_value()):
                            UIreceiver.keepalive.acquire()
                    except:
                        time.sleep(0.001) #do nothing
                    return()
                except Exception as excep:
                    print("remoteProcess: other exception:", excep)
                    self.keepalive.acquire()
                    try:
                        clientSocket.close()
                    except Exception as closeExcep:
                        print("couldn't close clientSocket after other exception:", closeExcep)
                    try:
                        mapSender.mapSock.close()
                    except Exception as closeExcep:
                        print("couldn't close mapSock after other exception:", closeExcep)
                    mapSender.initSuccess = False
                    try:
                        if(UIreceiver.keepalive.get_value()):
                            UIreceiver.keepalive.acquire()
                    except:
                        time.sleep(0.001) #do nothing
                    return()
                finally:
                    try: #regardless of whether it was started, try to stop it
                        if(UIreceiver.keepalive.get_value()):
                            UIreceiver.keepalive.acquire()
                        if(UIreceiver.is_alive()):
                            UIreceiver.terminate()
                            UIreceiver.join(1)
                        print("UIreceiverProcess still alive?:", UIreceiver.is_alive())
                        #UIreceiver.keepalive.release()
                    except:
                        print("couldn't stop UIreceiverProcess(?)")
        finally:
            print("remoteProcess ending")
            try:
                connToUIreceiver.close()
                connToSender.close()
            except: 
                print("couldn't close UIreceiver pipes")
            try:
                if(UIreceiver.is_alive()):
                    print("UIreceiver.is_alive():", UIreceiver.is_alive())
                    UIreceiver.stopLoop()
            except:
                try:
                    if(UIreceiver.is_alive()):
                        UIreceiver.terminate()
                        UIreceiver.join(2)
                finally:
                    print("couldn't stopLoop UIreceiver")
        return()
    
    def stopLoop(self, alsoJoin=True, joinTimeout=5):
        if(self.keepalive.get_value()):
            self.keepalive.acquire()
        if(alsoJoin and self.is_alive()):
            self.join(joinTimeout)
            if(self.is_alive()):
                print("stopLoop join failed, process still alive!")
            else:
                self.keepalive.release() #setup for next start (not needed)

class UIreceiveProcess(MP.Process):
    """a multiprocessing mapTransSock"""
    def __init__(self, connToMaster, connToSender, keepaliveSender, clientSocket):
        """an override of the Process class init function.
            initializes keekalive semaphore and inter-process communication"""
        MP.Process.__init__(self) #init Process class
        self.connToMaster = connToMaster
        self.connToSender = connToSender
        self.keepaliveSender = keepaliveSender
        self.clientSocket = clientSocket
        
        self.keepalive = MP.Semaphore() # this thread doesnt have its own keepalive by default (but you can pass a new MP.Semaphore() if you do want it to)
    
    def run(self):
        packetSizeHeaderLength = 6 #TBD! get these from somewhere else (cleanly)
        maxPacketFixAttempts = 10
        try:
            # if(not (self.keepalive.get_value())):
            #     self.keepalive.release()
            try:
                dummyVar = self.clientSocket.getpeername() #just try to do something basic, to test the connection
            except Exception as excep:
                print("UIreceiver getpeername failed:", excep)
                return(None)
            
            while((self.keepalive.get_value()) and (self.keepaliveSender.get_value())):
                try:
                    receivedBytes = self.clientSocket.recv(packetSizeHeaderLength) #get packet size
                    if(len(receivedBytes) == packetSizeHeaderLength):
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
                            if(packetSizeString == (str(packetSize).rjust(packetSizeHeaderLength, '0'))): #extra check
                                #print("good packet header:", packetSize)
                                receivedBytes = self.clientSocket.recv(packetSize)
                                if(len(receivedBytes) < packetSize):
                                    #print("received packet too small!?:", len(receivedBytes), packetSize)
                                    attemptsRemaining = maxPacketFixAttempts
                                    while((len(receivedBytes) < packetSize) and (attemptsRemaining>0)):
                                        attemptsRemaining -= 1
                                        remainingBytes = self.clientSocket.recv(packetSize-len(receivedBytes)) #this should wait/provide delay
                                        receivedBytes += remainingBytes
                                        if((len(receivedBytes) < packetSize) and (attemptsRemaining>0)): #only if the goal hasnt been accomplised
                                            time.sleep(0.010) #wait a tiny bit to let the bytes flow into the (underwater) buffer
                                    # if(len(receivedBytes) == packetSize):
                                    #     print("packet fixed in", maxPacketFixAttempts-attemptsRemaining, "attempts", packetSize)
                                    if(len(receivedBytes) != packetSize):
                                        print("UIreceiver: packet NOT FIXED!:", len(receivedBytes), packetSize)
                                        receivedBytes = b'' #avoid pickle exceptions
                            else:
                                print("UIreceiver: bad packet size header?:", receivedBytes, packetSizeString, packetSize, str(packetSize).rjust(packetSizeHeaderLength, '0'))
                                receivedBytes = b''
                        else:
                            print("UIreceiver: bad packet size header!:", receivedBytes, packetSizeString)
                            receivedBytes = b''
                    else:
                        print("UIreceiver: no packet size header received:", receivedBytes)
                        receivedBytes = self.clientSocket.recv(999999)
                    
                    if(len(receivedBytes) > 0):
                        try:
                            receivedObj = pickle.loads(receivedBytes)
                            if(receivedObj._hasData_):
                                specialInstructions = []
                                for i in range(3):
                                    for instruction in receivedObj._customLog_[i]:
                                        if(instruction[0] == 'FPSADJ'):
                                            specialInstructions.append(instruction)
                                            receivedObj._customLog_[i].remove(instruction)
                                for instruction in specialInstructions:
                                    self.connToSender.send(instruction)
                                if(receivedObj._hasData_): #if it still has data
                                    self.connToMaster.send(receivedObj)
                                    #self.connToMaster.send_bytes(receivedBytes) #you can send the original data (skip re-pickling) and let any special instructions be ignored over there
                        except Exception as excep:
                            print("UIreceiver: unpickling error?:", excep, excep.args)
                    
                except socket.error as excep: #handleable exceptions, this will only end things on major exceptions (unexpected ones)
                    errorResolved = False
                    # if(excep.args[0] == ):
                    #     print("UIreceiver: , ignoring exception...")
                    if(not errorResolved):
                        print("UIreceiver: major socket exception:", excep)
                        self.keepalive.acquire()
                        return()
                except KeyboardInterrupt:
                    print("UIreceiver: keyboardInterrupt!")
                    self.keepalive.acquire()
                    return()
                except Exception as excep:
                    errorResolved = False
                    # if((excep.args[0] == "faillureCounter") if (type(excep.args[0]) is str) else False):
                    #     print("simple failureCount overflow, ignoring exception (reconnecting)...")
                    #     errorResolved = True
                    if(not errorResolved):
                        print("UIreceiver: other exception:", excep)
                        self.keepalive.acquire()
                        return()
                time.sleep(0.2) #limit loop speed (you probably won't receive more than 5 instructions per second)
        finally:
            print("UIreceiveProcess ending")
        return()
    
    def stopLoop(self, alsoJoin=True, joinTimeout=5):
        if(self.keepalive.get_value()):
            self.keepalive.acquire()
        if(alsoJoin and self.is_alive()):
            self.join(joinTimeout)
            if(self.is_alive()):
                print("stopLoop join failed, process still alive!")
            else:
                self.keepalive.release() #setup for next start (not needed)

class mapTransmitterSocket:
    def __init__(self, host='', port=69420):
        """a class for transmitting Map objects over the network (IPV4 TCP steam) to nearby PCs to draw them on screen (to lighten processing load on car PC)"""
        self.mapSock = None
        self.initSuccess = False
        self.packetSizeHeaderLength = 6 #constant, must be same on transmitter and receiver
        
        self.host = host
        self.port = port
        
        self.maxPacketFixAttempts = 10 #only for UIreceiver
        
        self.mapSendInterval = 0.25 #avoids spamming
        
        #self.mapToUse = mapToUse
        
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
    
    # def UIparserExtended(self, mapToUse, instruction):
    #     """parse the received instruction (like place-cone or start-auto)
    #         this fulfills the funcion of drawDriverless, but with less(?) CPU/GPU drag"""
    #     if(instruction[0] == 'FPSADJ'):
    #         print("ajustding mapsPerSecond to:", instruction[1])
    #         self.mapSendInterval = min(max(1.0/float(instruction[1]), 0.03), 2.0)
    #     elif(instruction[0] == 'MAPSAV'): #overwrites the MAPSAV parser in genUI
    #         try:
    #             import map_loader as ML
    #             saveStartTime = time.time()
    #             filename, map_file = ML.save_map(mapToUse, instruction[1]) #if instruction[1] (filename) is None, a filename will be autogenerated
    #             ## i noticed that save_map is very very slow from here. This might be a problem with threading, like accessing resources takes long???
    #             print("(remotely instructed) saving took", round(time.time()-saveStartTime, 2), "seconds")
    #             # rawExcelFile = open(filename, "rb").read() #get the pickled data (pandas .to_pickle doesnt support outputting to a bytearray object right away)
    #             # print("SAVREP filesize:", len(rawExcelFile))
    #             # self.manualSendBuffer.append(['SAVREP', filename, rawExcelFile])
    #         except Exception as excep:
    #             print("failed to perform 'MAPSAV' instruction, exception:", excep)
    #     else:
    #         return(genUI.UIparser(mapToUse, instruction))
    #     return(True)
    


