#usage: import this into another program, make an instance of the class,
#        either connect at class initialization, or later using connect() (comPorts are named things like 'COM4' on windows)
#        you can send instructions to the car using sendSpeedAngle()
#        you have to either run getFeedback() regularly (manually), or start runOnThread() on a dedicated thread
#        the sensor data received from the car is stored in several FIFO buffers (for better math/filtering) (see class below for buffer names
#        (for example) angleFIFO[0] is the most recent steering angle measurement data, it was received at feedbackTimestampFIFO[0]


import serial
import serial.tools.list_ports
import time
import numpy as np

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use

from numba import njit, prange, vectorize, int64, float64, float32

## constants
START_SYNC_BYTES = np.array([255, 255], dtype=np.uint8) #can be any size/value, as long as the MCU has the same ones
END_SYNC_BYTES = np.array([13, 10], dtype=np.uint8) #can be any size/value, as long as the MCU has the same ones

DATA_LEN = 4 + 2 + 4 #byte sizes of sent datatypes

PACKET_LEN = len(START_SYNC_BYTES) + DATA_LEN + len(END_SYNC_BYTES)
START_SYNC_BYTE_INDEX = len(START_SYNC_BYTES)
END_SYNC_BYTE_INDEX = PACKET_LEN-len(END_SYNC_BYTES)

## angle communication constant (all to avoid the use of floats)
ANGLE_INTIFY_MULT = 500.0 #degree angle (float) is multiplied by this, and then turned into integer
## wheel encoder constants
#ENCO_HOLES = 1.0/6.0; //6 light-pulse-triggering holes in the encoder disc
ENCO_HOLES = 1.0/12.0; #12 light-pulse-triggering holes in the encoder disc
ENCO_GEAR_RATIO = 20.0/37.0; #37T gear on wheel side, 22T gear on encoder disc side
ENCO_REV_PER_COUNT = ENCO_HOLES * ENCO_GEAR_RATIO; #counts * this = wheel revolutions
## the previous values are absolute and precise, the wheel circumference is measured/calibrated and can vary a bit
ENCO_WHEEL_CIRC = np.pi * 62.4; #2*pi*r = pi*d = circumference
ENCO_MM_PER_COUNT = ENCO_REV_PER_COUNT * ENCO_WHEEL_CIRC
ENCO_M_PER_COUNT = ENCO_MM_PER_COUNT / 1000.0


@njit
def parsePacket(packetBuf: np.ndarray):
    """parse a serial packet (bytes). Returns an np.array with the values
        numba compiled!"""
    return(np.array([np.int64(np.frombuffer(packetBuf[0:4], dtype=np.uint32)[0]),
                     np.int64(np.frombuffer(packetBuf[4:6], dtype=np.int16)[0]),
                     np.int64(np.frombuffer(packetBuf[6:10], dtype=np.uint32)[0])], dtype=np.int64))

@njit
def extractPackets(serialData: bytes, packetBuf: np.ndarray, packetBufLen: int):
    """handle serial data (bytes) to produce packets
        numba compiled!"""
    maxPacketCount = int((len(serialData)+packetBufLen) / PACKET_LEN) #the max number of packets that could possibly be extracted from this data
    parsedPackets = np.empty((maxPacketCount,3), dtype=np.int64) #instead of making an appendable list (which is allowed in @njit), make a (possibly oversized) array. (faster)
    parsedPacketCount = 0 #if packets fail (sync issues or otherwise) then some the last (few) indices will not contain actual data. Use this instead of len(resultList)
    for i in range(len(serialData)):
        if(packetBufLen < START_SYNC_BYTE_INDEX):
            if(serialData[i] == START_SYNC_BYTES[packetBufLen]):
                packetBufLen += 1
            else:
                print("packet start sync failed after", packetBufLen, "bytes")
                packetBufLen = 0
        elif(packetBufLen >= END_SYNC_BYTE_INDEX):
            if(packetBufLen >= PACKET_LEN):
                print("ERROR, packet not emptied???") #an error in base logic, that can be caused by memory errors (flipping bits), multicore trickery or really REALLY bad code
                packetBufLen = 0
            elif(serialData[i] == END_SYNC_BYTES[packetBufLen-END_SYNC_BYTE_INDEX]):
                packetBufLen += 1
                if(packetBufLen == PACKET_LEN):
                    parsedPackets[parsedPacketCount]=parsePacket(packetBuf)
                    parsedPacketCount += 1
                    packetBufLen = 0
            else:
                print("packet end sync failed after", packetBufLen-END_SYNC_BYTE_INDEX, "bytes")
                packetBufLen = 0
        else:
            packetBuf[packetBufLen-START_SYNC_BYTE_INDEX]=serialData[i]
            packetBufLen += 1
            if((packetBufLen == PACKET_LEN) and (END_SYNC_BYTE_INDEX == PACKET_LEN)): #if there are no end-sync-bytes
                parsedPackets[parsedPacketCount]=parsePacket(packetBuf)
                parsedPacketCount += 1
                packetBufLen = 0
    return(packetBuf, packetBufLen, parsedPackets, parsedPacketCount)

##can't be numba-fied because it uses a python list as input argument, but it's fine.
def FIFOwrite(value, fifoList, fifoMaxLength):
        """append a value to a FIFO array and delete the oldest value if needed"""
        fifoList.insert(0, value) #insert the new value at the start of the list
        while(len(fifoList) > fifoMaxLength): #(if) FIFO too long
            fifoList.pop(len(fifoList)-1) #delete the tail entry of the list

@vectorize([float64(int64), float32(int64)]) #make into a numpy ufunc
def intAngleToRad(intAngle: np.int64):
    return(np.deg2rad(np.float64(intAngle)/ANGLE_INTIFY_MULT))

@vectorize([float64(int64), float32(int64)]) #make into a numpy ufunc
def intDistToMeters(encoCount: np.int64):
    return(encoCount * ENCO_M_PER_COUNT)

print("procompiling carMCUclass functions...")
compileStartTime = time.time()
parsePacket(np.frombuffer(b'\x00\x00\x00\x00'+b'\x00\x00'+b'\x00\x00\x00\x00', dtype=np.uint8))
extractPackets(b'', np.array([], dtype=np.uint8), 0)
intAngleToRad(np.int64(-1234))
intDistToMeters(np.int64(1234))
print("carMCUclass compilation done! (took", round(time.time()-compileStartTime,1), "seconds)")


class carMCU:
    """a class for handling the serial connection to the real car"""
    ## constants:
    sendMinInterval = 0.049 #minimum time between sending things (to avoid spamming the poor carMCU)
    defaultGetFeedbackInterval = 0.005 #used in runOnThread()
    maxFIFOlength = 20 #can safely be changed at runtime (excess FIFO entries will be removed at next write-oppertunity (once the next datapoint comes in)), MUST BE AT LEAST 2
    def __init__(self, connectAtInit=True, comPort=None, autoFind=True, clockFunc=time.time): #if no clock function is supplied, time.time is used
        self.carMCUserial = serial.Serial()
        self.carMCUserial.baudrate = 115200
        self.carMCUserial.timeout = 0.01 #a 10ms timeout (should only be needed for readline(), which i don't use)
        self.carMCUserial.rts = False
        self.carMCUserial.dtr = False
        #carMCUserial.port = comPort #already done in connect()
        self.oldComPortList = [entry.name for entry in serial.tools.list_ports.comports()] #not super clean, but makes debugging easier for now
        if(connectAtInit):
            self.connect(comPort, autoFind) #attempt to connect upon class creation
        
        self.clockFunc = clockFunc #clock function, can just be time.time, can also be Map.clock
        
        self.lastSendTime = self.clockFunc() #timestamp of last sendSpeedAngle() (attempt)
        
        self.packetBuf = np.zeros(DATA_LEN, dtype=np.uint8) #used to remember partially-received packets between getFeedback() runs
        self.packetBufLen = 0 #packetBuf is initialized at full length (For numba & speed reasons), this indicates how much of that data is current. Any data beyond this index is old (not part of the current packet)
        self.clockZeroVal = np.int64(0) #will be set to the first value reported by the MCU. This just helps to sync with clockFunc() for larger data comparisons (and checking if the connection as failed)
        self.distZeroVal = np.int64(-1) #will be set to the first value reported by the MCU.
        
        # sensor feedback data in First In First Out buffers, entry [0] is the newest, entry[len()-1] is the oldest
        self.feedbackTimestampFIFO = []
        self.angleFIFO = []
        self.distTotalFIFO = [] #total sum of all distance feedbacks
    
    # def __del__(self):
    #     try:
    #         self.carMCUserial.close()
    #     except:
    #         print("couldn't close carMCU serial port from __del__")
    
    def _autoFindComPort(self):
        """attempt to find a new/singular COM-port (unplugging and replugging the serial device will make this function return that device)"""
        comPortList = [entry.name for entry in serial.tools.list_ports.comports()]
        print("autoFinding comPorts...")
        if(len(comPortList) < 1):
            print("no comPorts found, cant connect to car serial")
            self.oldComPortList = comPortList
            return(None, False)
        elif(len(comPortList) == 1):
            print("only 1 comPort found:", comPortList[0])
            comPort = comPortList[0]
            self.oldComPortList = comPortList
            return(comPort, True)
        else:
            print("several comPorts found...")
            changedEntries = []
            for entry in comPortList:
                if(entry not in self.oldComPortList):
                    print("new comPort found:", entry)
                    changedEntries.append(entry)
            if(len(changedEntries) == 1):
                print("returning:", changedEntries[0])
                self.oldComPortList = comPortList
                return(changedEntries[0], True)
            elif(len(changedEntries) > 1):
                print("multiple comPorts changed, so none will be selected. changed ports:")
                print(changedEntries)
                self.oldComPortList = comPortList
                return(None, False)
            else:
                print("no newly added comPorts found")
                for entry in self.oldComPortList:
                    if(entry not in comPortList):
                        changedEntries.append(entry)
                if(len(changedEntries) > 0):
                    print("newly removed ports:", changedEntries)
                self.oldComPortList = comPortList
                return(None, False)
    
    def _comportCheck(self, comPort, autoFind):
        """check whether or not a given COM-port name exists in the list of COM-ports (may cause issues in linux)"""
        comPortList = [entry.name for entry in serial.tools.list_ports.comports()]
        if(comPort is None):
            print("no carMCU comPort specified")
            if(autoFind):
                return(self._autoFindComPort())
            else:
                self.oldComPortList = comPortList
                return(None, False)
        else:
            if(comPort in comPortList):
                self.oldComPortList = comPortList
                return(comPort, True)
            elif((comPort.lstrip('/dev/') in comPortList) if comPort.startswith('/dev/') else False): #quick 'n dirty linux fix
                self.oldComPortList = comPortList
                return(comPort, True)
            else:
                print("bad comPort entered:", comPort)
                if(autoFind):
                    return(self._autoFindComPort())
                else:
                    self.oldComPortList = comPortList
                    return(None, False)
                
    
    def connect(self, comPort=None, autoFind=True, replaceIfConnected=False):
        """connect to a given (or an automatically found) COM-port, or replace an active connection with a new one"""
        if(self.carMCUserial.is_open):
            if(replaceIfConnected):
                if(not self.disconnect()):
                    print("couldn't replace connection")
                    return(self.carMCUserial.is_open)
            else:
                return(self.carMCUserial.is_open)
        comPort, goodPortFound = self._comportCheck(comPort, autoFind)
        if(goodPortFound):
            print("connecting to:", comPort)
            self.carMCUserial.port = comPort
            try:
                self.carMCUserial.open()
            except Exception as excep:
                print("couldn't open comPort with port:", self.carMCUserial.port)
                print("reason:", excep)
        return(self.carMCUserial.is_open)
    
    def disconnect(self):
        """disconnect from the current COM-port (if connected)"""
        if(self.carMCUserial.is_open):
            try:
                self.carMCUserial.close()
            except:
                print("couldn't close carMCU serial port")
                return(False)
        else:
            print("carMCU serial already closed")
        return(True)
    
    def sendSpeedAngle(self, speed, angle): #NOTE: 'angle' input for this function should be in radians
        """send a speed and steering angle to the carMCU"""
        rightNow = self.clockFunc()
        if(abs(angle) > self.maxSteeringAngle): #an extra little check
            print("can't sendSpeedAngle, steering angle too large:", np.rad2deg(angle))
        else:
            if((rightNow - self.lastSendTime) > self.sendMinInterval):
                self.lastSendTime = rightNow
                if(self.carMCUserial.is_open):
                    #convert floats to string ('\r' is not really needed)
                    dataString = str(round(float(speed), 2)) + ' ' + str(round(np.rad2deg(float(angle)), 1)) + '\n'
                    #NOTE: the (current) serial formatting uses angles in degrees (for more precision per char (unless you start multiplying&dividing by powers of 10))
                    #print("sending:",dataString.encode())
                    try:
                        self.carMCUserial.write(dataString.encode())
                    except:
                        print("carMCU serial write exception (or encode() exception)")
                else:
                    print("can't sendSpeedAngle(), carMCU is not connected")
            else:
                print("you're spamming sendSpeedAngle(), stop it")
    
    def _savePacketData(self, packet: np.ndarray):
        if((self.clockZeroVal == 0) and (self.distZeroVal == -1)): #if this is the first time data is received
            ##synchronize clocks:
            localMillisEquivalent = np.int64(self.clockFunc()*1000)
            self.clockZeroVal = packet[2]-localMillisEquivalent #to synchronize clock values, save the value: (packet[2]-self.clockZeroVal)/1000.0
            ##cut off starting distance
            self.distZeroVal = packet[0] #to get the distance traveled WHILE THIS PYTHON SCRIPT WAS RUNNING, save the value: intDistToMeters(packet[0]-self.distZeroVal)
            ##this last one is not needed, but maybe nice to have:
            self.lastSendTime = self.clockFunc()
        FIFOwrite((packet[2]-self.clockZeroVal)/1000.0, self.feedbackTimestampFIFO, self.maxFIFOlength)
        FIFOwrite(intAngleToRad(packet[1]), self.angleFIFO, self.maxFIFOlength)
        FIFOwrite(intDistToMeters(packet[0]-self.distZeroVal), self.distTotalFIFO, self.maxFIFOlength)
    
    def getFeedback(self): #run this function regularly
        """(run this function regularly) read serial data (sensor feedback) and parse if possible"""
        if(self.carMCUserial.is_open):
            if(self.carMCUserial.in_waiting > 0):
                debugVar = self.carMCUserial.in_waiting
                try:
                    receivedBytes = self.carMCUserial.read(self.carMCUserial.in_waiting)
                except:
                    print("couldnt read", debugVar, "=", self.carMCUserial.in_waiting, "bytes from carMCU serial")
                    return(False)
                self.packetBuf, self.packetBufLen, parsedPackets, parsedPacketCount = extractPackets(receivedBytes, self.packetBuf, self.packetBufLen) # a numba-fied function (fast) to parse (& store if leftover) serial bytes.
                for i in range(parsedPacketCount):
                    #if(parsedPackets[i][0] >= self.distZeroVal): #the simplest data integrity check possible
                    self._savePacketData(parsedPackets[i])
                return(parsedPacketCount)
            else: #no data to be read
                return(0)
        else:
            print("can't getFeedback(), carMCU is not connected")
            return(0)
    
    def runOnThread(self, threadKeepRunning, autoreconnect=False):
        """(run this on a thread) runs getFeedback() forever and handles exceptions (supports hotplugging serial devices)
            USAGE: enter a 1-sized list with a boolean (e.g.: 'keepRunning=[True]; runOnThread(keepRunning)'), 
            to stop the thread, set that boolean to False and then run thisThread.join()"""
        if(((type(threadKeepRunning[0]) is not bool) if (len(threadKeepRunning)==1) else True) if (type(threadKeepRunning) is list) else True): #bad usage checking
            print("bad usage of carMCU.runOnThread(), please enter a 1-sized list with a boolean (python pointer hack)")
            #raise
            return()
        #import threading as thr
        #myThread = thr.current_thread()
        while(threadKeepRunning[0]): #super robust, might cause CPU overload though
            try:
                while((not self.carMCUserial.is_open) and autoreconnect and threadKeepRunning[0]):
                    print("carMCU reconnecting with port:", self.carMCUserial.port)
                    self.connect(self.carMCUserial.port, False) #try to connect again with the same comPort
                    if(not self.carMCUserial.is_open):
                        time.sleep(0.5) #wait a bit between connection attempts
                while(self.carMCUserial.is_open and threadKeepRunning[0]):
                    self.getFeedback() #run this to get the data
                    time.sleep(self.defaultGetFeedbackInterval) #wait just a little bit, as to not needlessly overload the CPU
                if(not autoreconnect):
                    print("carMCU connection on thread stopped becuase is_open:", self.carMCUserial.is_open)
                    return()
                if(not threadKeepRunning[0]):
                    print("stopping carMCU runOnThread function...")
                    return()
            except serial.SerialException as excepVar:
                print("carMCU serial exception, dealing with it...")
                if(excepVar.args[0].find("Access is denied.")): #this is in the error message that happens when you unplug an active device
                    self.connect(self.carMCUserial.port, False, True) #try to connect again with the same comPort (will probably result in just disconnecting)
                time.sleep(0.25)
            except KeyboardInterrupt:
                print("keyboardInterrupt! attempting to close carMCU serial connection...")
                try:
                    self.disconnect()
                except:
                    print("couldn't disconnect")
                threadKeepRunning[0] = False
                return()
            except Exception as excepVar:
                print("carMCU other exception:", excepVar)
                try:
                    self.disconnect()
                except:
                    print("couldn't disconnect")
                threadKeepRunning[0] = False
                return()


#a TEMPORARY class, to be replaced by SLAM code (which uses real filtering and stuff)
class realCar(carMCU, Map.Car):
    """ a TEMPORARY class that uses carMCU sensor feedback to get car state (position, velocity, etc.)
        (overwrites Map.Car.update() and carMCU.runOnThread()) """
    def __init__(self, clockFunc, connectAtInit=True, comPort=None, autoFind=True):
        Map.Car.__init__(self, clockFunc)
        self.clockFunc = clockFunc
        carMCU.__init__(self, connectAtInit, comPort, autoFind, self.clockFunc)
        self.timeSinceLastUpdate = self.clockFunc()
        self.skippedUpdateCheckVar = 0.0
    
    def update(self, inputDt=0): #this update() overwrites the update() in Map.Car, but this doesnt use the dt argument (because timestamps from the FIFO are used). inputDT is to removed in a future version
        """(overwrites Map.Car.update()) update state (position, velocity, etc.) based ONLY on car sensor feedback"""
        if((self.feedbackTimestampFIFO[0] > self.timeSinceLastUpdate) if (len(self.feedbackTimestampFIFO)>1) else False): #if the newest entry ([0]) is newer than the last processed entry (and (len() > 1) because we want to have at least 2 datapoints)
            updates = 0 #ideally, you'd only be dealing with a single new datapoint
            for i in range(len(self.angleFIFO)):
                if(self.feedbackTimestampFIFO[i] > self.timeSinceLastUpdate):
                    updates += 1
            if(updates == len(self.angleFIFO)): #if this is True, it means there is no previously processed datapoint in the FIFOs, and data was (probably) lost (discarded)
                if((len(self.angleFIFO) > 2) or (self.maxFIFOlength == 2)): #dont report error if the program only just started, or if maxFIFOlength is only 1
                    print("!!! updateOverflow !!!:", updates)
                updates -= 1 #updates is equal to the length of the array, which is not a valid index (and doing -1 is the whole point of the overflow exception)
                dt = self.feedbackTimestampFIFO[updates]-self.timeSinceLastUpdate #old timestamp - new timestamp
                stepSteering = (self.steering + self.angleFIFO[updates])/2
                stepDist = self.distTotalFIFO[updates] - self.skippedUpdateCheckVar #little tricky, but should work
                stepVelocity = (self.velocity + (stepDist / dt))/2
                
                #turning math
                if((abs(stepSteering) > 0.001) and (abs(stepVelocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                    rearAxlePos = self.getRearAxlePos()
                    turning_radius = self.wheelbase/np.tan(stepSteering)
                    angular_velocity = self.velocity/turning_radius
                    arcMov = angular_velocity * dt
                    #one way to do it
                    turning_center = GF.distAnglePosToPos(turning_radius, self.angle+(np.pi/2), rearAxlePos) #get point around which car turns
                    rearAxlePos = GF.distAnglePosToPos(turning_radius, self.angle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long
                    #update position
                    self.angle += arcMov
                    self.position = GF.distAnglePosToPos(self.wheelbase/2, self.angle, rearAxlePos)
                else:
                    # self.position[0] += dt * stepVelocity * np.cos(self.angle)
                    # self.position[1] += dt * stepVelocity * np.sin(self.angle)
                    self.position[0] += stepDist * np.cos(self.angle)
                    self.position[1] += stepDist * np.sin(self.angle)
                self.skippedUpdateCheckVar += stepDist
            #regular forloop to get through
            for i in range(updates):
                dt = self.feedbackTimestampFIFO[updates-i-1]-self.feedbackTimestampFIFO[updates-i] #new timestamp - old timestamp
                stepSteering = self.angleFIFO[updates-i-1] #just take current angle (alternatively, you could take an average of several points)
                stepDist = self.distTotalFIFO[updates-i-1]-self.distTotalFIFO[updates-i] #distance traveled (wheel encoder difference)
                stepVelocity = stepDist / dt
                
                #turning math
                if((abs(stepSteering) > 0.001) and (abs(stepVelocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                    rearAxlePos = self.getRearAxlePos()
                    turning_radius = self.wheelbase/np.tan(stepSteering)
                    angular_velocity = stepVelocity/turning_radius
                    arcMov = angular_velocity * dt
                    #one way to do it
                    turning_center = GF.distAnglePosToPos(turning_radius, self.angle+(np.pi/2), rearAxlePos) #get point around which car turns
                    rearAxlePos = GF.distAnglePosToPos(turning_radius, self.angle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long
                    #update position
                    self.angle += arcMov
                    self.position = GF.distAnglePosToPos(self.wheelbase/2, self.angle, rearAxlePos)
                else:
                    # self.position[0] += dt * stepVelocity * np.cos(self.angle)
                    # self.position[1] += dt * stepVelocity * np.sin(self.angle)
                    self.position[0] += stepDist * np.cos(self.angle)
                    self.position[1] += stepDist * np.sin(self.angle)
                self.skippedUpdateCheckVar += stepDist
                if(abs(self.skippedUpdateCheckVar - self.distTotalFIFO[updates-i-1]) > 0.1): #both variables hold the total (summed up) distance traveled
                    print("you should run car.update() more often, because you are missing important data:", round(self.distTotalFIFO[updates-i-1]-self.skippedUpdateCheckVar, 2))
                    #print("traveled dist (car.update()):", self.skippedUpdateCheckVar)
                    #print("traveled dist (distTotalFIFO["+str(updates-i-1)+"]):", self.distTotalFIFO[updates-i-1])
                    self.skippedUpdateCheckVar = self.distTotalFIFO[updates-i-1]
            #self.velocity = (self.distTotalFIFO[0]-self.distTotalFIFO[1]) / (self.feedbackTimestampFIFO[0]-self.feedbackTimestampFIFO[1])
            self.velocity = stepVelocity #(only python allows you to take semi-initialized variables like this)
            self.steering = self.angleFIFO[0]
            self.timeSinceLastUpdate = self.feedbackTimestampFIFO[0]
            #print([round(self.position[0], 2), round(self.position[1], 2)], round(GF.radRoll(self.angle),2), round(self.velocity,2), round(np.rad2deg(self.steering),2), round(self.distTotalFIFO[0],2))
    
    def runOnThread(self, threadKeepRunning, autoreconnect=False): #overwrites runOnThread() in carMCU class
        """(run this on a thread) runs getFeedback() and update() forever and handles exceptions (supports hotplugging serial devices)
            USAGE: enter a 1-sized list with a boolean (e.g.: 'keepRunning=[True]; runOnThread(keepRunning)'), 
            to stop the thread, set that boolean to False and then run thisThread.join()"""
        if(((type(threadKeepRunning[0]) is not bool) if (len(threadKeepRunning)==1) else True) if (type(threadKeepRunning) is list) else True): #bad usage checking
            print("bad usage of carMCU.runOnThread(), please enter a 1-sized list with a boolean (python pointer hack)")
            #raise
            return()
        #import threading as thr
        #myThread = thr.current_thread()
        while(threadKeepRunning[0]): #super robust, might cause CPU overload though
            try:
                while((not self.carMCUserial.is_open) and autoreconnect and threadKeepRunning[0]):
                    print("carMCU reconnecting with port:", self.carMCUserial.port)
                    self.connect(self.carMCUserial.port, False) #try to connect again with the same comPort
                    if(not self.carMCUserial.is_open):
                        time.sleep(0.5) #wait a bit between connection attempts
                while(self.carMCUserial.is_open and threadKeepRunning[0]):
                    self.getFeedback() #get serial data and parse into values for the fifos
                    self.update()      #parse fifos to get position
                    time.sleep(self.defaultGetFeedbackInterval) #wait just a little bit, as to not needlessly overload the CPU
                if(not autoreconnect):
                    print("carMCU connection on thread stopped becuase is_open:", self.carMCUserial.is_open)
                    return()
                if(not threadKeepRunning[0]):
                    print("stopping carMCU runOnThread function...")
                    return()
            except serial.SerialException as excepVar:
                print("carMCU serial exception, dealing with it...")
                if(excepVar.args[0].find("Access is denied.")): #this is in the error message that happens when you unplug an active device
                    self.connect(self.carMCUserial.port, False, True) #try to connect again with the same comPort (will probably result in just disconnecting)
                time.sleep(0.25)
            except KeyboardInterrupt:
                print("keyboardInterrupt! attempting to close carMCU serial connection...")
                try:
                    self.disconnect()
                except:
                    print("couldn't disconnect")
                threadKeepRunning[0] = False
                return()
            except Exception as excepVar:
                print("carMCU other exception:", excepVar)
                try:
                    self.disconnect()
                except:
                    print("couldn't disconnect")
                threadKeepRunning[0] = False
                return()


# testing code, turn on a print() inside a function of interest (like car.update()) to see it working
if __name__ == '__main__':
    mapWithCar = Map() #map doesnt actually need to contain car object, it just needs to be initialized and have a .clock function
    someCar = realCar(mapWithCar.clock, True, 'COM5', False)
    keepRunning = [True]
    someCar.runOnThread(keepRunning) #not how it's meant to be used, for the record.
    someCar.disconnect()