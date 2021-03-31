#usage: import this into another program, make an instance of the class,
#        either connect at class initialization, or later using connect() (comPorts are named things like 'COM4' on windows)
#        you can send instructions to the car using sendSpeedAngle()
#        you have to either run getFeedback() regularly (manually), or start runOnThread() on a dedicated thread
#        the sensor data received from the car is stored in several FIFO buffers (for better math/filtering) (see class below for buffer names
#        (for example) speedFIFO[0] is the most recent speed measurement data, it was received (actually parsed) at feedbackTimestampFIFO[0] (which stores time.time())


import serial
import serial.tools.list_ports
import time
import numpy as np

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use

class carMCU:
    """a class for handling the serial connection to the real car"""
    def __init__(self, connectAtInit=True, comPort=None, autoFind=True):
        self.carMCUserial = serial.Serial()
        self.carMCUserial.baudrate = 115200
        self.carMCUserial.timeout = 0.01 #a 10ms timeout (should only be needed for readline(), which i don't use)
        self.carMCUserial.rts = False
        self.carMCUserial.dtr = False
        #carMCUserial.port = comPort #already done in connect()
        self.oldComPortList = [entry.name for entry in serial.tools.list_ports.comports()] #not super clean, but makes debugging easier for now
        if(connectAtInit):
            self.connect(comPort, autoFind) #attempt to connect upon class creation
        
        self.maxSteeringAngle = np.deg2rad(25) #maybe this should be Map.Car?
        
        # constants
        self.minimumSerialLength = 14 #the minimum length of a feedback message is "x.xx x.xx x.x\r"
        self.unfinTimeout = 0.015 #if the unfinished message is older than this, just dump it (this value should be larger than the time between getFeedback() calls
        self.sendMinInterval = 0.09 #minimum time between sending things (to avoid spamming the poor carMCU)
        self.defaultGetFeedbackInterval = 0.005 #used in runOnThread()
        # variables
        self.unfinString = '' #data may come in unfinished, store that unfinished data here (untill the rest is found)
        self.unfinTimestamp = time.time() #a timestamp to remember how old the last unfinished message is (if it's too old, discard it)
        self.lastSendTime = time.time() #timestamp of last sendSpeedAngle() (attempt)
        
        # sensor feedback data in First In First Out buffers, entry [0] is the newest, entry[len()-1] is the oldest
        self.maxFIFOlength = 10 #can safely be changed at runtime (excess FIFO entries will be removed at next write-oppertunity (once the next datapoint comes in))
        self.feedbackTimestampFIFO = []
        self.speedFIFO = []
        self.distFIFO = []
        self.angleFIFO = []
        self.distTotalFIFO = [0]; #total sum of all distance feedbacks
    
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
        rightNow = time.time()
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
    
    def _FIFOwrite(self, value, fifoList, fifoMaxLength):
        """append a value to a FIFO array and delete the oldest value if needed"""
        fifoList.insert(0, value) #insert the new value at the start of the list
        while(len(fifoList) > fifoMaxLength): #(if) FIFO too long
            fifoList.pop(len(fifoList)-1) #delete the tail entry of the list
    
    def _parseSensorString(self, stringToParse):
        """parse (decode) a string of sensor feedback from the carMCU"""
        #print("parsing string:", stringToParse.encode())
        splitString = stringToParse.split(' ')
        if(len(splitString) != 3):
            print("len(splitString) wrong, can't parse string:", stringToParse.encode()) #encodet to make '\n', '\r' and ' ' visible
            return(False)
        self._FIFOwrite(time.time(), self.feedbackTimestampFIFO, self.maxFIFOlength)
        self._FIFOwrite(float(splitString[0]), self.speedFIFO, self.maxFIFOlength)
        self._FIFOwrite(float(splitString[1]), self.distFIFO, self.maxFIFOlength)
        self._FIFOwrite(np.deg2rad(float(splitString[2])), self.angleFIFO, self.maxFIFOlength)
        self._FIFOwrite(self.distTotalFIFO[0]+self.distFIFO[0], self.distTotalFIFO, self.maxFIFOlength)
        #print(self.speedFIFO[0], self.distFIFO[0], self.angleFIFO[0])
        return(True)
    
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
                try:
                    receivedString = receivedBytes.decode()
                except:
                    print("couldnt decode receivedBytes:", receivedBytes)
                    print("wrong baud rate???")
                    return(False)
                #print("received:", receivedString.encode()) #use encode() becuase otherwise print() will use '\n' itself
                #splitData = [entry.strip('\r') for entry in receivedString.split('\n')] #split data and remove '\r'
                splitData = receivedString.split('\n')
                # special case (unfinished data from (recent) last message)
                if(((len(self.unfinString)+len(receivedString)) < (2*self.minimumSerialLength)) and \
                   (len(self.unfinString) > 0) and ((time.time() - self.unfinTimestamp) < self.unfinTimeout)): #if there's unfinished data
                    self.unfinString += splitData[0]
                    #print("appending unfinString (top):", self.unfinString.encode(), "after", int((time.time() - self.unfinTimestamp)*1000), "ms")
                    if((len(self.unfinString) >= self.minimumSerialLength) and (self.unfinString.endswith('\r'))):
                        while(self.unfinString.count('\r') > 1): #only in case of a skipped '\n'
                            print("multiple '\r' in unfinString:", self.unfinString.encode())
                            self.unfinString = self.unfinString[(self.unfinString.index('\r')+1)::] #only keep everything after the first '\r', for the next time
                        result = self._parseSensorString(self.unfinString.strip('\r'))
                        if(result):
                            self.unfinString = ''
                        if(len(splitData) > 1):
                            if((len(splitData) > 2) or (len(splitData[1]) > 0)): #if there is still another potentially valid message after the current half-one
                                print("rare serial double-half-parse, consider slowing down carMCU feedback frequency or increasing getFeedback() run frequency")    
                                splitData.pop(0)
                                # and now just let the normal parser sort out what's in splitData[1::]
                            else:
                                return(result)
                        else:
                            return(result)
                    else:
                        if(self.unfinString.count('\r') > 0): #if it contains a '\r', but was rejected earlier, something has gone terribly wrong
                            print("unfinished string contains carriage return, but badly:", self.unfinString.encode())
                            self.unfinString = self.unfinString[(self.unfinString.index('\r')+1)::] #only keep everything after the '\r', for the next time
                        # unfinished string is STILL not ready to be parsed, even after appending to it
                        return(False)
                # normal situation
                for i in range(len(splitData)):
                    # we'll want to scroll through array from back to front, becuase the last message was sent most recently (and we want to dump any old crap)
                    if(len(splitData[len(splitData)-i-1]) > 0):
                        if((len(splitData[len(splitData)-i-1]) >= self.minimumSerialLength) and (splitData[len(splitData)-i-1].endswith('\r'))):
                            result = self._parseSensorString(splitData[len(splitData)-i-1].strip('\r'))
                            self.unfinString = ''
                            return(result)
                        else:
                            self.unfinString = splitData[len(splitData)-i-1]
                            self.unfinTimestamp = time.time()
                            #print("setting unfinString:", self.unfinString.encode())
                # if the code gets here, it means no valid datapoints were found in splitData
                #print("no valid datapoints in splitData, unfinString:", self.unfinString.encode())
                return(False)
            else: #no data to be read
                return(False)
        else:
            print("can't getFeedback(), carMCU is not connected")
            return(False)
    
    def runOnThread(self, autoreconnect=False):
        """(run this on a thread) runs getFeedback() forever and handles exceptions (supports hotplugging serial devices)"""
        carMCUconThreadkeepRunning = True
        while(carMCUconThreadkeepRunning): #super robust, might cause CPU overload though
            try:
                while((not self.carMCUserial.is_open) and autoreconnect):
                    print("carMCU reconnecting with port:", self.carMCUserial.port)
                    self.connect(self.carMCUserial.port, False) #try to connect again with the same comPort
                    if(not self.carMCUserial.is_open):
                        time.sleep(0.5) #wait a bit between connection attempts
                while(self.carMCUserial.is_open):
                    self.getFeedback() #run this to get the data
                    time.sleep(self.defaultGetFeedbackInterval) #wait just a little bit, as to not needlessly overload the CPU
                if(not autoreconnect):
                    print("carMCU connection on thread stopped becuase is_open:", self.carMCUserial.is_open)
                    return()
            except serial.SerialException as excepVar:
                print("carMCU serial exception")
                if(excepVar.args[0].find("Access is denied.")): #this is in the error message that happens when you unplug an active device
                    self.connect(self.carMCUserial.port, False, True) #try to connect again with the same comPort (will probably result in just disconnecting)
                time.sleep(0.25)
            except KeyboardInterrupt:
                print("attempting to close carMCU serial connection...")
                self.disconnect()
                carMCUconThreadkeepRunning = False
                return()
            except Exception as excepVar:
                print("carMCU other exception:", excepVar)
                carMCUconThreadkeepRunning = False
                return()


#a TEMPORARY class, to be replaced by SLAM code (which uses real filtering and stuff)
class realCar(carMCU, Map.Car):
    """ a TEMPORARY class that uses carMCU sensor feedback to get car state (position, velocity, etc.)
        (overwrites Map.Car.update() and carMCU.runOnThread()) """
    def __init__(self, pos=[0,0], angle=0, connectAtInit=True, comPort=None, autoFind=True):
        Map.Car.__init__(self, pos, angle)
        carMCU.__init__(self, connectAtInit, comPort, autoFind)
        self.timeSinceLastUpdate = time.time()
        self.skippedUpdateCheckVar = 0.0
    
    def update(self, inputDt=0): #this update() overwrites the update() in Map.Car, but this doesnt use the dt argument (because timestamps from the FIFO are used)
        """(overwrites Map.Car.update()) update state (position, velocity, etc.) based ONLY on car sensor feedback"""
        if((len(self.speedFIFO) > 0) and (self.feedbackTimestampFIFO[0] > self.timeSinceLastUpdate)): #if the newest entry ([0]) is newer than the last processed entry
            updates = 0 #ideally, you'd only be dealing with a single new datapoint
            for i in range(len(self.speedFIFO)):
                if(self.feedbackTimestampFIFO[i] > self.timeSinceLastUpdate):
                    updates += 1
            updateOverflow = (updates == len(self.speedFIFO)) #if this is True, it means there is no previously processed datapoint in the FIFOs
            if(updateOverflow):
                if(len(self.speedFIFO) > 1): #dont report error if the program only just started, or if maxFIFOlength is only 1
                    print("!!! updateOverflow !!!:", updates)
                updates -= 1 #updates is equal to the length of the array, which is not a valid index (and doing -1 is the whole point of the overflow exception)
                dt = self.feedbackTimestampFIFO[updates]-self.timeSinceLastUpdate #old timestamp - new timestamp
                stepVelocity = (self.velocity + self.speedFIFO[updates])/2
                stepSteering = (self.steering + self.angleFIFO[updates])/2
                stepDist = self.distTotalFIFO[updates] - self.skippedUpdateCheckVar #little tricky, but should work
                
                #turning math
                if((abs(stepSteering) > 0.001) and (abs(stepVelocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                    rearAxlePos = GF.distAnglePosToPos(self.wheelbase/2, GF.radInv(self.angle), self.position)
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
                    self.position[0] += dt * stepVelocity * np.cos(self.angle)
                    self.position[1] += dt * stepVelocity * np.sin(self.angle)
                    # self.position[0] += stepDist * np.cos(self.angle)
                    # self.position[1] += stepDist * np.sin(self.angle)
                self.skippedUpdateCheckVar += stepDist
            #regular forloop to get through
            for i in range(updates):
                dt = self.feedbackTimestampFIFO[updates-i-1]-self.feedbackTimestampFIFO[updates-i] #old timestamp - new timestamp
                stepVelocity = (self.speedFIFO[updates-i-1]+self.speedFIFO[updates-i])/2 #idk, average speed between datapoints i guess
                stepSteering = (self.angleFIFO[updates-i-1]+self.angleFIFO[updates-i])/2 #idk, average steering angle between datapoints i guess
                stepDist = self.distFIFO[updates-i-1] #distance traveled (wheel encoder difference)
                
                #turning math
                if((abs(stepSteering) > 0.001) and (abs(stepVelocity) > 0.001)): #avoid divide by 0 (and avoid complicated math in a simple situation)
                    rearAxlePos = GF.distAnglePosToPos(self.wheelbase/2, GF.radInv(self.angle), self.position)
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
                    self.position[0] += dt * stepVelocity * np.cos(self.angle)
                    self.position[1] += dt * stepVelocity * np.sin(self.angle)
                    # self.position[0] += stepDist * np.cos(self.angle)
                    # self.position[1] += stepDist * np.sin(self.angle)
                self.skippedUpdateCheckVar += stepDist
                if(abs(self.skippedUpdateCheckVar - self.distTotalFIFO[updates-i-1]) > 0.1): #both variables hold the total (summed up) distance traveled
                    print("you should run car.update() more often, because you are missing important data")
                    print("traveled dist (car.update()):", self.skippedUpdateCheckVar)
                    print("traveled dist (distTotalFIFO["+str(updates-i-1)+"]):", self.distTotalFIFO[updates-i-1])
                    self.skippedUpdateCheckVar = self.distTotalFIFO[updates-i-1]
            #self.velocity = self.speedFIFO[0] #just take the most recent velocity
            self.velocity = stepVelocity
            self.steering = stepSteering
            self.timeSinceLastUpdate = self.feedbackTimestampFIFO[0]
            #print([round(self.position[0], 2), round(self.position[1], 2)], round(GF.radRoll(self.angle),2), round(self.velocity,2), round(self.skippedUpdateCheckVar,2))
    
    def runOnThread(self, autoreconnect=False): #overwrites runOnThread() in carMCU class
        """(run this on a thread) runs getFeedback() and update() forever and handles exceptions (supports hotplugging serial devices)"""
        carMCUconThreadkeepRunning = True
        while(carMCUconThreadkeepRunning): #super robust, might cause CPU overload though
            try:
                while((not self.carMCUserial.is_open) and autoreconnect):
                    print("carMCU reconnecting with port:", self.carMCUserial.port)
                    self.connect(self.carMCUserial.port, False) #try to connect again with the same comPort
                    if(not self.carMCUserial.is_open):
                        time.sleep(0.5) #wait a bit between connection attempts
                while(self.carMCUserial.is_open):
                    self.getFeedback() #get serial data and parse into values for the fifos
                    self.update()      #parse fifos to get position
                    time.sleep(self.defaultGetFeedbackInterval) #wait just a little bit, as to not needlessly overload the CPU
                if(not autoreconnect):
                    print("carMCU connection on thread stopped becuase is_open:", self.carMCUserial.is_open)
                    return()
            except serial.SerialException as excepVar:
                print("carMCU serial exception")
                if(excepVar.args[0].find("Access is denied.")): #this is in the error message that happens when you unplug an active device
                    self.connect(self.carMCUserial.port, False, True) #try to connect again with the same comPort (will probably result in just disconnecting)
                time.sleep(0.25)
            except KeyboardInterrupt:
                print("attempting to close carMCU serial connection...")
                self.disconnect()
                carMCUconThreadkeepRunning = False
                return()
            except Exception as excepVar:
                print("carMCU other exception:", excepVar)
                carMCUconThreadkeepRunning = False
                return()


# testing code, turn on a print() inside a function of interest (like car.update()) to see it working
if __name__ == '__main__':
    someCar = realCar(comPort='COM8')
    someCar.runOnThread()