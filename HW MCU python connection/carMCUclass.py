#usage: import this into another program, make an instance of the class,
#        either connect at class initialization, or later using connect() (comPorts are named things like 'COM4' on windows)
#        you can send instructions to the car using sendSpeedAngle()
#        you have to either run getFeedback() regularly (manually), or start runOnThread() on a dedicated thread
#        the sensor data received from the car is stored in several FIFO buffers (for better math/filtering) (see class below for buffer names
#        (for example) speedFIFO[0] is the most recent speed measurement data, it was received (actually parsed) at feedbackTimestampFIFO[0] (which stores time.time())


import serial
import serial.tools.list_ports
import time
import mapClassTemp as mp
import numpy as np

class carMCU:
    def __init__(self, connectAtInit=True, comPort=None, autoFind=True):
        self.carMCUserial = serial.Serial()
        self.carMCUserial.baudrate = 115200
        self.carMCUserial.timeout = 0.01 #a 10ms timeout (should only be needed for readline(), which i don't use)
        #carMCUserial.port = comPort #already done in connect()
        self.oldComPortList = [entry.name for entry in serial.tools.list_ports.comports()] #not super clean, but makes debugging easier for now
        if(connectAtInit):
            self.connect(comPort, autoFind) #attempt to connect upon class creation
        
        # constants
        self.minimumSerialLength = 14 #the minimum length of a feedback message is "x.xx x.xx x.x\r"
        self.unfinTimeout = 0.015 #if the unfinished message is older than this, just dump it (this value should be larger than the time between getFeedback() calls
        self.defaultGetFeedbackInterval = 0.005 #used in runOnThread()
        # variables
        self.unfinString = ''
        self.unfinTimestamp = time.time()
        
        # sensor feedback data in First In First Out buffers, entry [0] is the newest, entry[len()-1] is the oldest
        self.maxFIFOlength = 10 #can safely be changed at runtime (excess FIFO entries will be removed at next write-oppertunity (once the next datapoint comes in))
        self.feedbackTimestampFIFO = []
        self.speedFIFO = []
        self.distFIFO = []
        self.angleFIFO = []
        self.distTotalFIFO = [0]; #total sum of all distance feedbacks
    
    def _autoFindComPort(self):
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
            else:
                print("bad comPort entered:", comPort)
                if(autoFind):
                    return(self._autoFindComPort())
                else:
                    self.oldComPortList = comPortList
                    return(None, False)
                
    
    def connect(self, comPort=None, autoFind=True, replaceIfConnected=False):
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
            except:
                print("couldn't open comPort with port:", self.carMCUserial.port)
        return(self.carMCUserial.is_open)
    
    def disconnect(self):
        if(self.carMCUserial.is_open):
            try:
                self.carMCUserial.close()
            except:
                print("couldn't close carMCU serial port")
                return(False)
        else:
            print("carMCU serial already closed")
        return(True)
    
    def sendSpeedAngle(self, speed, angle): #TBD: angle unit (radian/degree) conversion, depending on which ones we choose to use
        if(self.carMCUserial.is_open):
            dataString = str(round(float(speed), 2)) + ' ' + str(round(float(angle), 1)) + '\n' #convert floats to string ('\r' is not really needed)
            #print("sending:",dataString.encode())
            try:
                self.carMCUserial.write(dataString.encode())
            except:
                print("carMCU serial write exception (or encode() exception)")
        else:
            print("can't sendSpeedAngle(), carMCU is not connected")
    
    def _FIFOwrite(self, value, fifoList, fifoMaxLength):
        fifoList.insert(0, value) #insert the new value at the start of the list
        while(len(fifoList) > fifoMaxLength): #(if) FIFO too long
            fifoList.pop(len(fifoList)-1) #delete the tail entry of the list
    
    def _parseSensorString(self, stringToParse):
        #print("parsing string:", stringToParse.encode())
        splitString = stringToParse.split(' ')
        if(len(splitString) != 3):
            print("len(splitString) wrong, can't parse string:", stringToParse.encode()) #encodet to make '\n', '\r' and ' ' visible
            return(False)
        self._FIFOwrite(time.time(), self.feedbackTimestampFIFO, self.maxFIFOlength)
        self._FIFOwrite(float(splitString[0]), self.speedFIFO, self.maxFIFOlength)
        self._FIFOwrite(float(splitString[1]), self.distFIFO, self.maxFIFOlength)
        self._FIFOwrite(float(splitString[2]), self.angleFIFO, self.maxFIFOlength)
        self._FIFOwrite(self.distTotalFIFO[0]+self.distFIFO[0], self.distTotalFIFO, self.maxFIFOlength)
        #print(self.speedFIFO[0], self.distFIFO[0], self.angleFIFO[0])
        return(True)
    
    def getFeedback(self): #run this function regularly
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
class realCar(carMCU, mp.Map.Car):
    def __init__(self, pos=[5,11], angle=0, connectAtInit=True, comPort=None, autoFind=True):
        mp.Map.Car.__init__(self, pos[0], pos[1], angle)
        carMCU.__init__(self, connectAtInit, comPort, autoFind)
        self.timeSinceLastUpdate = time.time()
        self.skippedUpdateCheckVar = 0.0
    
    def update(self): #this update() overwrites the update() in Map.Car, but this has no dt argument (because why would you do that)
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
                dt = self.timeSinceLastUpdate-self.feedbackTimestampFIFO[updates] #old timestamp - new timestamp
                stepVelocity = (self.velocity + self.speedFIFO[updates])/2
                stepSteering = (self.steering + self.angleFIFO[updates])/2
                stepDist = self.distTotalFIFO[updates] - self.skippedUpdateCheckVar #little tricky, but should work
                
                angularVelocity = 0 #init var
                if(abs(stepSteering) > 0.001): #avoid divide by 0
                    turningRadius = self.length / np.sin(stepSteering)
                    angularVelocity = stepVelocity / turningRadius #(rework math for rotation around actual point of rotation?)
                self.angle += (dt * angularVelocity) / 2 #half now...
                self.position[0] += stepDist * np.cos(self.angle) #x
                self.position[1] += stepDist * np.sin(self.angle) #y
                self.angle += (dt * angularVelocity) / 2 #second half of angle change is added after linear movement is calulated, to (hopefully) increase accuracy slightly
                self.skippedUpdateCheckVar += stepDist
            #regular forloop to get through
            for i in range(updates):
                dt = self.feedbackTimestampFIFO[updates-i]-self.feedbackTimestampFIFO[updates-i-1] #old timestamp - new timestamp
                stepVelocity = (self.speedFIFO[updates-i]+self.speedFIFO[updates-i-1])/2 #idk, average speed between datapoints i guess
                stepSteering = (self.angleFIFO[updates-i]+self.angleFIFO[updates-i-1])/2 #idk, average steering angle between datapoints i guess
                stepDist = self.distFIFO[updates-i-1] #distance traveled (wheel encoder difference)
                
                angularVelocity = 0 #init var
                if(abs(stepSteering) > 0.001): #avoid divide by 0
                    turningRadius = self.length / np.sin(stepSteering)
                    angularVelocity = stepVelocity / turningRadius #(rework math for rotation around actual point of rotation?)
                self.angle += (dt * angularVelocity) / 2 #half now...
                self.position[0] += stepDist * np.cos(self.angle) #x
                self.position[1] += stepDist * np.sin(self.angle) #y
                self.angle += (dt * angularVelocity) / 2 #second half of angle change is added after linear movement is calulated, to (hopefully) increase accuracy slightly
                self.skippedUpdateCheckVar += stepDist
                if(abs(self.skippedUpdateCheckVar - self.distTotalFIFO[updates-i-1]) > 0.1): #both variables hold the total (summed up) distance traveled
                    print("you should run car.update() more often, because you are missing important data")
                    print("traveled dist (car.update()):", self.skippedUpdateCheckVar)
                    print("traveled dist (distTotalFIFO["+str(updates-i-1)+"]):", self.distTotalFIFO[updates-i-1])
                    self.skippedUpdateCheckVar = self.distTotalFIFO[updates-i-1]
            self.velocity = self.speedFIFO[0] #just take the most recent velocity
            self.timeSinceLastUpdate = self.feedbackTimestampFIFO[0]
            #print([round(self.position[0], 2), round(self.position[1], 2)], round(self.angle,2), round(self.velocity,2), round(self.skippedUpdateCheckVar,2))
    
    def runOnThread(self, autoreconnect=False): #overwrites runOnThread() in carMCU class
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
                    self.update()
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


## testing code, turn on a print() inside a function of interest (like car.update()) to see it working
# someCar = realCar(comPort='COM5')
# someCar.runOnThread()