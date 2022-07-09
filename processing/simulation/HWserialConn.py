## todo: make sure float positions have the correct unit (MM -> meters)
# fuction comments """ """ for every function!
# easier/better support for  'saveMeasurementOrigin' (arduino lidar define)
# a little automatic loop autoFinding comports (which adds already-opened ports to the exclusionList)

import serial
import serial.tools.list_ports
import time
import numpy as np

## handshake communication constants:
CONN_COMMON_BAUD = 115200
INVALID_HANDSHAKE_IDENTIFIER = 0x00
PC_HANDSHAKE_IDENTIFIER = 0x01
KART_MCU_HANDSHAKE_IDENTIFIER = 0x02
LIDAR_HANDSHAKE_IDENTIFIERS = (0x03, 0x04)
HANDSHAKE_SYNC_BYTES = b'\x5a\xa5'  # used for all handshake packets

## lidar communication constants:
LIDAR_PACKET_SYNC_BYTES = b'\x66\x99'  # used for both requests and packet descriptors
LIDAR_REQUEST_INVALID = 0x00  # the request command was not recognised
LIDAR_REQUEST_CONEDATA = 0x01  # (default) request cone positions
LIDAR_REQUEST_SET_MAX_RANGE = 0x02  # request to set a new max range (see request payload)
# LIDAR_REQUEST_SET_POS_OFFSET = 0x03 # request to set a new position offset (offset from the car center). TBD(?)

## kartMCU communication constants:
KART_MCU_PACKET_SYNC_BYTES = b'\xf0\x0f'
KART_MCU_REQUEST_INVALID = 0x00  # the request command was not recognised
KART_MCU_REQUEST_UPDATE = 0x01  # (default) request sensor feedback (and supply target speed & steering as part of the request)
KART_MCU_REQUEST_UPDATE_RAW = 0x02  # (alt to default) request raw sensor feedback (and supply (raw!) target speed & steering as part of the request)
KART_MCU_REQUEST_STEER_MOTOR_EN = 0x03  # request to enable/disable the steering motor (so a human can steer it)
KART_MCU_REQUEST_PEDAL_EN = 0x04  # request to enable/disable the throttle-pedal passthrough (so a human can throttle it)
# KART_MCU_REQUEST_SET_

## general ESP32 related constants
ESP_RESET_TIME = 0.01  # how long to keep RTS high (when attempting to reset the ESP)
ESP_POST_RESET_TIME = 0.25  # how long to wait after resetting an ESP

## communication structs:
connHandshake = np.dtype([('identifier', np.uint8),
                          ('timestamp', np.uint32),
                          ('checksum', np.uint8)])

## lidar-specific communication structs:
LIDARconnRequest = np.dtype([('requestCMD', np.uint8),
                             ('requestPayload', np.uint8, (4,)),
                             # use np.frombuffer(obj['requestPayload'][0:2], dtype=np.uint16) to cast to a larger reference
                             ('carPosX', np.float32),
                             ('carPosY', np.float32),
                             ('carOrient', np.float32),
                             ('checksum', np.uint8)])

LIDARpacketDescriptor = np.dtype([('requestAck', np.uint8),
                                  ('payloadCount', np.uint8),
                                  ('payloadType', np.uint16),
                                  ('checksum', np.uint8)])
## lidar payload types
maxBlobPointCount = 30
blobType = np.dtype([('timestamp', np.uint32),
                     ('appendTimestamp', np.uint32),
                     ('points', np.float32, (maxBlobPointCount, 2)),
                     # ('origins', np.float32, (maxBlobPointCount, 2)),
                     # ('originAngles', np.float32. (maxBlobPointCount)),
                     ('lines', np.float32, (maxBlobPointCount - 1, 2)),
                     ('pointCount', np.uint8)])
#  ('padding', np.uint8, (3,))]) # not needed if the arduino struct has __attribute__((packed))

conePosType = np.dtype([('timestamp', np.uint32),
                        ('appendTimestamp', np.uint32),
                        ('pos', np.float32, (2,)),
                        # ('origin', np.float32, (2,)),
                        # ('originAngle', np.float32),
                        ('inaccuracy', np.float32),
                        ('pointCount', np.uint8),
                        ('CRCbyte', np.uint8)])
# ('padding', np.uint8, (2,))]) # not needed if the arduino struct has __attribute__((packed))

## kartMCU-specific communication structs:
kartMCUdataRaw = np.dtype(
    [('steerAngle', np.int32),  # the raw steering encoder value (so you can do conversion math in python)
     ('encoders', np.uint32, (4,)),  # the raw wheel encoder values (so you can do conversion math in python)
     ('throttle', np.uint8),
     ('checksum', np.uint8)])
kartMCUdata = np.dtype([('steerAngle', np.float32),  # (radians) steering wheel angle
                        ('encoders', np.float32, (4,)),  # (meters) distance each wheel has travelled since boot
                        ('throttle', np.uint8),
                        ('checksum', np.uint8)])
kartMCUconnRequest = np.dtype([('requestCMD', np.uint8),
                               ('requestPayload', np.uint8, (4,)),
                               ('targetSteer', np.float32),
                               # NOTE: contains either int32 (for raw data) OR float32 (for default data).
                               ('targetSpeed', np.float32),
                               # always a float32 (even when using raw data for everything else)
                               ('checksum', np.uint8)])


def _calcChecksum_connStruct(obj):
    checksumVal = np.uint8(0)
    for _data in obj.tobytes()[:-1]:  # the last byte is the checksum byte. If you include it, you should end up with 0
        checksumVal ^= _data
    return (checksumVal)


# def _calcChecksum_conePosType(obj): # not needed since using __attribute__((packed))
#     checksumVal = np.uint8(0)
#     print(dir(obj))
#     for _data in obj.tobytes()[:-3]:
#         checksumVal ^= _data
#     return(checksumVal)


class handshakeESPserial:
    """a class for handling the serial connection to ESP32's on the kart (kartMCU, lidar, other(?))
        uses a constant handshake protocol (must be present on all counterparts),
         the rest of the communication is for you to decide ;)"""
    ownHandshakeIdentifier = PC_HANDSHAKE_IDENTIFIER  # static property

    def __init__(self):
        self._serial = serial.Serial()
        self._serial.baudrate = CONN_COMMON_BAUD
        self._serial.timeout = 0.005  # a 5ms timeout. is used whenever you use _serial.read() (at least if you're not assured by the number of bytes .in_waiting)
        self._serial.rts = 0
        self._serial.dtr = 0
        # _serial.port = comPort #already done in connect()
        self.oldComPortList = [entry.name for entry in
                               serial.tools.list_ports.comports()]  # not super clean, but makes debugging easier for now

        self.handshakeLocalTimestamp = 0.0  # set when doing handshake
        self.handshakeReceivedTimestamp = np.uint32(0)  # set when doing handshake
        self.handshakeReceivedIdentifier = INVALID_HANDSHAKE_IDENTIFIER
        self.handshake_done = False

        self.DEBUG_ignoredSerialData = bytearray(b'')
        bytearray()

    @property
    def is_open(self):  # just pass through values from _serial
        """macro to ._serial.is_open"""
        return (self._serial.is_open)

    @property
    def comPort(self):  # just pass through values from _serial (under a slightly more specific name)
        """macro to ._serial.port"""
        return (self._serial.port)

    def __del__(self):  # doesn't seem to work very well
        try:
            self._serial.close()
            # print("closed serial port from __del__")
        except:
            print("couldn't close serial port from __del__")

    def _autoFindComPort(self, tryAny=True, exclusionList=[], printDebug=True):
        """attempt to find a new/singular COM-port (unplugging and replugging the serial device will make this function return that device)"""
        comPortList = [entry.name for entry in serial.tools.list_ports.comports()]
        if (printDebug):
            print("autoFinding comPorts...")
        if (len(comPortList) < 1):
            if (printDebug):
                print("no comPorts found, cant connect to car serial")
            self.oldComPortList = comPortList
            return (None, False)
        elif (len(comPortList) == 1):
            if (printDebug):
                print("only 1 comPort found:", comPortList[0])
            comPort = comPortList[0]
            self.oldComPortList = comPortList
            if (comPort in exclusionList):
                if (printDebug):
                    print("unfortunately, that 1 comPort is on the exclusionList:", exclusionList)
                return (None, False)
            return (comPort, True)
        else:
            if (printDebug):
                print("several comPorts found...")
            changedEntries = []
            for entry in comPortList:
                if (entry not in self.oldComPortList):
                    if (printDebug):
                        print("new comPort found:", entry)
                    changedEntries.append(entry)
            self.oldComPortList = comPortList
            if (len(changedEntries) == 1):
                if (printDebug):
                    print("found 1 changed comPort:", changedEntries[0])
                if (changedEntries[0] in exclusionList):
                    if (printDebug):
                        print("unfortunately, that 1 changed comPort is on the exclusionList:", exclusionList)
                    return (None, False)
                return (changedEntries[0], True)
            elif (len(changedEntries) > 1):
                if (printDebug):
                    print("multiple comPorts changed...")
                nonExcludedPorts = []
                for entry in changedEntries:
                    if (entry not in exclusionList):
                        nonExcludedPorts.append(entry)
                if (len(nonExcludedPorts) == 0):
                    if (printDebug):
                        print("all changed comPorts appeared to be on the exclusionList though:", changedEntries,
                              exclusionList)
                    return (None, False)
                elif (len(nonExcludedPorts) == 1):
                    if (printDebug):
                        print("found 1 comPort that is NOT on the exclusionList:", nonExcludedPorts[0])
                    return (nonExcludedPorts[0], True)
                else:
                    if (tryAny):
                        if (printDebug):
                            print("just gonna try:", changedEntries[0])
                        return (changedEntries[0], True)
                    ## if there is truly no right decision:
                    if (printDebug):
                        print("too many changed, so none will be selected. changed ports:", changedEntries)
                    return (None, False)
            else:  # changedEntries is empty
                if (tryAny):  # a very direct trial and error approach
                    for entry in comPortList:
                        if (entry not in exclusionList):
                            if (printDebug):
                                print("just gonna try comPort:", entry)
                            return (entry, True)
                ## if there is truly no right decision:
                if (printDebug):
                    print("no newly added comPorts found")
                return (None, False)

    def _comportCheck(self, comPort, autoFind, tryAny=False, exclusionList=[], printDebug=True):
        """check whether or not a given COM-port name exists in the list of COM-ports (may cause issues in linux)"""
        comPortList = [entry.name for entry in serial.tools.list_ports.comports()]
        if (comPort is None):
            if (printDebug):
                print("no comPort specified")
            if (autoFind):
                return (self._autoFindComPort(tryAny, exclusionList, printDebug))
            else:
                self.oldComPortList = comPortList
                return (None, False)
        else:
            if (comPort in comPortList):
                self.oldComPortList = comPortList
                return (comPort, True)
            elif ((comPort.lstrip('/dev/') in comPortList) if comPort.startswith(
                    '/dev/') else False):  # quick 'n dirty linux fix
                self.oldComPortList = comPortList
                return (comPort, True)
            else:
                if (printDebug):
                    print("bad comPort entered:", comPort)
                if (autoFind):
                    return (self._autoFindComPort(tryAny, exclusionList, printDebug))
                else:
                    self.oldComPortList = comPortList
                    return (None, False)

    def connect(self, comPort=None, autoFind=True, replaceIfConnected=False, tryAny=False, exclusionList=[],
                printDebug=True):
        """connect to a given (or an automatically found) COM-port, or replace an active connection with a new one"""
        # if(printDebug):
        #     print("connecting to:", comPort, "current connection:", self._serial.port)
        if (self._serial.is_open):
            if (replaceIfConnected):
                if (not self.disconnect(printDebug)):
                    if (printDebug):
                        print("couldn't replace connection")
                    return (self._serial.is_open)
                if (printDebug):
                    print("replacing", self._serial.port, "with", comPort, " debug check:",
                          comPort != self._serial.port)
                if ((self._serial.port is not None) and (
                        comPort != self._serial.port)):  # technically not needed, but it's nice to make sure anyway
                    exclusionList.append(self._serial.port)
            else:
                return (self._serial.is_open)
        comPort, goodPortFound = self._comportCheck(comPort, autoFind, tryAny, exclusionList,
                                                    printDebug)  # checks existing ports or autoFinds a new one
        if (goodPortFound):
            if (printDebug):
                print("connecting to:", comPort)
            self._serial.port = comPort
            try:
                self._serial.open()
            except Exception as excep:
                # if(printDebug):  # try-except debug is too critical to block
                print("couldn't open comPort with port:", self._serial.port)
                print("reason:", excep)
        return (self._serial.is_open)

    def disconnect(self, printDebug=True):
        """disconnect from the current COM-port (if connected)"""
        if (self._serial.is_open):
            try:
                self._serial.close()
            except Exception as excep:
                print("couldn't close serial port:", excep)
                return (False)
        elif (printDebug):
            print("serial already closed")
        return (True)

    def _resetESP(self):
        self._serial.setRTS(1)
        time.sleep(ESP_RESET_TIME)
        self._serial.setRTS(0)
        time.sleep(ESP_POST_RESET_TIME)

    def DEBUG_checkIgnoredSerialData(self):  # TODO: fancier print
        if (len(self.DEBUG_ignoredSerialData) > 0):
            print("DEBUG_checkIgnoredSerialData:", self.DEBUG_ignoredSerialData)
            self.DEBUG_ignoredSerialData.clear()

    def sendHandshake(self, handshakeToSend: connHandshake):
        if (not self._serial.is_open):
            print("can't sendHandshake(), serial port is not open!")
        handshakeToSend['checksum'] = _calcChecksum_connStruct(
            handshakeToSend)  # perpare for transmission by calculating and storing checksum
        self._serial.write(HANDSHAKE_SYNC_BYTES)  # send sync bytes
        self._serial.write(handshakeToSend.tobytes())  # send handshake bytes

    def waitForSyncBytes(self, syncBytes, timeout, printDebug=True):
        if (not self._serial.is_open):
            print("can't waitForSyncBytes(), serial port is not open!")
        startTime = time.time()
        syncProgress = 0
        while (((time.time() - startTime) < timeout) and (syncProgress < len(syncBytes)) and self._serial.is_open):
            while (self._serial.in_waiting and (syncProgress < len(
                    syncBytes))):  # making this a while-loop ensures the timeout won't be triggered halfway through a successfull sync
                potentialSyncByte = self._serial.read(1)[
                    0]  # read() returns a byte array (even when there's only 1 entry)
                if (potentialSyncByte == syncBytes[syncProgress]):
                    syncProgress += 1
                else:
                    self.DEBUG_ignoredSerialData += syncBytes[:syncProgress]  # save the ignored data for debugging
                    self.DEBUG_ignoredSerialData.append(potentialSyncByte)  # save the ignored data for debugging
                    syncProgress = 0
        if (printDebug):
            self.DEBUG_checkIgnoredSerialData()
        return (syncProgress)

    def waitForHandshake(self, startFlagTimeout=0.01, printDebug=True):
        if (not self._serial.is_open):
            print("can't waitForHandshake(), serial port is not open!")
        syncBytesRead = self.waitForSyncBytes(HANDSHAKE_SYNC_BYTES, startFlagTimeout)
        if (syncBytesRead < len(HANDSHAKE_SYNC_BYTES)):
            if (printDebug):
                print("waitForHandshake()", syncBytesRead, "<", len(HANDSHAKE_SYNC_BYTES))
            return (None)
        # else: # sync bytes were all received, time to read the message
        ## read bytes:
        # while((self._serial.in_waiting < connHandshake.itemsize) and ((time.time()-startTime) < startFlagTimeout)): # a manually enforced serial timeout
        #     doNothing = 0
        readBytes = self._serial.read(connHandshake.itemsize)  # NOTE: uses _serial.timeout
        if (len(readBytes) < connHandshake.itemsize):
            if (printDebug):
                print("waitForHandshake() insufficient bytes received when trying to read handshake")
            self.DEBUG_ignoredSerialData += readBytes  # save the ignored data for debugging
            return (None)
        ## with this many safety checks already done, i'm not gonna worry about this next part:
        handshakeToReturn = np.frombuffer(readBytes, dtype=connHandshake)[0]
        # check checksum
        return (handshakeToReturn)

    # # the following function works, but you should really use _doHandshakeIndef() instead. This function does NOT re-transmit the own handshake, it just wants endlessly
    # def waitForHandsakeIndef(self, resetESP=False, printEvery=1.0): # wait indefinitely (set printEvery to <= 0 to disable printing)
    #     if(not self._serial.is_open):
    #         print("can't waitForHandsakeIndef(), serial port is not open!")
    #     handshakeToReturn = None
    #     while((handshakeToReturn is None) and self._serial.is_open):
    #         if(resetESP):
    #             self._resetESP()
    #         handshakeToReturn = self.waitForHandshake((printEvery if (printEvery > 0) else 1.0), printEvery > 0)
    #     return(handshakeToReturn)

    def secsToMillis(self, PCtimestamp):  # used when sending a handshake
        return (np.uint32((PCtimestamp * 1000) % 4294967296))

    def convertTimestamp(self, ESPtimestamp):  # post-synchronization clock values
        return ((ESPtimestamp - self.handshakeReceivedTimestamp) / 1000.0)

    def timeSync(self, ESPtimestamp, PCtimestamp):
        self.handshakeLocalTimestamp = PCtimestamp
        self.handshakeReceivedTimestamp = ESPtimestamp

    def _doHandshake(self, meFirst=True, clockFunc=time.time, timeout=1.0, resetESP=False, printDebug=True):
        self.handshake_done = False
        if (not self._serial.is_open):
            print("can't _doHandshake(), serial port is not open!")
            return (None)
        if (resetESP):
            self._resetESP()
        ownHandshake = np.zeros((1,), dtype=connHandshake)[0]
        ownHandshake['identifier'] = self.ownHandshakeIdentifier  # see static class property (above __init__)
        if (meFirst):  # this device sends the handshake first
            ownHandshake['timestamp'] = self.secsToMillis(clockFunc())
            self.sendHandshake(ownHandshake)
            handshakeToReturn = self.waitForHandshake(timeout, printDebug)
            if (handshakeToReturn is not None):
                self.timeSync(handshakeToReturn['timestamp'],
                              clockFunc())  # the ESP responded with the handshake, so that is the most recent timestamp
                self.handshakeReceivedIdentifier = handshakeToReturn['identifier']
                self.handshake_done = True
        else:  # the other device sends the handshake first
            handshakeToReturn = self.waitForHandshake(timeout, printDebug)
            if (handshakeToReturn is not None):
                self.timeSync(handshakeToReturn['timestamp'],
                              clockFunc())  # the ESP responded with the handshake, so that is the most recent timestamp
                self.handshakeReceivedIdentifier = handshakeToReturn['identifier']
                self.handshake_done = True
                ownHandshake['timestamp'] = self.secsToMillis(clockFunc())
                self.sendHandshake(ownHandshake)
        return (handshakeToReturn)  # may return None, if waitForHandshake() returned None

    def _doHandshakeIndef(self, meFirst=True, clockFunc=time.time, printEvery=1.0, resetESP=False):
        self.handshake_done = False
        if (not self._serial.is_open):
            print("can't _doHandshakeIndef(), serial port is not open!")
            return (None)
        handshakeToReturn = None
        while ((handshakeToReturn is None) and self._serial.is_open):
            handshakeToReturn = self._doHandshake(meFirst, clockFunc, (printEvery if (printEvery > 0) else 1.0),
                                                  resetESP, printEvery > 0)
        return (handshakeToReturn)

    def switchSerials(self,
                      otherObj):  # if the lidarESP and kartMCU get their serial ports mixed up, you can switch their handlers on the fly
        temp = (
        self._serial, self.handshakeLocalTimestamp, self.handshakeReceivedTimestamp, self.handshakeReceivedIdentifier,
        self.handshake_done)
        # overwrite own values:
        self._serial = otherObj._serial
        self.handshakeLocalTimestamp = otherObj.handshakeLocalTimestamp
        self.handshakeReceivedTimestamp = otherObj.handshakeReceivedTimestamp
        self.handshakeReceivedIdentifier = otherObj.handshakeReceivedIdentifier
        self.handshake_done = otherObj.handshake_done
        # overwrite otherObj values:
        otherObj._serial, otherObj.handshakeLocalTimestamp, otherObj.handshakeReceivedTimestamp, otherObj.handshakeReceivedIdentifier, otherObj.handshake_done = temp


class lidarESPserialClass(
    handshakeESPserial):  # handles serial communication with the ESP32 (which communicates with the actual lidar). This is NOT a library for reading the lidar data directly
    def __init__(self, clockFunc=time.time, identifierIndex=0):
        handshakeESPserial.__init__(self)
        self.clockFunc = clockFunc
        self.correctHandshakeIdentifier = LIDAR_HANDSHAKE_IDENTIFIERS[
            identifierIndex]  # there are several lidarESPs, so this

    @property
    def _handshakeIdentifierCheck(self):
        """just a macro to make things sightly more legible (given the long-ass variable names and whatnot)"""
        return (self.handshakeReceivedIdentifier == self.correctHandshakeIdentifier)

    @property
    def is_ready(self):
        return (self.handshake_done and self._handshakeIdentifierCheck)

    def doHandshake(self, timeout=1.0, resetESP=False, printDebug=True):
        self.handshake_done = False
        if (not self.is_open):
            print("can't doHandshake(), serial port is not open!")
            return (self.is_ready)
        returnHandshake = self._doHandshake(True, self.clockFunc, timeout, resetESP, printDebug)
        if (returnHandshake is not None):
            if ((not self._handshakeIdentifierCheck) and printDebug):
                print("doHandshake() returnhandshake identifier wrong!:", returnHandshake['identifier'], "!=",
                      self.correctHandshakeIdentifier)
        return (self.is_ready)

    def doHandshakeIndef(self, resetESP=False, printDebug=True):
        self.handshake_done = False
        if (not self.is_open):
            print("can't doHandshakeIndef(), serial port is not open!")
            return (self.is_ready)
        returnHandshake = self._doHandshakeIndef(True, self.clockFunc, (1.0 if printDebug else -1), resetESP)
        if ((not self._handshakeIdentifierCheck) and printDebug):
            print("doHandshakeIndef() returnhandshake identifier wrong!:", returnHandshake['identifier'], "!=",
                  self.correctHandshakeIdentifier)
        return (self.is_ready)

    def _sendRequestPacket(self, requestToSend: LIDARconnRequest):
        if (not self.is_open):
            print("can't _sendRequestPacket(), serial port is not open!")
            return (None)
        requestToSend['checksum'] = _calcChecksum_connStruct(
            requestToSend)  # perpare for transmission by calculating and storing checksum
        self._serial.write(LIDAR_PACKET_SYNC_BYTES)
        self._serial.write(requestToSend.tobytes())

    def _recvPacketDescriptor(self, descriptorTimeout=0.01, printDebug=True):  # waits a little for the ESP to respond
        if (not self.is_open):
            print("can't _recvPacketDescriptor(), serial port is not open!")
            return (None)
        syncBytesRead = self.waitForSyncBytes(LIDAR_PACKET_SYNC_BYTES, descriptorTimeout)
        if (syncBytesRead < len(LIDAR_PACKET_SYNC_BYTES)):
            if (printDebug):
                print("_recvPacketDescriptor()", syncBytesRead, "<", len(LIDAR_PACKET_SYNC_BYTES))
            return (None)
        readBytes = self._serial.read(LIDARpacketDescriptor.itemsize)  # NOTE: uses _serial.timeout
        if (len(readBytes) < LIDARpacketDescriptor.itemsize):
            if (printDebug):
                print("_recvPacketDescriptor() insufficient bytes received when trying to read packet descriptor")
            self.DEBUG_ignoredSerialData += readBytes  # save the ignored data for debugging
            return (None)
        ## with this many safety checks already done, i'm not gonna worry about this next part:
        packetDescriptor = np.frombuffer(readBytes, dtype=LIDARpacketDescriptor)[0]
        if (packetDescriptor['checksum'] != _calcChecksum_connStruct(packetDescriptor)):  # basic safety check
            print("_recvPacketDescriptor() checksum bad!:", packetDescriptor['checksum'],
                  _calcChecksum_connStruct(packetDescriptor))
            self.DEBUG_ignoredSerialData += readBytes  # save the bad data for debugging
            return (None)
        return (packetDescriptor)

    def _requestPacket(self, requestToSend, carToUse, requestCMD):  # set request payload before using this function
        if (not self.is_open):
            print("can't _requestPacket(), serial port is not open!")
            return (None)
        requestToSend['requestCMD'] = requestCMD
        requestToSend['carPosX'] = np.float32(carToUse.position[0])
        requestToSend['carPosY'] = np.float32(carToUse.position[1])
        requestToSend['carOrient'] = np.float32(carToUse.angle)
        self._sendRequestPacket(requestToSend)
        packetDescriptor = self._recvPacketDescriptor()
        if (packetDescriptor is None):
            print("_requestPacket() no packetDescriptor received")
            return (None)
        # print("DEBUG: packetDescriptor received:", packetDescriptor)
        if (packetDescriptor['requestAck'] != requestCMD):  # extra safety check
            print("_requestPacket() requestAck returned different value:", packetDescriptor['requestAck'], requestCMD)
            return (None)
        return (packetDescriptor)

    def requestLidarData(self,
                         carToUse):  # note: carToUse is either a Map.Car object (thijs sim) or a car.py Car object (alex sim). the values .position and .angle have the same name in both
        payloadDtype = conePosType  # TBD: use
        if (not self.is_open):
            print("can't requestLidarData(), serial port is not open!")
            return (np.empty((0,), dtype=payloadDtype))
        requestToSend = np.zeros((1,), dtype=LIDARconnRequest)[0]
        packetDescriptor = self._requestPacket(requestToSend, carToUse, LIDAR_REQUEST_CONEDATA)
        if (packetDescriptor is None):
            print("requestLidarData() no packetDescriptor received")
            return (np.empty((0,), dtype=payloadDtype))
        if (packetDescriptor['requestAck'] != LIDAR_REQUEST_CONEDATA):  # extra safety check
            print("requestLidarData() requestAck returned different value:", packetDescriptor['requestAck'],
                  LIDAR_REQUEST_CONEDATA)
            return (np.empty((0,), dtype=payloadDtype))
        if (packetDescriptor['payloadCount'] <= 0):  # extra safety check
            # print("requestLidarData() payloadCount <= 0:", packetDescriptor['payloadCount'])
            return (np.empty((0,), dtype=payloadDtype))
        # print("DEBUG payloadType:", packetDescriptor['payloadType'], payloadDtype.itemsize)
        howManyBytesToRead = int(payloadDtype.itemsize * packetDescriptor['payloadCount'])
        readBytes = self._serial.read(howManyBytesToRead)  # NOTE: uses _serial.timeout
        if (len(readBytes) < howManyBytesToRead):
            print("requestLidarData() readBytes is smaller than expected!", len(readBytes), howManyBytesToRead,
                  len(readBytes) // payloadDtype.itemsize)
            # return(np.empty((0,), dtype=payloadDtype)) # no need per-se
            howManyBytesToRead = len(readBytes) // payloadDtype.itemsize
            self.DEBUG_ignoredSerialData += readBytes[howManyBytesToRead:]  # save the ignored data for debugging
            if (howManyBytesToRead == 0):
                return (np.empty((0,), dtype=payloadDtype))
            readBytes = readBytes[:howManyBytesToRead]  # ensure perfect item sizes
        lidarData = np.frombuffer(readBytes, dtype=payloadDtype)  # with
        # print("DEBUG lidarData:", len(lidarData), lidarData)
        # for i in range(len(lidarData)): # checksum checking loop
        #     if(lidarData[i]['CRCbyte'] != _calcChecksum_connStruct(lidarData[i])):
        #         print("lidarData entry CRC failed!", lidarData[i])
        #         self.DEBUG_ignoredSerialData += readBytes[(i*payloadDtype.itemsize):((i+1)*payloadDtype.itemsize)] # save the bad data for debugging
        #     # now to remove the bad entry from the list:
        #     lidarData = np.concatenate((lidarData[0:i], lidarData[(i+1):len(lidarData)])) # (not efficient, i know) carve entry out of array
        return (lidarData)

    def requestSetMaxRange(self, carToUse, newMaxRange: int):  # note: newMaxRange is in millimeters
        """set the max range at which the lidar will consider data to be valid.
            unit is in millimters!"""
        if (not self.is_open):
            print("can't requestSetMaxRange(), serial port is not open!")
            return (False)
        requestToSend = np.zeros((1,), dtype=LIDARconnRequest)[0]
        np.frombuffer(requestToSend['requestPayload'], dtype=np.uint16)[0] = newMaxRange
        packetDescriptor = self._requestPacket(requestToSend, carToUse, LIDAR_REQUEST_SET_MAX_RANGE)
        if (packetDescriptor is None):
            print("requestSetMaxRange() no packetDescriptor received")
            return (False)
        if (packetDescriptor['payloadCount'] != 0):  # extra debug check
            print("requestSetMaxRange() payloadCount should be 0:", packetDescriptor['payloadCount'])
        if (packetDescriptor['payloadType'] != newMaxRange):  # extra debug check
            print("requestSetMaxRange() payloadType should be equal to newMaxRange:", packetDescriptor['payloadType'],
                  newMaxRange)
        return (True)


class kartMCUserialClass(
    handshakeESPserial):  # handles communication between the kartMCU and the main PC. the PC sends desired speed & steering and the kartMCU responds with sensor data
    correctHandshakeIdentifier = KART_MCU_HANDSHAKE_IDENTIFIER  # static property

    def __init__(self, clockFunc=time.time):
        handshakeESPserial.__init__(self)
        self.clockFunc = clockFunc

    @property
    def _handshakeIdentifierCheck(self):
        """just a macro to make things sightly more legible (given the long-ass variable names and whatnot)"""
        return (self.handshakeReceivedIdentifier == self.correctHandshakeIdentifier)

    @property
    def is_ready(self):
        return (self.handshake_done and self._handshakeIdentifierCheck)

    def doHandshake(self, timeout=1.0, resetESP=False, printDebug=True):
        self.handshake_done = False
        if (not self.is_open):
            print("can't doHandshake(), serial port is not open!")
            return (self.is_ready)
        returnHandshake = self._doHandshake(True, self.clockFunc, timeout, resetESP, printDebug)
        if (returnHandshake is not None):
            if ((not self._handshakeIdentifierCheck) and printDebug):
                print("doHandshake() returnhandshake identifier wrong!:", returnHandshake['identifier'], "!=",
                      self.correctHandshakeIdentifier)
        return (self.is_ready)

    def doHandshakeIndef(self, resetESP=False, printDebug=True):
        self.handshake_done = False
        if (not self.is_open):
            print("can't doHandshakeIndef(), serial port is not open!")
            return (self.is_ready)
        # who send something first is basically arbitrary, since the PC has to request data packets, i'll also let it take the leading role in sending the handshake
        returnHandshake = self._doHandshakeIndef(True, self.clockFunc, (1.0 if printDebug else -1), resetESP)
        if ((not self._handshakeIdentifierCheck) and printDebug):
            print("doHandshakeIndef() returnhandshake identifier wrong!:", returnHandshake['identifier'], "!=",
                  self.correctHandshakeIdentifier)
        return (self.is_ready)

    def _sendRequestPacket(self, requestToSend: kartMCUconnRequest):
        if (not self.is_open):
            print("can't _sendRequestPacket(), serial port is not open!")
            return (None)
        requestToSend['checksum'] = _calcChecksum_connStruct(
            requestToSend)  # perpare for transmission by calculating and storing checksum
        self._serial.write(KART_MCU_PACKET_SYNC_BYTES)
        self._serial.write(requestToSend.tobytes())

    def _recvPacket(self, raw=False, descriptorTimeout=0.01, printDebug=True):  # waits a little for the ESP to respond
        if (not self.is_open):
            print("can't _recvPacket(), serial port is not open!")
            return (None)
        syncBytesRead = self.waitForSyncBytes(KART_MCU_PACKET_SYNC_BYTES, descriptorTimeout)
        if (syncBytesRead < len(KART_MCU_PACKET_SYNC_BYTES)):
            if (printDebug):
                print("_recvPacket()", syncBytesRead, "<", len(KART_MCU_PACKET_SYNC_BYTES))
            return (None)
        readBytes = self._serial.read((kartMCUdataRaw if raw else kartMCUdata).itemsize)  # NOTE: uses _serial.timeout
        if (len(readBytes) < (kartMCUdataRaw if raw else kartMCUdata).itemsize):
            if (printDebug):
                print("_recvPacket() insufficient bytes received when trying to read packet descriptor")
            self.DEBUG_ignoredSerialData += readBytes  # save the ignored data for debugging
            return (None)
        ## with this many safety checks already done, i'm not gonna worry about this next part:
        packet = np.frombuffer(readBytes, dtype=(kartMCUdataRaw if raw else kartMCUdata))[0]
        if (packet['checksum'] != _calcChecksum_connStruct(packet)):  # basic safety check
            print("_recvPacket() checksum bad!:", packet['checksum'], _calcChecksum_connStruct(packet))
            self.DEBUG_ignoredSerialData += readBytes  # save the bad data for debugging
            return (None)
        return (packet)

    def _requestPacket(self, requestToSend, carToUse, requestCMD):  # set request payload before using this function
        if (not self.is_open):
            print("can't _requestPacket(), serial port is not open!")
            return ()
        requestToSend['requestCMD'] = requestCMD
        if (requestCMD == KART_MCU_REQUEST_UPDATE_RAW):
            requestToSend['targetSteer'] = \
            np.frombuffer(np.int32(carToUse.desired_steering_raw).tobytes(), dtype=np.float32)[
                0]  # the numpy equivalent of a static cast
        else:
            requestToSend['targetSteer'] = np.float32(carToUse.desired_steering)
        requestToSend['targetSpeed'] = np.float32(carToUse.desired_velocity)
        self._sendRequestPacket(requestToSend)

    def requestKartData(self, carToUse,
                        raw=False):  # note: carToUse is either a Map.Car object (thijs sim) or a car.py Car object (alex sim). the values .position and .angle have the same name in both
        if (not self.is_open):
            print("can't requestKartData(), serial port is not open!")
            return (None)
        requestToSend = np.zeros((1,), dtype=kartMCUconnRequest)[0]
        self._requestPacket(requestToSend, carToUse, (KART_MCU_REQUEST_UPDATE_RAW if raw else KART_MCU_REQUEST_UPDATE))
        packet = self._recvPacket(raw)
        return (packet)

    def requestSetSteeringEnable(self, carToUse,
                                 enabled):  # note: carToUse is either a Map.Car object (thijs sim) or a car.py Car object (alex sim). the values .position and .angle have the same name in both
        if (not self.is_open):
            print("can't requestKartData(), serial port is not open!")
            return (None)
        requestToSend = np.zeros((1,), dtype=kartMCUconnRequest)[0]
        requestToSend['requestPayload'][0] = 1 if enabled else 0
        self._requestPacket(requestToSend, carToUse, KART_MCU_REQUEST_STEER_MOTOR_EN)
        # packet = self._recvPacket()
        # return(packet)

    def requestSetPedalPassthroughEnable(self, carToUse,
                                         enabled):  # note: carToUse is either a Map.Car object (thijs sim) or a car.py Car object (alex sim). the values .position and .angle have the same name in both
        if (not self.is_open):
            print("can't requestKartData(), serial port is not open!")
            return (None)
        requestToSend = np.zeros((1,), dtype=kartMCUconnRequest)[0]
        requestToSend['requestPayload'][0] = 1 if enabled else 0
        self._requestPacket(requestToSend, carToUse, KART_MCU_REQUEST_PEDAL_EN)
        # packet = self._recvPacket()
        # return(packet)


def shuffleSerials(serialHandlers):
    orderArray = []
    ## TODO!


def processConeData(lidarData, handshakeESPserial):
    '''
    Converts the Lidar data into a np array for the simulation
    '''
    ConeData = np.zeros((len(lidarData), 4))
    for i in range(len(lidarData)):
        ConeData[i, 0] = lidarData['pos'][i][0]  # absolute x_pos
        ConeData[i, 1] = lidarData['pos'][i][1]  # absolute y_pos
        ConeData[i, 2] = handshakeESPserial.convertTimestamp(lidarData['timestamp'][i])  # timestamp
        ConeData[i, 3] = lidarData['inaccuracy'][i]  # inaccuracy
    return ConeData


if __name__ == '__main__':

    startTime = time.time()
    clockFunc = lambda: (time.time() - startTime)
    printConnectionDebug = True  # so you dont have to change it for all the functions below

    ## lidar only test (with logging):
    try:
        test = lidarESPserialClass(clockFunc=clockFunc, identifierIndex=0)
        test.connect(comPort=None, autoFind=True, tryAny=False, printDebug=printConnectionDebug)
        test.doHandshakeIndef(resetESP=True, printDebug=True)
        if test.is_ready:
            class tempCarClass:
                position = np.array([0.0, 0.0])
                angle = 0.0
                velocity = 0.0

            tempCar = tempCarClass()
            print("conn test success:", test.requestSetMaxRange(tempCar, 1000))
            if test.is_ready:
                from log.HWserialConnLogging import LIDARserialLogger

                lidarLogger = LIDARserialLogger()
                while True:
                    test.DEBUG_checkIgnoredSerialData()
                    #Car = test.requestCar()
                    #lidarData = test.requestLidarData(Car)
                    lidarData = test.requestLidarData(tempCar)
                    ConeData = processConeData(lidarData, test)
                    print("test requestLidarData:", lidarData)
                    for conePos in lidarData:  # lidarData is always an array, but sometimes empty (if requestLidarData failed)
                        lidarLogger.logConePos(conePos)

                    time.sleep(0.02)  # 20ms == 50Hz == probably enough to keep up
            test.DEBUG_checkIgnoredSerialData()
    finally:
        print("newSerialComTest ending")
        try:
            test.disconnect()
            print("serial close success:", not test.is_open)
        except Exception as excep:
            print("couldn't disconnect serial:", excep)

    # ## kartMCU only test (with logging):
    # try:
    #     test = kartMCUserialClass(clockFunc=clockFunc)
    #     test.connect(comPort=None, autoFind=True, tryAny=False, printDebug=printConnectionDebug)
    #     test.doHandshakeIndef(resetESP=True, printDebug=True)
    #     if(test.is_ready):
    #         class tempCarClass:
    #             desired_steering = 0.0
    #             desired_steering_raw = 0
    #             desired_velocity = 0.0
    #         tempCar = tempCarClass()
    #         test.requestSetSteeringEnable(tempCar, False)
    #         test.requestSetPedalPassthroughEnable(tempCar, True)
    #         if(test.is_ready):
    #             from log.HWserialConnLogging import kartMCUserialLogger
    #             kartMCULogger = kartMCUserialLogger()
    #             while(True):
    #                 test.DEBUG_checkIgnoredSerialData()
    #                 # regularPacket = test.requestKartData(tempCar)
    #                 rawPacket = test.requestKartData(tempCar, True)
    #                 # print("test requestKartData:", regularPacket)
    #                 print("test requestKartData raw:", rawPacket)
    #                 if(rawPacket is not None):
    #                     kartMCULogger.logPacket(rawPacket, clockFunc())
    #                 time.sleep(0.01) # 10ms == 100Hz == plenty
    #         test.DEBUG_checkIgnoredSerialData()
    # finally:
    #     print("newSerialComTest ending")
    #     try:
    #         test.disconnect()
    #         print("serial close success:", not test.is_open)
    #     except Exception as excep:
    #         print("couldn't disconnect serial:", excep)
    #     try:
    #         kartMCULogger.close()
    #     except Exception as excep:
    #         print("couldn't close kartMCULogger:", excep)

    # ## lidar and kartMCU test:
    # try:
    #     lidar = lidarESPserialClass(clockFunc=clockFunc, identifierIndex=0)
    #     while(not lidar.connect(comPort=None, autoFind=True, tryAny=True, printDebug=printConnectionDebug)):
    #         time.sleep(0.5)
    #     lidar.doHandshakeIndef(resetESP=True, printDebug=True)
    #     kartMCU = kartMCUserialClass(clockFunc=clockFunc)
    #     while(not kartMCU.connect(comPort=None, autoFind=True, tryAny=True, exclusionList=[lidar.comPort], printDebug=printConnectionDebug)):
    #         time.sleep(0.5)
    #     kartMCU.doHandshakeIndef(resetESP=True, printDebug=True)
    #     if((not lidar._handshakeIdentifierCheck) and (not kartMCU._handshakeIdentifierCheck)): # if both have done a handshake (by definition, as doIndefHandshake was used on initialization)
    #         print("SWITCHING COM PORTS!")
    #         lidar.switchSerials(kartMCU)
    #     class tempCarClass:
    #         position = np.array([0.0, 0.0])
    #         angle = 0.0
    #         desired_steering = 0.0
    #         desired_steering_raw = 0
    #         desired_velocity = 0.0
    #     tempCar = tempCarClass()
    #     print("lidar conn test success:", lidar.requestSetMaxRange(tempCar, 1000))
    #     print("kartMCU conn test success:", kartMCU.requestSetSteeringEnable(tempCar, False))
    # finally:
    #     print("newSerialComTest ending")
    #     try:
    #         lidar.disconnect()
    #         print("lidar serial close success:", not lidar.is_open)
    #     except Exception as excep:
    #         print("couldn't disconnect lidar serial:", excep)
    #     try:
    #         kartMCU.disconnect()
    #         print("kartMCU serial close success:", not kartMCU.is_open)
    #     except Exception as excep:
    #         print("couldn't disconnect kartMCU serial:", excep)

    # ## two lidar test:
    # try:
    #     lidars = [lidarESPserialClass(clockFunc=clockFunc, identifierIndex=lidarIndex) for lidarIndex in range(2)]
    #     for lidarIndex in range(2):
    #         while(not lidars[lidarIndex].connect(comPort=None, autoFind=True, tryAny=True, exclusionList=[lidar.comPort for lidar in lidars], printDebug=printConnectionDebug)):
    #             time.sleep(0.5) # wait a little bit, to avoid spamming the terminal
    #         lidars[lidarIndex].doHandshakeIndef(resetESP=True, printDebug=True)
    #     if((not lidars[0].is_ready) and (not lidars[1].is_ready)): # if both have done a handshake (by definition, as doIndefHandshake was used on initialization)
    #         print("SWITCHING COM PORTS!")
    #         lidars[0].switchSerials(lidars[1])
    #     class tempCarClass:
    #         position = np.array([0.0, 0.0])
    #         angle = 0.0
    #     tempCar = tempCarClass()
    #     print("lidars[0] conn test success:", lidars[0].requestSetMaxRange(tempCar, 1000))
    #     print("lidars[1] conn test success:", lidars[1].requestSetMaxRange(tempCar, 2000))
    # finally:
    #     print("newSerialComTest ending")
    #     for lidarIndex in range(2):
    #         try:
    #             lidars[lidarIndex].disconnect()
    #             print("lidars[",lidarIndex,"] serial close success:", not lidars[lidarIndex].is_open)
    #         except Exception as excep:
    #             print("couldn't disconnect lidars[",lidarIndex,"] serial:", excep)

    # ## lidar and kartMCU test (with logging):
    # try:
    #     lidar = lidarESPserialClass(clockFunc=clockFunc, identifierIndex=0)
    #     while(not lidar.connect(comPort=None, autoFind=True, tryAny=True, printDebug=printConnectionDebug)):
    #         time.sleep(0.5)
    #     lidar.doHandshakeIndef(resetESP=True, printDebug=True)
    #     kartMCU = kartMCUserialClass(clockFunc=clockFunc)
    #     while(not kartMCU.connect(comPort=None, autoFind=True, tryAny=True, exclusionList=[lidar.comPort], printDebug=printConnectionDebug)):
    #         time.sleep(0.5)
    #     kartMCU.doHandshakeIndef(resetESP=True, printDebug=True)
    #     if((not lidar._handshakeIdentifierCheck) and (not kartMCU._handshakeIdentifierCheck)): # if both have done a handshake (by definition, as doIndefHandshake was used on initialization)
    #         print("SWITCHING COM PORTS!")
    #         lidar.switchSerials(kartMCU)
    #     class tempCarClass:
    #         position = np.array([0.0, 0.0])
    #         angle = 0.0
    #         desired_steering = 0.0
    #         desired_steering_raw = 0
    #         desired_velocity = 0.0
    #     tempCar = tempCarClass()
    #     if(lidar.is_ready and kartMCU.is_ready):
    #         print("starting logging...")
    #         from log.HWserialConnLogging import LIDARserialLogger, kartMCUserialLogger
    #         lidarLogger = LIDARserialLogger()
    #         kartMCULogger = kartMCUserialLogger()
    #         while(True):
    #             lidarData = lidar.requestLidarData(tempCar)
    #             for conePos in lidarData:
    #                 lidarLogger.logConePos(conePos)
    #             kartPacket = kartMCU.requestKartData(tempCar, True) # get (raw) data
    #             if(kartPacket is not None):
    #                 kartMCULogger.logPacket(kartPacket, clockFunc())
    # finally:
    #     print("newSerialComTest ending")
    #     try:
    #         lidar.disconnect()
    #         print("lidar serial close success:", not lidar.is_open)
    #     except Exception as excep:
    #         print("couldn't disconnect lidar serial:", excep)
    #     try:
    #         kartMCU.disconnect()
    #         print("kartMCU serial close success:", not kartMCU.is_open)
    #     except Exception as excep:
    #         print("couldn't disconnect kartMCU serial:", excep)
    #     try:
    #         lidarLogger.close()
    #     except Exception as excep:
    #         print("couldn't close lidarLogger:", excep)
    #     try:
    #         kartMCULogger.close()
    #     except Exception as excep:
    #         print("couldn't close kartMCULogger:", excep)