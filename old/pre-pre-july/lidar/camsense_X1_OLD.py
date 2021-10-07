#TBD: serial timeout (asin, using the time since last byte to determine whether it's a new packet or part of the current one)


import serial
#import serial.tools.list_ports  #not needed, but useful to remember
import time
import numpy as np
import threading as thr #used for Semaphores

#note: lidar stops working correctly below 4.7V (400mA)
#at (norminal) 5V, it uses 400-420mA (fluctuation is the result of the PID loop on the motor speed control)


CAMSENSE_X1_SYNC_BYTES = (85, 170, 3, 8) #constants
CAMSENSE_X1_ANGLE_SUBTRACT_16 = 40960  #angle in degrees is ((16bit value)-0xA000)/64.00 or (((buffer[H]-0xA0)<<8) + buffer[L])/64.00
NO_MEASUREMENT = 32768 #the value the lidar reports if the it couldn't get a good measurement (bad surface reflection, too close, too far, etc.)
DEG_DIV = 64.0
DEG_MULT = 1/DEG_DIV
RAD_DIV = DEG_DIV * (180.0/np.pi)
RAD_MULT = 1/RAD_DIV

class camsense_X1:
    """a class for managing the Camsense-X1 serial communication.
        run the .run() function as often as possible (at least 400+ times/sec?),
        this will read serial data and parse packets when the serial buffer is sufficiently filled.
        the number of packets in .lidarData (list) is not constant, as this lidar transmits at constant intervals, not constant angles
        packets may overlap in angle-region, this is to preserve (sometimes very small) snippits of data. Just use the newer (lower index) packet's data.
        set .postParseCallback to a function you'd like to have called when new data is parsed.
        for multicore safety, please consider the .dataArrayWriting threading.Semaphore()"""
    def __init__(self, comPort, clockFunc=time.time):
        ##private variables:
        self._serialPort = serial.Serial(comPort, 115200, timeout=0.01)
        # self._serialPort.port = comPort
        # self._serialPort.baudrate = 115200
        # self._serialPort.timeout = 0.01 #a 10ms timeout (should only be needed for readline(), which i don't use)
        # self._serialPort.open()
        
        self.clockFunc = clockFunc
        
        self._packetSyncByteCount = 0 #counter to keep track of how many serial sync bytes, the total bytes of the current packet received is: _packetSyncByteCount+len(_packetRawBuf)
        self._packetRawBuf = [] #a buffer to hold the serial data for the 
        self._packetRawBufWriting = thr.Semaphore() #a semaphore, for multithreading safety
        #self._packetRawBufReady = False #flag to let another core/interrupt know that the data needs to be processed/translated
        self._runPacketTimeout = 0.5 #if no new packets have been received in this time, it's probably no longer spinning
        self._packetTimeoutTimer = self.clockFunc()
        
        self._lastAngles = [0, 0] #hold the angles of the last packet, to compare to the new ones (you could circumvent this with volatileindex-1 (with rollover!), but it's not that much trouble to have this little variable)
        
        ##public variables:
        self.spinning = False #set to true once the first sync byte comes through
        self.rotationCount = 0 #mostly used as an indicator that the first rotation is done, but also usefull to know when a new set of datapoints is available
        
        self.RPMraw = 0 #divide by 64 to get RPM
        
        self.lidarData = []
        self.volatileIndex = 0
        self.dataArrayWriting = thr.Semaphore() #a semaphore, for multithreading safety
        
        self.postParseCallback = None #callback function (pointer), it is called (after new data is added) with args: lidarSelf (pointer to lidar class), newData (lidarPacket with new data), pointsAdded (number of new datapoints)
        
    # def __del__(self): #doesn't work, i dont know why. You just have to close it manually ('try:' the whole sketch and close() port on exit ('finally:'))
    #     #print("deinitializing camsense_X1 object")
    #     try:
    #         self._serialPort.close() #conventional method
    #         self._serialPort.__del__() #this just calls close() underwater, i think
    #         del(self._serialPort) #a last ditch effort
    #     except:
    #         print("couldn't close camsense_X1 serial port from __del__")
    
    class lidarPacket:
        """a class for lidar data packets, based on the lidar's own communication format"""
        def __init__(self, RPM=0, startAngle=0, endAngle=0, timestamp=time.time(), measurements=[], reservedData=[], CRCpassed=True):
            self.RPM = RPM
            self.startAngle = startAngle
            self.endAngle = endAngle
            self.timestamp = timestamp
            self.measurements = [item for item in measurements] #copy list (possibly empty)
            self.reservedData = [item for item in reservedData] #copy list (possibly empty)
            self.CRCpassed = CRCpassed #default to true, because it doesnt really matter that much
    
    
    def _deleteLidarData(self, fromWhere, howMany):
        """pop several packets at once, enter from which index and how many packets"""
        for i in range(howMany):
            if(len(self.lidarData)>fromWhere): #safety
                self.lidarData.pop(fromWhere) #pop at the (new) end, which moves the next entry to delete dowwn into this (same) spot
            else:
                print("trying to delete non-existent lidarData?!")

    def _parseFullPacketBuf(self): #should be run with (semaphore) lock on _packetRawBufWriting
        """parse the full _packetRawBuf, returns the new lidar packet and how many datapoints were captured
            if the returned point count is 0, something went wrong.
            this should allow for future functions that only parse the currently available parts of the lidar packet"""
        if(len(self._packetRawBuf) < 32): #extra safety check
            print("you're trying to parse an incomplete packet with _parseFullPacketBuf() !!!")
            return(None, 0)
        newPacket = self.lidarPacket()
        newPacket.timestamp = self.clockFunc()
        newPacket.RPM = (self._packetRawBuf[1] << 8) + self._packetRawBuf[0] #combine bytes to make number
        newPacket.startAngle = (self._packetRawBuf[3] << 8) + self._packetRawBuf[2] #combine bytes to make number
        if(newPacket.startAngle == 0): #when booting up, the lidar will send packets with 0000, then A000 (CAMSENSE_X1_ANGLE_SUBTRACT_16), then real data.
            return(None, 0)
        newPacket.startAngle -= CAMSENSE_X1_ANGLE_SUBTRACT_16
        for i in range(8): #measurement data, 8 points per packet #only works in _parse FULL PacketBuf
            newPacket.measurements.append((self._packetRawBuf[(i*3)+5] << 8) + self._packetRawBuf[(i*3)+4]) #combine bytes to make number
            newPacket.reservedData.append(self._packetRawBuf[(i*3)+6]) #i still dont know what that last byte holds, but sometimes it's nonzero, so better store it to be safe
        newPacket.endAngle = (self._packetRawBuf[29] << 8) + self._packetRawBuf[28] #combine bytes to make number
        newPacket.endAngle -= CAMSENSE_X1_ANGLE_SUBTRACT_16
        if(newPacket.startAngle == newPacket.endAngle): #if both angles are equal, the packet is invalid (device still spinning up). These packets do have real distance measurements, but no associated angle
            return(None, 0)
        newPacket.CRCpassed = True # TBD  #the last 2 bytes ([30] and [31]) are probably the CRC, but documentation is... lacking
        
        ## determine how to put the new data in the existing array
        with self.dataArrayWriting: #wait for the semaphore to be free, and aquire() it (and release when done)
            if((newPacket.startAngle < self._lastAngles[0]) and (self.volatileIndex > 0)): #if the new packet is the first one of the new rotation (except the endAngle of the last packet can already be rolled over)
                if(self.volatileIndex < len(self.lidarData)):
                    self._deleteLidarData(self.volatileIndex, len(self.lidarData)-self.volatileIndex)
                    #self.lidarData = self.lidarData[:self.volatileIndex] #untested alternative
                self.volatileIndex = 0
                self.rotationCount += 1
            #if(self.rotationCount == 0): #for the first rotation, just insert data. (inserting data at the end of the array (even if the current array size is 0) is perfectly fine)
            if((self.rotationCount == 0) or (self.volatileIndex >= len(self.lidarData))): #for the first rotation, just insert data. (inserting data at the end of the array (even if the current array size is 0) is perfectly fine)
                self.lidarData.append(newPacket)
            else:
                keepSearchingArray = True
                packetsToDelete = 0
                insertNewPacket = False
                for i in range(self.volatileIndex, len(self.lidarData)):
                    if(self.lidarData[i].startAngle <= newPacket.startAngle): #if next startAngle is below 
                        if((self.lidarData[i].endAngle <= newPacket.startAngle) and (self.lidarData[i].startAngle < self.lidarData[i].endAngle)): #if the old packet is completely behind the new one, then it must be deleted altogether (e.g: old{0,6} new{7,13})
                            packetsToDelete+=1
                            if((len(self.lidarData)-packetsToDelete) <= self.volatileIndex): #if it intends to delete all entries between volatileIndex and the end of the list, then it must append (a.k.a. insert at len(lidarData))
                                insertNewPacket = True
                        elif((self.lidarData[i].endAngle <= newPacket.endAngle) or ((newPacket.endAngle < newPacket.startAngle) and (self.lidarData[i].startAngle < self.lidarData[i].endAngle))): #if next endAngle falls in within angle-range of the new packet
                            keepSearchingArray = False #the packet to overwrite has been found, stop searching
                            break
                        else: #if this is true, the old packet completely encompasses the new packet, which is strange
                            if(self.lidarData[i].startAngle < self.lidarData[i].endAngle):#in normal non-rollover conditions (so old{5,15} is fine, but old{355,10} is not)
                                insertNewPacket = True #will preserve more data (at the cost of array complexety, and therefore time)
                            #in rollover conditions, the last packet should simply be overwritten (at the cost of minor data loss) because it will not get deleted/overwritten correctly once obsolete
                            keepSearchingArray = False
                            break
                    else: #the next startAngle is obove the current one
                        if((self.lidarData[i].endAngle <= newPacket.endAngle) and (self.lidarData[i].startAngle < self.lidarData[i].endAngle)): #if this is true, the new packet completely encompasses the old packet, which is strange
                            #(very niche scenario) just overwrite the old packet with the new (encompassing) one.  (BTW, the (nextAngles[0] < nextAngles[1]) is to avoid old{358,5} packets being misinterpreted)
                            keepSearchingArray = False
                            break
                        else:
                            insertNewPacket = True
                            keepSearchingArray = False
                            break
                if((keepSearchingArray) and (packetsToDelete==0)):
                    print("keepSearchingArray but no deleted packets", self.volatileIndex, len(self.lidarData)) #error
                if((not insertNewPacket) and (packetsToDelete==0)):
                    #no weird stuff, just overwrite the existing packet at volatileIndex
                    self.lidarData[self.volatileIndex] = newPacket
                elif(insertNewPacket):
                    if(packetsToDelete > 0): #no need to delete AND insert, if you just delete one less and overwrite the leftover one, you can save a lot of time
                        packetsToDelete-=1
                        if(packetsToDelete > 0): #if multiple packets
                            self._deleteLidarData(self.volatileIndex, packetsToDelete)
                        self.lidarData[self.volatileIndex] = newPacket
                    else:
                        self.lidarData.insert(self.volatileIndex, newPacket)
                elif(packetsToDelete > 0):
                    self._deleteLidarData(self.volatileIndex, packetsToDelete)
                    self.lidarData[self.volatileIndex] = newPacket
            self.volatileIndex+=1
            if(newPacket.endAngle < newPacket.startAngle): #if this is a rollover packet
                if(self.volatileIndex < len(self.lidarData)): #volatileIndex should be at the max (technically an illigal position) by now. if it isn't, that means there is old data at the end
                    self._deleteLidarData(self.volatileIndex, len(self.lidarData)-self.volatileIndex)
                    #self.lidarData = self.lidarData[:self.volatileIndex] #untested alternative
                self.volatileIndex = 0 #not 100% necessary, but this does make sure volatileIndex is never equal to len(self.lidarData) (i think)
                self.rotationCount += 1
            #these last 3 could be excluded from the semaphore, to potentially save a few microseconds, but i'm not currently concerned with that sort of thing
            self._lastAngles[0] = newPacket.startAngle
            self._lastAngles[1] = newPacket.endAngle
            self.RPMraw = newPacket.RPM
            self._packetTimeoutTimer = self.clockFunc()
        return(newPacket, 8)
    
    def run(self, allowCallback=True): #you should NOT run this on multiple threads at once (for 1 lidar), it will mess with the serial buffer order
        """read serial data and parse when available.
            this should be ran as often as possible (400+ times/sec?)
            this should NOT be run on multiple threads at once, as that will mess with the order of the serial data!
            returns packet serial buffer progress, 0->36"""
        didSomething=False
        if(self._serialPort.is_open):
            readBytes = b''
            try:
                readBytes = self._serialPort.read_all() #this approach might be more pythonfriendly (instead of read()ing single bytes, which can have a lot of overhead?)
            except Exception as excep:
                print("read faillure:", excep)
            #while(self._serialPort.in_waiting > 0):
            for readByte in readBytes: #parse received bytes (if any)
                with self._packetRawBufWriting: #wait for the semaphore to be free, and aquire() it (and release when done)
                    if(self._packetSyncByteCount >= 4): #if the starting bytes have been received (note: _packetSyncByteCount should never exceed 4)
                        self._packetRawBuf.append(readByte)
                        if(len(self._packetRawBuf) == 32): #full packet received
                            #_packetRawBufReady = True #flag to let another core/interrupt know that the data needs to be processed/translated
                            newPacket, pointsAdded = self._parseFullPacketBuf()
                            self._packetRawBuf = []
                            self._packetSyncByteCount = 0
                            if(callable(self.postParseCallback) and allowCallback and ((newPacket is not None) or (pointsAdded > 0))):
                                self.postParseCallback(self, newPacket, pointsAdded)
                        didSomething=True
                    else:
                        #start sequence
                        if(readByte == CAMSENSE_X1_SYNC_BYTES[self._packetSyncByteCount]):
                            self._packetSyncByteCount+=1
                            if(self._packetSyncByteCount == 4):
                                self.spinning = True
                                didSomething=True
                        elif(self._packetSyncByteCount > 0):
                            print("packet sync byte faillure after", self._packetSyncByteCount, "good byte(s)")
                        else:
                            self._packetSyncByteCount=0
        else:
            print("can't run(), port not open")
        if(not didSomething):
            if((self.clockFunc() - self._packetTimeoutTimer) > self._runPacketTimeout):
                self.spinning = False
        return(self._packetSyncByteCount+len(self._packetRawBuf)) #return the number of bytes from the current packet are received (including sync bytes)
    
    def RPM(self):
        """returns actual Rotations Per Minute
            (.RPMraw is the raw (integer) value parsed in the last packet, divide by 64 to get actual RPM"""
        return(self.RPMraw/64.0)