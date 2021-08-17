#TBD: serial timeout (asin, using the time since last byte to determine whether it's a new packet or part of the current one)


import serial
#import serial.tools.list_ports  #not needed, but useful to remember
import time
import numpy as np
# import multiprocessing as MP #used for Semaphores
# from multiprocessing import shared_memory

#note: lidar stops working correctly below 4.7V (400mA)
#at (norminal) 5V, it uses 400-420mA (fluctuation is the result of the PID loop on the motor speed control)

from numba import njit


CAMSENSE_X1_SYNC_BYTES = (85, 170, 3, 8) #constants
CAMSENSE_X1_ANGLE_SUBTRACT_16 = 40960  #angle in degrees is ((16bit value)-0xA000)/64.00 or (((buffer[H]-0xA0)<<8) + buffer[L])/64.00
NO_MEASUREMENT = 32768 #the value the lidar reports if the it couldn't get a good measurement (bad surface reflection, too close, too far, etc.)
DEG_DIV = 64.0
DEG_MULT = 1/DEG_DIV
RAD_DIV = DEG_DIV * (180.0/np.pi)
RAD_MULT = 1/RAD_DIV
DATA_LEN = 2 + 2 + 2 + 8*2 + 8*1 + 2
PACKET_LEN = len(CAMSENSE_X1_SYNC_BYTES) + DATA_LEN

EXAMPLE_SERIAL_DATA = [b'U\xaa\x03\x08\xcfM\xe3\xb7Z\x089c\x086g\x08:\x00\x80\x00F\x07\x06\x00\x80\x00\x00\x80\x00f\x07\x02s\xb9pZ',
                       b'U\xaa\x03\x08\xcfM\xac\xb9\x9e\x08\x1c\xa2\x08/\xab\x08&R\x08\x1c\xfe\x07%\xb3\x07 \xa4\x07=\xa2\x07`=\xbb\xbc\x05',
                       b'U\xaa\x03\x08\xcfMv\xbb\xad\x07]~\x076@\x07?\x02\x07I\xc8\x06X\x97\x06Mf\x06\\;\x06a\x07\xbd3a']


lidarPacket = np.dtype([('RPM', np.uint16),
                        ('startAngle', np.uint16),
                        ('endAngle', np.uint16),
                        ('timestamp', np.float64),
                        ('measurements', np.uint16, (8,)),
                        ('reservedData', np.uint8, (8,)),
                        ('dataFilled', np.int16),
                        ('CRCpassed', bool)])

@njit
def fullBufferToPacket(byteBuffer, packetToWrite, timestamp: np.float64):
    """convert a (full) serial-byte-buffer into a lidarPacket object/struct/dtype
        numba compiled!"""
    packetToWrite['timestamp'] = timestamp
    packetToWrite['RPM'] = (byteBuffer[1] << 8) + byteBuffer[0] #combine bytes to make number
    packetToWrite['startAngle'] = (byteBuffer[3] << 8) + byteBuffer[2] #combine bytes to make number
    if(packetToWrite['startAngle'] == 0): #when booting up, the lidar will send packets with 0000, then A000 (CAMSENSE_X1_ANGLE_SUBTRACT_16), then real data.
        return(0)
    packetToWrite['startAngle'] -= CAMSENSE_X1_ANGLE_SUBTRACT_16
    for i in range(8): #measurement data, 8 points per packet #only works in _parse FULL PacketBuf
        packetToWrite['measurements'][i] = (byteBuffer[(i*3)+5] << 8) + byteBuffer[(i*3)+4] #combine bytes to make number
        packetToWrite['reservedData'][i] = byteBuffer[(i*3)+6] #i still dont know what that last byte holds, but sometimes it's nonzero, so better store it to be safe
    packetToWrite['dataFilled'] = 8
    packetToWrite['endAngle'] = (byteBuffer[29] << 8) + byteBuffer[28] #combine bytes to make number
    packetToWrite['endAngle'] -= CAMSENSE_X1_ANGLE_SUBTRACT_16
    if(packetToWrite['startAngle'] == packetToWrite['endAngle']): #if both angles are equal, the packet is invalid (device still spinning up). These packets do have real distance measurements, but no associated angle
        return(0)
    packetToWrite['CRCpassed'] = True # TBD  #the last 2 bytes ([30] and [31]) are probably the CRC, but documentation is... lacking
    return(8)

@njit
def extractPackets(serialData: bytes, _packetBuf: np.ndarray, _packetBufLen, timestamp: np.float64):
    """handle serial data (bytes) to produce packets
        numba compiled!"""
    maxPacketCount = int((len(serialData)+_packetBufLen[0]) / PACKET_LEN) #the max number of packets that could possibly be extracted from this data
    parsedPackets = np.empty(maxPacketCount, dtype=lidarPacket) #instead of making an appendable list (which is technically allowed in @njit), make a (possibly oversized) array. (faster)
    parsedPacketCount = 0 #if packets fail (sync issues or otherwise) then some the last (few) indices will not contain actual data. Use this instead of len(resultList)
    for i in range(len(serialData)):
        if(_packetBufLen[0] < len(CAMSENSE_X1_SYNC_BYTES)):
            if(serialData[i] == CAMSENSE_X1_SYNC_BYTES[_packetBufLen[0]]):
                _packetBufLen[0] += 1
            else:
                if(_packetBufLen[0] > 0): #dont spam warnings on first startup
                    print("packet start sync failed after", _packetBufLen[0], "bytes")
                _packetBufLen[0] = 0
        else:
            _packetBuf[_packetBufLen[0]-len(CAMSENSE_X1_SYNC_BYTES)]=serialData[i]
            _packetBufLen[0] += 1
            if(_packetBufLen[0] >= PACKET_LEN): #_packetBufLen[0] should never exceed PACKET_LEN, but just to be sure
                if(fullBufferToPacket(_packetBuf, parsedPackets[parsedPacketCount], timestamp)):
                    parsedPacketCount += 1
                _packetBufLen[0] = 0
    return(parsedPackets, parsedPacketCount)


def precompileAll():
    print("procompiling camsense_X1 functions...")
    compileStartTime = time.time()
    ## important!: when precompiling liek this, remember to make sure to use EXACTLY the same types (it's easy to miss a uint16/uint32 difference for example)
    tempList = np.zeros(3, dtype=lidarPacket)
    fullBufferToPacket(EXAMPLE_SERIAL_DATA[0][4::], tempList[0], time.time())
    tempPacket = extractPackets(EXAMPLE_SERIAL_DATA[1], np.zeros(DATA_LEN, dtype=np.uint8), np.array([0], dtype=np.uint8), time.time())[0][0]
    del(tempList); del(tempPacket)
    print("camsense_X1 compilation done! (took", round(time.time()-compileStartTime,1), "seconds)")




class camsense_X1:
    """a class for managing the Camsense-X1 serial communication.
        run the .run() function as often as possible (at least 400+ times/sec?),
        this will read serial data and parse packets when the serial buffer is sufficiently filled.
        the number of packets in .lidarData (list) is not constant, as this lidar transmits at constant intervals, not constant angles
        packets may overlap in angle-region, this is to preserve (sometimes very small) snippits of data. Just use the newer (lower index) packet's data.
        set .postParseCallback to a function you'd like to have called when new data is parsed.
        for multicore safety, please consider the .dataLock multiprocessing.Lock()"""
    def __init__(self, comPort, clockFunc=time.time):
        precompileAll() #if it's already compiled (for some reason), then this will not take as long
        ##private variables:
        self._serialPort = serial.Serial(comPort, 115200, timeout=0.01)
        # self._serialPort.port = comPort
        # self._serialPort.baudrate = 115200
        # self._serialPort.timeout = 0.01 #a 10ms timeout (should only be needed for readline(), which i don't use)
        # self._serialPort.open()
        
        self.clockFunc = clockFunc
        
        self._packetBuf = np.zeros(DATA_LEN, dtype=np.uint8) #used to remember partially-received packets between getFeedback() runs
        self._packetBufLen = np.array([0], dtype=np.uint8) #_packetBuf is initialized at full length (For numba & speed reasons), this indicates how much of that data is current. Any data beyond this index is old (not part of the current packet)
        
        self._runPacketTimeout = 0.5 #if no new packets have been received in this time, it's probably no longer spinning
        self._packetTimeoutTimer = self.clockFunc()
        
        self._lastAngles = np.array([0, 0], dtype=lidarPacket['startAngle']) #hold the angles of the last packet, to compare to the new ones (you could circumvent this with volatileindex-1 (with rollover!), but it's not that much trouble to have this little variable)
        
        ##public variables:
        self.spinning = False #set to true once the first sync byte comes through
        self.rotationCount = np.array([0], dtype=np.uint32) #(pointer hack) #mostly used as an indicator that the first rotation is done, but also usefull to know when a new set of datapoints is available
        
        self.RPMraw = 0 #divide by 64 to get RPM
        
        self.postParseCallback = None #callback function (pointer), it is called (after new data is added) with args: lidarSelf (pointer to lidar class), newData (lidarPacket with new data), pointsAdded (number of new datapoints)
        self.callbackExtraArg = None #an extra place to store data (intended for the callback function). Not passed to function, you can just call lidarSelf.callbackExtraArg (becuase this class's "self" is passed)
        
        
    # def __del__(self): #doesn't work, i dont know why. You just have to close it manually ('try:' the whole sketch and close() port on exit ('finally:'))
    #     #print("deinitializing camsense_X1 object")
    #     try:
    #         self._serialPort.close() #conventional method
    #         self._serialPort.__del__() #this just calls close() underwater, i think
    #         del(self._serialPort) #a last ditch effort
    #     except:
    #         print("couldn't close camsense_X1 serial port from __del__")
    
    # def __del__(self):
    #     try:
    #         self.lidarDataSharedMem.close()
    #         self.lidarDataSharedMem.unlink()
    #     except:
    #         print("(__del__) sharedMem unlink failed! for camsense_X1")
    
    def __repr__(self):
        return("lidar("+str(self._serialPort.port)+\
               ", spin:"+str(self.spinning)+\
               ", spun:"+str(self.rotationCount[0])+\
               ((", CB:"+self.postParseCallback.__name__) if callable(self.postParseCallback) else "")+")")
    
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
            parsedPackets, parsedPacketCount = extractPackets(readBytes, self._packetBuf, self._packetBufLen, self.clockFunc())
            # for i in range(parsedPacketCount):
            #     self._putInPacketList(parsedPackets[i]) #used to call numba compiled function
            if(parsedPacketCount > 0):
                self.spinning = True
                self.RPMraw = parsedPackets[parsedPacketCount-1]['RPM']
                self._packetTimeoutTimer = self.clockFunc()
            if(callable(self.postParseCallback) and allowCallback):
                for i in range(parsedPacketCount):
                    self.postParseCallback(parsedPackets[i], 8, self.callbackExtraArg) #partial-packet-parsing is not currently supported, so there are always 8 new datapoints/packet, or 0
        else:
            print("can't run(), port not open")
        if(not didSomething):
            if((self.clockFunc() - self._packetTimeoutTimer) > self._runPacketTimeout):
                self.spinning = False
        return(self._packetBufLen[0]) #return the number of bytes from the current packet are received (including sync bytes)
    
    def RPM(self):
        """returns actual Rotations Per Minute
            (.RPMraw is the raw (integer) value parsed in the last packet, divide by 64 to get actual RPM"""
        return(self.RPMraw/64.0)


