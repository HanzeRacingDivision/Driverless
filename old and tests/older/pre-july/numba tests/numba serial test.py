from numba import njit, prange
import numpy as np
import time


## constants
START_SYNC_BYTES = np.array([], dtype=np.uint8)
END_SYNC_BYTES = np.array([], dtype=np.uint8)

DATA_LEN = 4 + 2 + 4 #byte sizes of sent datatypes

PACKET_LEN = len(START_SYNC_BYTES) + DATA_LEN + len(END_SYNC_BYTES)
START_SYNC_BYTE_INDEX = len(START_SYNC_BYTES)
END_SYNC_BYTE_INDEX = PACKET_LEN-len(END_SYNC_BYTES)



@njit
def parsePacket(packetBuf: np.ndarray):
    return(np.array([np.int64(np.frombuffer(packetBuf[0:4], dtype=np.uint32)[0]),
                     np.int64(np.frombuffer(packetBuf[4:6], dtype=np.int16)[0]),
                     np.int64(np.frombuffer(packetBuf[6:10], dtype=np.uint32)[0])], dtype=np.int64))

@njit
def formPacket(serialData: bytes, packetBuf: np.ndarray, packetBufLen: int):
    maxPacketCount = int(len(serialData) / PACKET_LEN)
    resultList = np.empty((maxPacketCount,3), dtype=np.int64)
    returnedItems = 0
    for i in range(len(serialData)):
        if(packetBufLen < START_SYNC_BYTE_INDEX):
            if(serialData[i] == START_SYNC_BYTES[packetBufLen]):
                packetBufLen += 1
            else:
                print("packet start sync failed after", packetBufLen, "bytes")
                packetBufLen = 0
        elif(packetBufLen >= END_SYNC_BYTE_INDEX):
            if(packetBufLen >= PACKET_LEN):
                print("ERROR, packet not emptied???")
            elif(serialData[i] == END_SYNC_BYTES[packetBufLen-END_SYNC_BYTE_INDEX]):
                packetBufLen += 1
                if(packetBufLen == PACKET_LEN):
                    resultList[returnedItems]=parsePacket(packetBuf)
                    returnedItems += 1
                    packetBufLen = 0
            else:
                print("packet end sync failed after", packetBufLen-END_SYNC_BYTE_INDEX, "bytes")
                packetBufLen = 0
        else:
            print(packetBufLen)
            packetBuf[packetBufLen-START_SYNC_BYTE_INDEX]=serialData[i]
            packetBufLen += 1
            if((packetBufLen == PACKET_LEN) and (END_SYNC_BYTE_INDEX == PACKET_LEN)): #if there are no end-sync-bytes
                resultList[returnedItems]=parsePacket(packetBuf)
                returnedItems += 1
                packetBufLen = 0
    return(packetBuf, packetBufLen, resultList, returnedItems)

packetBufLen = 0
packetBuf = np.zeros(DATA_LEN, dtype=np.uint8)

simSerialData = b'\x0E\x00\x00\x00\xF4\xFF\x22\x00\x00\x00'
packetBuf, packetBufLen, resultList, returnedItems = formPacket(simSerialData, packetBuf, packetBufLen)
print("precom", packetBufLen, resultList, returnedItems)

# N = 10000

# times = np.zeros(10)

# output = np.empty((N,3), dtype=np.int64)
# times[0] = time.time_ns()
# for i in range(N):
#     packetBuf, packetBufLen, resultList, returnedItems = formPacket(simSerialData, packetBuf, packetBufLen)
#     for j in range(returnedItems):
#         output[i] = resultList[j]
# times[1] = time.time_ns()

# print((times[1]-times[0]) / 1000000)