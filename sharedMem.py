import multiprocessing as MP
from multiprocessing import shared_memory
import pickle

import time #just for a time.sleep()

class sharedMemWriteWrapper(shared_memory.SharedMemory):
    """a wrapper for multiprocessing shared_memory.sharedMemory. Use pickle() to write data.
        (all data is double-buffered, so make sure to enter 2x max pickle size)"""
    def __init__(self, name=None, create=False, size=0):
        shared_memory.SharedMemory.__init__(self, name, create, size) #init sharedMemory as usual
        
        self.writeCursor = 0
        self.writeLock = MP.Lock()
        self.reading = MP.Array('i', [0, 0]) #these values represent how many other (slave) processes are currently reading each buffer (odd/even)
        self.pickleLen = MP.Array('i', [0, 0]) #how long the pickled data is (NOTE: pickleLen[1] also includes int(size/2), so the real length = pickleLen[1]-int(size/2)
        self.frameCounter = MP.Value('i', 0)
    
    def write(self, byteLikeObject):
        """used for pickle.dump()
            (this functions like IO (file) write functions, 
              it can be called multiple times by one pickle() call)"""
        writeSize = len(byteLikeObject)
        if((self.size if (self.writeCursor >= int(self.size/2)) else int(self.size/2)) >= (self.writeCursor + writeSize)): #if there is still room left
            self.buf[self.writeCursor:(self.writeCursor + writeSize)] = byteLikeObject[:]
            self.writeCursor += writeSize
        else:
            print("can't write() to sharedMem, not enough room!:", int(self.size/2), "< (", self.writeCursor - (int(self.size/2) if (self.writeCursor >= int(self.size/2)) else 0), "+", writeSize, ")")
            #raise(IndexError)
    
    def pickle(self, obj):
        """save an object to sharedMem"""
        with self.writeLock: #,self.frameCounter.get_lock():  #you don't really need to get frameCounter Lock
            if((self.frameCounter.value % 2) == 1): #if frameCounter is odd
                self.writeCursor = int(self.size/2) #write to odd space
            else:                                   #if frameCounter is even
                self.writeCursor = 0                #write to even space
            while(self.reading[(self.frameCounter.value % 2)] > 0): #before writing new data, make sure that all clients are done reading the old data (not the latest data, the older data)
                time.sleep(0.001) #wait a very short time
            pickle.dump(obj, self)
            self.pickleLen[(self.frameCounter.value % 2)] = self.writeCursor
            with self.frameCounter.get_lock():
                self.frameCounter.value += 1 #only once the new data is completely ready, does this counter increment
    
    def writePickled(self, pickledBytes):
        """save an object (that was already pickled) to sharedMem"""
        with self.writeLock: #,self.frameCounter.get_lock():  #you don't really need to get frameCounter Lock
            if((self.frameCounter.value % 2) == 1): #if frameCounter is odd
                self.writeCursor = int(self.size/2) #write to odd space
            else:                                   #if frameCounter is even
                self.writeCursor = 0                #write to even space
            while(self.reading[(self.frameCounter.value % 2)] > 0): #before writing new data, make sure that all clients are done reading the old data (not the latest data, the older data)
                time.sleep(0.001) #wait a very short time
            self.write(pickledBytes)
            self.pickleLen[(self.frameCounter.value % 2)] = self.writeCursor
            with self.frameCounter.get_lock():
                self.frameCounter.value += 1 #only once the new data is completely ready, does this counter increment
    
    @property
    def passToProcess(self):
        return(self.name, self.size, self.writeLock, self.reading, self.pickleLen, self.frameCounter)

class sharedMemReadWrapper(shared_memory.SharedMemory):
    def __init__(self, name, size, writeLock, reading, pickleLen, frameCounter: MP.Value): #you can use sharedMemWriteWrapper.passToProcess() to generate these arguments (in the correct order)
        shared_memory.SharedMemory.__init__(self, name, False) #init sharedMemory as usual
        
        self.initSize = size #the size of the shared memory file is often larger than the size that was requested, because of memory block size. This solves that
        self.writeLock = writeLock
        self.reading = reading              #an MP.Array('i') with size 2
        self.pickleLen = pickleLen          #an MP.Array('i') with size 2
        self.frameCounter = frameCounter    #an MP.Value('i')
        self.lastFrameCounter = 0 #self.frameCounter.value #(regular int) to know whether there is new data or not
    
    def poll(self, returnBool=True):
        if(returnBool):
            return(self.frameCounter.value > self.lastFrameCounter)
        else:
            return(self.frameCounter.value - self.lastFrameCounter)
    
    def readObj(self, keepWaiting=None):
        while((self.frameCounter.value < 1) and (keepWaiting.get_value() if (keepWaiting is not None) else True)):    #block untill any data exists
            print("waiting for frameCounter")
            # self.writeLock.acquire(True) #block if data write in progress
            # self.writeLock.release()
            time.sleep(0.2)
        if((self.frameCounter.value < 1) or ((not keepWaiting.get_value()) if (keepWaiting is not None) else False)):
           print("frameCounter waiting stopped, returning None")
           return(None)
        if(self.frameCounter.value == self.lastFrameCounter):
            print("warning: retrieving same object twice")
        returnObj = None #init var
        frameCounterVal = self.frameCounter.value #read once (because frameCounter may increment at (almost) any time)
        with self.reading.get_lock():
            self.reading[(0 if ((frameCounterVal % 2) == 1) else 1)] += 1
        try: #i'm not expecting errors, i just really want to make sure the reading[] gets decremented
            #dataStart = (0 if ((frameCounterVal % 2) == 1) else int(self.initSize/2))
            #dataEnd = self.pickleLen[(0 if ((frameCounterVal % 2) == 1) else 1)] #pickle stops automatically, so this is needless
            #returnObj = pickle.loads(self.buf[dataStart : dataEnd])
            if((frameCounterVal % 2) == 1):                                 #if frameCounter is odd
                returnObj = pickle.loads(self.buf[0 : int(self.initSize/2)])#read from even space
            else:                                                           #if frameCounter is even
                returnObj = pickle.loads(self.buf[int(self.initSize/2) : ]) #read from odd space
        finally:
            with self.reading.get_lock():
                self.reading[(0 if ((frameCounterVal % 2) == 1) else 1)] -= 1
            self.lastFrameCounter = frameCounterVal
        return(returnObj)
    
    def readNewObj(self, keepWaiting=None, timeout=None, printTimeoutMessage=True):
        if(self.frameCounter.value < 1):
            return(self.readObj(keepWaiting))
        startTime = time.time()
        while((not self.poll()) and (True if (timeout is None) else ((time.time()-startTime)<timeout))):
            time.sleep(0.001) #wait for new data to come in
        if((False if (timeout is None) else ((time.time()-startTime)>timeout)) and printTimeoutMessage):
            print("readNewObj timout")
        return(self.readObj(keepWaiting))
    
    def readBytes(self, keepWaiting=None):
        while((self.frameCounter.value < 1) and (keepWaiting.get_value() if (keepWaiting is not None) else True)):    #block untill any data exists
            print("waiting for frameCounter")
            # self.writeLock.acquire(True) #block if data write in progress
            # self.writeLock.release()
            time.sleep(0.2)
        if((self.frameCounter.value < 1) or ((not keepWaiting.get_value()) if (keepWaiting is not None) else False)):
           print("frameCounter waiting stopped, returning None")
           return(None)
        if(self.frameCounter.value == self.lastFrameCounter):
            print("warning: retrieving same bytes twice")
        returnBytes = None #init var
        frameCounterVal = self.frameCounter.value #read once (because frameCounter may increment at (almost) any time)
        with self.reading.get_lock():
            self.reading[(0 if ((frameCounterVal % 2) == 1) else 1)] += 1
        dataStart = (0 if ((frameCounterVal % 2) == 1) else int(self.initSize/2))
        dataEnd = self.pickleLen[(0 if ((frameCounterVal % 2) == 1) else 1)] #pickle stops automatically, so this is needless
        returnBytes = self.buf[dataStart : dataEnd] #i'm hoping this is not too shallow of a copy
        with self.reading.get_lock():
            self.reading[(0 if ((frameCounterVal % 2) == 1) else 1)] -= 1
        self.lastFrameCounter = frameCounterVal
        return(returnBytes)
    
    def readNewBytes(self, keepWaiting=None, timeout=None, printTimeoutMessage=True):
        if(self.frameCounter.value < 1):
            return(self.readBytes(keepWaiting))
        startTime = time.time()
        while((not self.poll()) and (True if (timeout is None) else ((time.time()-startTime)<timeout))):
            time.sleep(0.001) #wait for new data to come in
        if((False if (timeout is None) else ((time.time()-startTime)>timeout)) and printTimeoutMessage):
            print("readNewObj timout")
        return(self.readBytes(keepWaiting))

def makeNewSharedMem(name: str, size: int):
    """initializing sharedMemory objects can throw a number of exceptions, this function attempts to handle them"""
    try:
        sharedMem = sharedMemWriteWrapper(name, False) #if the memory block still exist (bad cleanup on exit last time)
        if(sharedMem.size >= size):
            print("found old sharedMem, but the size >= requested size, so let's recycle it,", sharedMem.size, ">=", size)
            return(sharedMem)
        else:
            print("found old sharedMem!, closing and recreating:", name, size)
            sharedMem.close()
            sharedMem.unlink()
            del(sharedMem)
            time.sleep(0.1)
    except FileNotFoundError:
        print("creating new sharedMem:", name, size)
    try:
        return(sharedMemWriteWrapper(name, True, size))
    except FileExistsError as excep:
        print("couldn't create sharedMem, python sucks (at least on windows)")
        raise(excep)