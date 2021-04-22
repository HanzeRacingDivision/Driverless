from camsense_X1 import camsense_X1
import time

# def callbackFunc(lidarSelf, newPacket, pointsAdded): #example callback function
#     if((newPacket is not None) and (pointsAdded > 0)):
#         print(newPacket.startAngle) #do note: this code will be run about 50packets/Rotation * 5rotations/sec = 250 times per second, so try to make this as quick as possible (like an interrupt function)

printTimer = time.time()
try:
    lidar = camsense_X1('COM7')
    #lidar.postParseCallback = callbackFunc
    
    lastRotCount = 0
    while(True):
        lidar.run() #needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
        
        rightNow = time.time()
        if((rightNow - printTimer) > 0.5):
            with lidar.dataLock:
                RPM = ((lidar.rotationCount[0] - lastRotCount) / (rightNow - printTimer))*60 #a (very inaccurate) RPM measurement, based on packet data rollover (rotationCount)
                print(lidar.spinning, lidar.lidarDataLen[0], lidar.rotationCount[0], round(RPM,1), round(lidar.RPM(),1))
                lastRotCount = lidar.rotationCount[0]
            printTimer = rightNow
finally: #required for correct serial port closing (otherwise, you need to unplug and replug device every time)
    try:
        lidar._serialPort.close()
        print("closed lidar serial port")
    except:
        print("coudln't close lidar serial port")