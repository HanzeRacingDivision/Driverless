import serial
import serial.tools.list_ports
import time
import numpy as np

from Map import Map
import carMCUclass  as RC


if __name__ == "__main__":
    try:
        comPort = "COM4"
        autoFind = False
        autoreconnect = True
        #precompileAll()
        carConn = RC.carMCUserial(comPort, autoFind, time.time, print) #you could also initialize with "time.time)" as the clock func, you'd just need to let
        while(True):
            try:
                while((not carConn.carMCUserial.is_open) and autoreconnect):
                    print("carMCU reconnecting with port:", carConn.carMCUserial.port)
                    carConn.connect(carConn.carMCUserial.port, False) #try to connect again with the same comPort
                    if(not carConn.carMCUserial.is_open):
                        time.sleep(0.25) #wait a bit between connection attempts
                while(carConn.carMCUserial.is_open):
                    loopStart = time.time()
                    carConn.getFeedback() #run this to get the data
                    steerAngle = np.sin((loopStart * 0.125) * 2*np.pi) * Map.Car.maxSteeringAngle
                    carConn.sendSpeedAngle(0.0, steerAngle)
                    # else: #this else-statement is not needed, but i figure if you've already spent time running functions, you dont also need to sleep
                    #     time.sleep(carConn.defaultGetFeedbackInterval) #wait just a little bit, as to not needlessly overload the CPU
                    loopEnd = time.time()
                    if((loopEnd-loopStart)>0.1):
                        print("carMCUserial running slow", 1/(loopEnd-loopStart))
                # if(not self.autoreconnect):   #just for debugging
                #     print("carMCU connection on thread stopped becuase is_open:", carConn.carMCUserial.is_open)
                #     continue
                # if(not self.keepalive.get_value()):
                #     print("stopping carMCU runOnThread function...")
                #     continue
            except serial.SerialException as excepVar:
                print("carMCU serial exception, dealing with it...")
                if(excepVar.args[0].find("Access is denied.")): #this is in the error message that happens when you unplug an active device
                    carConn.connect(carConn.carMCUserial.port, False, True) #try to connect again with the same comPort (will probably result in just disconnecting)
                time.sleep(0.25)
            except Exception as excep:
                print("carMCU other exception:", excep)
                try:
                    carConn.disconnect()
                except:
                    print("couldn't disconnect")
                time.sleep(0.25)
    finally:
        print("carMCUserialProcess ending")
        try:
            carConn.disconnect()
        except:
            print("couldn't disconnect carSerial")