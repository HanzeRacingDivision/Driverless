## some quick'n'dirty code for logging the main loop (ARCv0)

import datetime # used to auto-generate filenames
from basiclogger import basicLogger

class mapLoggerClass(basicLogger):
    def __init__(self, filename=None):
        if(filename is None):
            filename = "mainLog_" + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
        basicLogger.__init__(self, filename) # auto-infer columnNames!
    
    def logMap(self, mapToUse):
        dictToWrite = {'timestamp' : mapToUse.clock(),
                       'carPosX' : mapToUse.car.position[0],
                       'carPosY' : mapToUse.car.position[1],
                       'carOrient' : mapToUse.car.angle,
                       'velocity' : mapToUse.car.velocity,
                       'steering' : mapToUse.car.steering,
                       'desired_velocity' : mapToUse.car.desired_velocity,
                       'desired_steering' : mapToUse.car.desired_steering}
        self._writeLineFromDict(dictToWrite)
