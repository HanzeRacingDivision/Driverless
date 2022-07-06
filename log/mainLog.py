## some quick'n'dirty code for logging the main loop (of ARCv0). Mostly useful for logging SLAM

import datetime # used to auto-generate filenames
from basiclogger import basicLogger

class masterMapLogger(basicLogger):
    def __init__(self, filename=None):
        if(filename is None):
            filename = "masterMapLog_" + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
        basicLogger.__init__(self, filename) # auto-infer columnNames!
    
    def logMasterMap(self, masterMap):
        dictToWrite = {'timestamp' : masterMap.clock(),
                       'carPosX' : TBD!!!
                       }
        self._writeLineFromDict(dictToWrite)
