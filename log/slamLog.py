## some quick'n'dirty code for logging SLAM

import datetime # used to auto-generate filenames
from basiclogger import basicLogger

class slamLogger(basicLogger):
    def __init__(self, filename=None):
        if(filename is None):
            filename = "slamLog_" + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
        basicLogger.__init__(self, filename) # auto-infer columnNames!
    
    def logSLAM(self, timestamp, ):
        dictToWrite = {'timestamp' : timestamp,
                       '' : , # TBD!
                       }
        self._writeLineFromDict(dictToWrite)
