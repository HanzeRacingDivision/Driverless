## some quick'n'dirty code for logging the serial communication

import datetime # used to auto-generate filenames
try:
    from basiclogger import basicLogger
except:
    from log.basiclogger import basicLogger

class LIDARserialLogger(basicLogger):
    def __init__(self, filename=None):
        if(filename is None):
            filename = "LIDARlog_" + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
        basicLogger.__init__(self, filename, ('timestamp', 'appendTimestamp',
                                                'pos_x', 'pos_y',
                                                #'origin_x', 'origin_y', 'originAngle',
                                                'inaccuracy', 'pointCount'))
    def logConePos(self, conePos):
        dictToWrite = {'timestamp' : conePos['timestamp'],
                        'appendTimestamp' : conePos['appendTimestamp'],
                        'pos_x' : conePos['pos'][0], 'pos_y' : conePos['pos'][1],
                        #'origin_x' : conePos['origin'][0], 'origin_y' : conePos['origin'][1], 'originAngle' : conePos['originAngle'],
                        'inaccuracy' : conePos['inaccuracy'],
                        'pointCount' : conePos['pointCount']}
        self._writeLineFromDict(dictToWrite)

class kartMCUserialLogger(basicLogger):
    def __init__(self, filename=None):
        if(filename is None):
            filename = "kartMCUlog_" + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S")
        basicLogger.__init__(self, filename, ('timestamp', 'steerAngle', 'enco_0', 'enco_1', 'enco_2', 'enco_3', 'throttle'))
    def logPacket(self, packet, timestamp):
        dictToWrite = {'timestamp' : timestamp,
                        'steerAngle' : packet['steerAngle'],
                        'enco_0' : packet['encoders'][0],
                        'enco_1' : packet['encoders'][1],
                        'enco_2' : packet['encoders'][2],
                        'enco_3' : packet['encoders'][3],
                        'throttle' : packet['throttle']}
        self._writeLineFromDict(dictToWrite)