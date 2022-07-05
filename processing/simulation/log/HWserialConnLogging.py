## TODO:
# check if leaving an extra comma at the end is a problem for excel


import datetime # used to auto-generate filenames

class basicLogger:
    def __init__(self, filename: str, columnNames: tuple):
        self.filename = filename # store filename for later use
        self.logfile = open((filename if filename.endswith('.csv') else (filename + ".csv")), "w")
        self.columnNames = columnNames
        if(len(columnNames)):
            columnString = ""
            for name in columnNames:
                columnString += name + ','
            columnString += '\n'
            self.logfile.write(columnString)

    def __del__(self):
        try:
            self.logfile.close()
            # print("closed logfile from __del__")
        except:
            print("couldn't close logfile from __del__")

    def close(self):
        """just a macro for logfile.close()"""
        return(self.logfile.close())

    def _writeLineFromDict(self, dataToWrite: dict):
        newLine = ""
        if(len(self.columnNames) == 0): # if columnNames are not set, it'll work, but i can't guarantee a consistant data order
            for entryName in dataToWrite:
                newLine += str(dataToWrite[entryName])
                newLine += ','
        #elif(len(self.columnNames) == len(dataToWrite)):
        else:
            for entryName in self.columnNames:
                if(entryName in dataToWrite):
                    newLine += str(dataToWrite[entryName])
                    dataToWrite.pop(entryName) # only done so i can write any remaining data last
                newLine += ','
            for entryName in dataToWrite: # any entries that are NOT defined in the columnNames are just put at the end (in no particular order)
                newLine += str(dataToWrite[entryName])
                newLine += ','
        newLine += '\n'
        ## save line to file:
        self.logfile.write(newLine)
        return(newLine)


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



if __name__ == '__main__':
    test = basicLogger("testfile", ('one', 'two'))
    test._writeLineFromDict({'one' : 1, 'two' : 2}) # regular usage
    test._writeLineFromDict({'two' : 2, 'one' : 1, 'four' : 4, 'three' : 3}) # saving more data than defined in the columnNames
    test._writeLineFromDict({'two' : 2}) # saving less data than defined in the columnNames