## a basic file logging class. creates simple CSV files.
## you can either specify the columnNames, or they'll be automatically inferred the first time you store a (dict{}) object

from hashlib import new


class basicLogger:
    def __init__(self, filename: str, columnNames = tuple()):
        self.filename = filename # store filename for later use
        self.logfile = open((filename if filename.endswith('.csv') else (filename + ".csv")), "w")
        self.columnNames = tuple()
        if(len(columnNames) > 0):
            self._writeColumnNames(columnNames)

    def __del__(self):
        try:
            self.logfile.close()
            # print("closed logfile from __del__")
        except:
            print("couldn't close logfile from __del__")

    def close(self):
        """just a macro for logfile.close()"""
        return(self.logfile.close())
    
    def _writeColumnNames(self, newColumnNames: tuple): # note: could (theoretically) also be used to change the format halfway through, if you really wanted to)
        if(len(newColumnNames) == 0): # extra safety check
            print("basicLogger.writeColumnNames error: no newColumnNames provided:", newColumnNames)
        ## TODO: order the new columnNames such that any shared entries with the old columnNames are in the same order (with spaces inbetween if entries are missing)
        columnString = ""
        for name in newColumnNames:
            columnString += name + ','
        columnString += '\n'
        self.logfile.write(columnString)
        self.columnNames = tuple(newColumnNames)

    def _writeLineFromDict(self, dataToWrite: dict):
        newLine = ""
        if(len(self.columnNames) == 0): # if columnNames are not set, infer them based on the provided dict
            self._writeColumnNames([entryName for entryName in dataToWrite])
        for entryName in self.columnNames:
            if(entryName in dataToWrite): # check if the entryName is an existing (key!) value in dataToWrite
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


if __name__ == '__main__':
    test = basicLogger("testfile", ('one', 'two'))
    test._writeLineFromDict({'one' : 1, 'two' : 2}) # regular usage
    test._writeLineFromDict({'two' : 2, 'one' : 1, 'four' : 4, 'three' : 3}) # saving more data than defined in the columnNames
    test._writeLineFromDict({'two' : 2}) # saving less data than defined in the columnNames