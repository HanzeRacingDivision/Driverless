from humanfriendly import parse_date
import serial
from pynmeagps.nmeareader import NMEAReader
from math import floor
import json
import time

class BN220:
    def __init__(self):
        self._ser = serial.Serial('/dev/ttyS0',9600,timeout=.1)
        self._nmeareader = NMEAReader(self._ser)
        self._events = []
        self._start_time = time.time()

    def _serial_read_until(self,char='\n'):
        return self._ser.read_until(char)

    def get_pose(self):
        raw_data, msg = self._nmeareader.read()
        if msg:
            if msg.msgID == "GGA":
                payload = msg.payload
                lat = float(payload[1])
                lon = float(payload[3])
                alt = float(payload[8])

                dd_lat = self._dm_to_dd(lat)
                dd_lon = -self._dm_to_dd(lon)

                event = json.dumps({'time_stamp':time.time() - self._start_time,'lat':dd_lat,'lon':dd_lon,"alt":alt})
                self._events.append(event)
                return(dd_lat,dd_lon,alt)
        return (None,None,None)

    def _dm_to_dd(self,dm):
        degree = floor(dm/100.)
        min = dm - degree*100
        dd = degree + min/60.
        return dd
    
    
    def __del__(self):
        file_name = "raw_gps_" + str(self._start_time) + ".json"
        with open(file_name, 'w') as f:
            for event in self._events:
                f.write(event + '\n')

            