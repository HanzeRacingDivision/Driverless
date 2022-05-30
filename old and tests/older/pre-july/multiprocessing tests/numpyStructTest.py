import numpy as np

lidarPacket = np.dtype([('RPM', np.uint16),
                        ('startAngle', np.uint16),
                        ('endAngle', np.uint16),
                        ('timestamp', np.float64),
                        ('measurements', np.uint16, (8,)),
                        ('reservedData', np.uint8, (8,)),
                        ('dataFilled', np.int16),
                        ('CRCpassed', bool)])

aList = np.zeros(10, dtype=lidarPacket)




# class lidarPacket:
#     """a class for lidar data packets, based on the lidar's own communication format"""
#     def __init__(self, RPM=0, startAngle=0, endAngle=0, timestamp=time.time(), measurements=np.array([]), reservedData=np.array([]), CRCpassed=True):
#         self.RPM = RPM
#         self.startAngle = startAngle
#         self.endAngle = endAngle
#         self.timestamp = timestamp
#         self.measurements = np.zeros(8, dtype=np.int16);   self.measurements[0:len(measurements)] = measurements
#         self.reservedData = np.zeros(8, dtype=np.int16);   self.reservedData[0:len(reservedData)] = reservedData
#         self.dataFilled = len(measurements) #not all datapoint have to be filled with real data (if packets are not read all at once)
#         self.CRCpassed = CRCpassed #default to true, because it doesnt really matter that much
