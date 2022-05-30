from numba import njit, typeof
from numba import types as Ctyp
import numpy as np
import time

lidarPacket = np.dtype([('RPM', np.uint16),
                        ('startAngle', np.uint16),
                        ('endAngle', np.uint16),
                        ('timestamp', np.float64),
                        ('measurements', np.uint16, (8,)),
                        ('reservedData', np.uint8, (8,)),
                        ('dataFilled', np.int16),
                        ('CRCpassed', bool)])

#print(Ctyp.Array(Ctyp.int32, 1, 'C'))


@njit#((Ctyp.Array(Ctyp.int32, 1, 'C'), Ctyp.Array(lidarPacket, 1, 'C')))
def thing(one, two):
    print(one[0])
    print(two[one[0]])

aList = np.array([1,2,3,4], dtype=np.int32)
also = np.zeros(5, dtype=lidarPacket)


times = np.zeros(10)

times[0] = time.time_ns()
thing(aList, also)
times[1] = time.time_ns()
# print(posList)
# print(result)
# print()
times[2] = time.time_ns()
thing(aList, also)
times[3] = time.time_ns()
# print(posListTwo)
# print(result)
# print()
times[4] = time.time_ns()
thing(aList, also)
times[5] = time.time_ns()
# print(posListTwo)
# print(result)
# print()
print((times[1]-times[0]) / 1000000)
print((times[3]-times[2]) / 1000000)
print((times[5]-times[4]) / 1000000)

thing.inspect_types()
