import socket
import pickle
import time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432        # The port used by the server
timers = []

import random
class someClass:
    def __init__(self):
        self.someList = []
obj = someClass()
count = 3000
for i in range(count):
    obj.someList.append(random.random())
byteThing = pickle.dumps(obj)
print("bytesize:", len(byteThing))


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(s.getpeername())
    s.connect((HOST, PORT))
    data = s.recv(1000000)
    # s.sendall(byteThing)
    # print("also:",str(len(byteThing)).rjust(10, '0').encode())
    # s.sendall(str(len(byteThing)).rjust(10, '0').encode())
    # timers.append(time.time())
    # data = s.recv(len(byteThing))
    # timers.append(time.time())

print('Received', len(data), type(data))
#print(timers[1]-timers[0])