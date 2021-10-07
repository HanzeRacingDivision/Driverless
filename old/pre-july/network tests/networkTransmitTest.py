import socket
import pickle
import time

HOST = ''  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
timers = []

class someClass:
    def __init__(self):
        self.aList = [] #the actual attributes of the class dont matter at all, becuase the class will be reconstructed 

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("listening...")
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        data = conn.recv(1000000)
        timers.append(time.time())
        conn.sendall(data)
        timers.append(time.time())
        print("returned:", len(data), type(data))
        unpacked = pickle.loads(data)
        print(len(pickle.dumps(unpacked)), len(data))
        print(type(unpacked), len(unpacked.someList))
        print(timers[1] - timers[0])