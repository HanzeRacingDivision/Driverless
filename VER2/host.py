import socket
from _thread import *

HOST = socket.gethostbyname(socket.gethostname())
PORT = 5050
ADDR = (HOST, PORT)

S = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
	S.bind(ADDR)
except:
	pass

S.listen(2)
print("[HOST HAS BOOTED]")

def readData(str, quant):
	str = str.split(",")
	lst = []
	for i in range(0,quant):
		lst.append(float(str[i]))
	return lst
	#return float(str[0]), float(str[1]), float(str[2]) , float(str[3]), float(str[4])

def genData(tup):
	return str(tup[0]) + "," + str(tup[1]) + "," + str(tup[2]) + "," + str(tup[3]) + "," + str(tup[4])

# object 0 --> simulation,    object 1 ---> reconstruction #

state = [(50,50,0,0,0),(50,50,0,0,0)]
#state = [(40,40,0,0,0),(250,250,90,250,250,90)]
def threadedClient(conn, object):
	conn.send(str.encode(genData(state[object])))
	reply = ""
	while True:
		data = readData(conn.recv(2048).decode(), 5)
		state[object] = data

		if object == 1:
			reply = state[0]

		else:
			reply = state[1]

		conn.send(str.encode(genData(reply)))

	print("[LOST CONNECTION]")
	conn.close()


while True:
	conn, ADDR = S.accept()
	print("Connected to ", ADDR)

	start_new_thread(threadedClient,(conn, currentObject))
	currentObject+=1
