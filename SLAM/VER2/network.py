import socket

class Network:
	def __init__(self):
		self.CLIENT = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.HOST = socket.gethostbyname(socket.gethostname())
		self.PORT = 5050
		self.ADDR = (self.HOST, self.PORT)
		self.state = self.connect()
		print(self.state)

	def getState(self):
		return self.state

	def connect(self):
		try:
			self.CLIENT.connect(self.ADDR)
			return self.CLIENT.recv(2048).decode()
		except:
			pass

	def send(self, state):
		self.CLIENT.send(str.encode(state))
		return self.CLIENT.recv(2048).decode()

	def receive(self):
		return self.CLIENT.recv(2048).decode()

