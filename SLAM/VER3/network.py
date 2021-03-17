import socket


class Network:
    def __init__(self, transceiver):
        self.CLIENT = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.HOST = socket.gethostbyname(socket.gethostname())
        self.PORT = 5050
        self.ADDR = (self.HOST, self.PORT)

        self.connect()
        self.transceiver = transceiver
        self.send_transceiver()
        self.state = self.CLIENT.recv(2048).decode()

    def getState(self):
        return self.state

    def connect(self):
        try:
            self.CLIENT.connect(self.ADDR)
        except:
            print("Connection failed.")

    def send_transceiver(self):
        self.CLIENT.send(str.encode(self.transceiver))

    def send(self, state):
        self.CLIENT.send(str.encode(state))
        return self.CLIENT.recv(2048).decode()

    def receive(self):
        return self.CLIENT.recv(2048).decode()
