import socket
from _thread import *


def readData(params: str) -> list:
    """ Convert string data to list of floats for RECEIVING"""
    params = params.split(",")
    float_params = [float(param) for param in params]
    return float_params


def genData(params: tuple) -> str:
    """ Convert tuple of floats data to string for TRANSMITTING"""
    return str(params)[1:-1]


# state = [(40,40,0,0,0),(250,250,90,250,250,90)]
def threadedClient(conn):
    transceiver = conn.recv(2048).decode()
    print(transceiver)
    conn.send(str.encode(genData(state[transceiver])))

    while True:
        try:
            data = readData(conn.recv(2048).decode())
        except (ConnectionResetError, ValueError):  # remote station terminates connection
            print(f"[LOST CONNECTION TO '{ADDR[0]}' ON '{ADDR[1]}']")
            conn.close()
            break

        state[transceiver] = data
        if transceiver == "transmitter":
            reply = state["receiver"]
        else:
            reply = state["transmitter"]
        conn.send(str.encode(genData(reply)))


if __name__ == '__main__':

    # --------------------------------------------------------------------
    # Setting up server, port, socket, and communication
    # --------------------------------------------------------------------
    HOST = socket.gethostbyname(socket.gethostname())
    PORT = 5050
    ADDR = (HOST, PORT)

    # Define address family and communication type for the socket
    S = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(S)
    # - AF_INET: expects a pair (host,port) with the host having an IPv4 address and the port being an integer
    # - SOCK_STREAM: a connection-oriented communication type (use DGRAM for connectionless)

    # Bind server and port to the socket
    # you need try-except because you don't know if it works, e.g. port may be in use
    try:
        S.bind(ADDR)
    except OSError:
        print("Binding server and port to socket has failed.\n Check if port is already in use.")

    # Open up the port, a.k.a. "listen" to it with TCP listener
    S.listen(2)
    print("[HOST HAS BOOTED]")

    # --------------------------------------------------------------------
    # Handling connections
    # --------------------------------------------------------------------
    state = {"transmitter": (50, 50, 0, 0, 0), "receiver": (50, 50, 0, 0, 0)}
    while True:
        conn, ADDR = S.accept()
        print("Connected to", ADDR)
        start_new_thread(threadedClient, (conn, ))



