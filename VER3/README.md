VER3: Third primitive version of a 'world builder'.
## Updates Relative to VER2:
- modifications in host.py:
    - prevented shutting down the hosting after stopping communication with either the transmitter or the receiver (e.g. main.py and main2.py),
    - there is no more limit on the number of connections and disconnections on the host (previous host was limited to two, regardless of transceiver type)
- Slight modifications in simulation.py (e.g. added reverse steering to the simulation, reduced velocity)
- Code refactorizations, comments


## The following files exist:
- host.py --> SERVER/HOST for python sockets.
- main.py --> PRINCIPAL PLACEHOLDER, this file simulates a real car driving and sending sensor data on self-state and cone positions.
- main2.py --> WORLD BUILDER, reconstructs the state and environment of the car from 'sensor data' sent through the network from main.py.
- network.py --> NETWORK/SOCKET settings.
- simulation.py --> CLASSES AND FUNCTION DEFINITIONS for car, cones and data handling.


## LOGIC:
- main.py [data: speed,angle, cone_R, cone_THETA, cone_COLOUR] ==>> host.py
- host.py ==>> main2.py
- main2.py [(speed,angle -> vel_x,vel_y), (cone_data -> cone_pos)]


USEFUL REFERENCE VIDEO: https://www.youtube.com/watch?v=McoDjOCb2Zo&t=2639s   (tech with tim, online multiplayer pygame development)

