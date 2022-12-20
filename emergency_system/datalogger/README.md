# Driverless Code Design Report

## INTRODUCTION
The datalogger is a device which has to send values specified in the FS competition handbook over a CAN bus. A data logger provided by the competition officials will be mounted to the CAN bus which gathers these values.


## RULES AND DEMANDS
Besides the messages from the RES three different messages have to be sent by the datalogger. The message specifications can be found in the competition handbook on page 16/19 in Table 4. The values have to be in the litte-endian format. These messages have to be sent every 100ms

## RESEARCH
Three different options were considered a Kvaser Memorater (UART), an Arduino Uno(UART) and a Raspberry Pi (WebSockets).
The Kvaser Memorater was not chosen because of software from Kvaser had to be used for development. Learning this software would take time and resources which I was not willing to spend. The Arduino Uno over UART was not chosen because it was reported by a team member that writing from a PC to the Arduino Uno could take more time than 100ms which is less than ideal since these messages have to be sent every 100ms.

The Raspberry Pi using Websockets was chosen because for the telemetry system a router will be installed which can provide a LAN network (A LAN network can also be provided by connecting the Raspberry Pi and the device that sends the data necessary with a Ethernet Crossover cable)

## CODING PROCESS
Two Python scripts were written. One transmitting script and one receiving script. The transmitting script sends mockdata in JSON format to the receiving script which runs on a Raspberry Pi.
The receiving script turns these values into bytes in the little-endian format and puts them into a bytearray. This bytearray is passed to the Python-can library which sends these messages over the CAN-bus using SocketCAN which is included in the Linux Kernel.

## TESTING
Practical tests were executed. Values were chosen beforehand and sent over the virtual CAN bus. These values were then checked if they were as expected. When a value was different than expected the problem was investigated and identified. Changes were applied and testing restarted until everything was in order.

## CONLCUSION/DISCUSSION
If the device which produces the values specified in the FS competition handbook runs on Linux, a websocket connection is unnecessary since the data can just be sent over the CAN bus using SocketCAN. A CAN interface will have to be attached to this device however. For the Raspberry Pi this is rather inexpensive. Other devices will have something similar. Another option is a USB to CAN adapter. These devices are expensive but Kvaser provides one and they are always very open to sponsorship.
A list with all the necessary components and system structure can be found on the Drive