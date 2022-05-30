import serial.tools.list_ports

print("printing serial ports:")
[print(entry) for entry in serial.tools.list_ports.comports()]
print("done printing ports.")
print()