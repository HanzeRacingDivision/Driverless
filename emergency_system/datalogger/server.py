from socketserver import StreamRequestHandler, TCPServer
import can
import json
import numpy as np

#This is a virtual CAN Bus interface
#Replace this by the real CAN bus interface
bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)
IP = '127.0.0.1'
PORT = 5000

def reverse_mask(x):
    x = ((x & 0x55555555) << 1) | ((x & 0xAAAAAAAA) >> 1)
    x = ((x & 0x33333333) << 2) | ((x & 0xCCCCCCCC) >> 2)
    x = ((x & 0x0F0F0F0F) << 4) | ((x & 0xF0F0F0F0) >> 4)
    return x


def create_can_messages(data: bytes) -> None:
    data = json.loads(data)
    # DV driving dynamics 1
    speed_actual = data["speed_actual"].to_bytes(1,byteorder='little')
    speed_target = data["speed_target"].to_bytes(1,byteorder='little')
    steering_angle_actual = data["steering_angle_actual"].to_bytes(1,byteorder='little', signed=True)
    steering_angle_target = data["steering_angle_target"].to_bytes(1,byteorder='little', signed=True)
    brake_hydr_actual = data["brake_hydr_actual"].to_bytes(1,byteorder='little')
    brake_hydr_target = data["brake_hydr_target"].to_bytes(1,byteorder='little')
    motor_moment_actual = data["motor_moment_actual"].to_bytes(1,byteorder='little', signed=True)
    motor_moment_target = data["motor_moment_target"].to_bytes(1,byteorder='little', signed=True) 

    # DV driving dynamics 2
    acceleration_longitudinal = data["acceleration_longitudinal"].to_bytes(2,byteorder='little', signed=True)
    acceleration_lateral = data["acceleration_lateral"].to_bytes(2,byteorder='little', signed=True)
    yaw_rate = data["yaw_rate"].to_bytes(2,byteorder='little', signed=True)

    # DV system status
    # Reverse all the bits so they can be pasted after each other
    as_state = reverse_mask(data["as_state"])
    ebs_state = reverse_mask(data["ebs_state"])
    ami_state = reverse_mask(data["ami_state"])
    # Bit shift so that the first three bits are as_state and the subsequent two bits are ebs_state
    first_byte = (as_state << 3) | ebs_state
    # Remove excess bits at the back
    first_byte = first_byte >> 3
    # Add ami_state at the back
    first_byte = (first_byte << 3) | ami_state
    # Remvoe excess bits
    first_byte = first_byte >> 3

    # Reverse all the bits
    steering_state = reverse_mask(data["steering_state"])
    service_brake_state = reverse_mask(data["service_brake_state"])
    lap_counter = reverse_mask(data["lap_counter"])
    # Adding the values to the second byte
    second_byte = (steering_state << 1) | service_brake_state
    second_byte = (second_byte << 1) | lap_counter
    second_byte = second_byte >> 3
    second_byte = (second_byte << 1) | (data["cones_count_actual"]>>7) #first bit of cones actual
    
    third_byte = (data["cones_count_actual"] << 1) >> 1 # last seven of cones actual
    third_byte = (third_byte) | (data["cones_count_all"] >> 15) #first bit of cones all 
    bit_removed = (data["cones_count_all"] >> 1) << 1 #remove first bit from cones all
    fourth_byte = (bit_removed >> 8) #get first 8 bits onto fourth byte
    mask = (1 << 8) - 1 #mask te get final 8 bits
    fifth_byte = (bit_removed & mask) #get final 8 bits on fifth byte

    # Turning values into bytes in little-endian format
    first_byte = first_byte.to_bytes(1,byteorder='little')
    second_byte = second_byte.to_bytes(1,byteorder='little')
    third_byte = third_byte.to_bytes(1,byteorder='little')
    fourth_byte = fourth_byte.to_bytes(1,byteorder='little')
    fifth_byte = fifth_byte.to_bytes(1,byteorder='little')

    data_dd1 = bytearray()

    # Adding byte values to bytearray
    data_dd1.extend(speed_actual)
    data_dd1.extend(speed_target)
    data_dd1.extend(steering_angle_actual)
    data_dd1.extend(steering_angle_target)
    data_dd1.extend(brake_hydr_actual)
    data_dd1.extend(brake_hydr_target)
    data_dd1.extend(motor_moment_actual)
    data_dd1.extend(motor_moment_target)

    data_dd2 = bytearray()

    data_dd2.extend(acceleration_longitudinal)
    data_dd2.extend(acceleration_lateral)
    data_dd2.extend(yaw_rate)

    data_ss = bytearray()

    data_ss.extend(first_byte)
    data_ss.extend(second_byte)
    data_ss.extend(third_byte)
    data_ss.extend(fourth_byte)
    data_ss.extend(fifth_byte)

    # CAN messages
    driving_dynamics1 = can.Message(
        arbitration_id = 0x500, data= data_dd1
    )

    driving_dynamics2 = can.Message(
        arbitration_id = 0x501, data = data_dd2
    )

    system_status = can.Message(
        arbitration_id = 0x502, data = data_ss
    )

    try:
        bus.send(driving_dynamics1)
        bus.send(driving_dynamics2)
        bus.send(system_status)
    except can.CanError:
        print("Message not sent")    


# This class handles receiving JSON packets from the transmitter and calls create_can_messages
class DumpHandler(StreamRequestHandler):
    def handle(self) -> None:
        """receive json packets from client"""
        print('connection from {}:{}'.format(*self.client_address))
        try:
            while True:
                data = self.rfile.readline()
                if not data:
                    break
                create_can_messages(data)
        finally:
            print('disconnected from {}:{}'.format(*self.client_address))

def main() -> None:
    server_address = (IP, PORT)
    print('starting up on {}:{}'.format(*server_address))
    with TCPServer(server_address, DumpHandler) as server:
        print('waiting for a connection')
        server.serve_forever()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
