import numpy as np
import random
import socket
import json
from time import time, sleep
from typing import Dict, Any

IP = "127.0.0.1"
PORT = 5000


def generate_json_message() -> None:
	# DV driving dynamics 1
	speed_actual = random.randint(0,80)
	speed_target = random.randint(0,80)
	steering_angle_actual = random.randint(-100, 100)
	steering_angle_target = random.randint(-100, 100)
	brake_hydr_actual = random.randint(0,100)
	brake_hydr_target = random.randint(0,100)
	motor_moment_actual = random.randint(0,100)
	motor_moment_target = random.randint(0,100)

	#DV driving dynamics 2
	acceleration_longitudinal = random.randint(-32768,32767)
	acceleration_lateral = random.randint(-32768,32767)
	yaw_rate = random.randint(-32768,32767)

	#DV system status
	as_state = random.randint(1,5)
	ebs_state = random.randint(1,3)
	ami_state = random.randint(1,6)
	steering_state = random.randint(0,1)
	service_brake_state = random.randint(1,3)
	lap_counter = random.randint(0,15)
	cones_count_actual = random.randint(0,255)
	cones_count_all = random.randint(0,65535)

	vals_dict = {"speed_actual": speed_actual,
				 "speed_target": speed_target,
				 "steering_angle_actual": steering_angle_actual,
				 "steering_angle_target": steering_angle_target,
				 "brake_hydr_actual": brake_hydr_actual,
				 "brake_hydr_target": brake_hydr_target,
				 "motor_moment_actual": motor_moment_actual,
				 "motor_moment_target": motor_moment_target,
				 "acceleration_longitudinal": acceleration_longitudinal,
				 "acceleration_lateral": acceleration_lateral,
				 "yaw_rate": yaw_rate,
				 "as_state": as_state,
				 "ebs_state": ebs_state,
				 "ami_state": ami_state,
				 "steering_state": steering_state,
				 "service_brake_state": service_brake_state,
				 "lap_counter": lap_counter,
				 "cones_count_actual": cones_count_actual,
				 "cones_count_all": cones_count_all
				}
	return vals_dict


def send_json_message(
    sock: socket.socket,
    json_message: Dict[str, Any],
) -> None:
    """Send json packet to server"""
    message = (json.dumps(json_message) + '\n').encode()
    sock.sendall(message)
    #print(f'{len(message)} bytes sent')

def main() -> None:
    with socket.socket() as sock:
        sock.connect((IP, PORT))
        while True:
            json_message = generate_json_message()
            send_json_message(sock, json_message)
            sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass



