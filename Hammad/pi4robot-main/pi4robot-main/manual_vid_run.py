from Robot import Robot
from RobotLoop import RobotLoop
from GamepadLib import Gamepad
import RPi.GPIO as GPIO
import numpy as np
import cv2
import time


def get_gamepad(rState, wState, context):
    speed, steer = (0, 0)
    # Gamepad settings
    gamepadType = Gamepad.PS4
    joystickSpeed = 'LEFT-Y'
    joystickSteering = 'RIGHT-X'
    if Gamepad.available():
        gamepad = gamepadType()
        gamepad.startBackgroundUpdates()
        try:
            if gamepad.isConnected():
                # Update the joystick positions
                # Speed control (inverted)
                speed = -gamepad.axis(joystickSpeed)
                # Steering control (not inverted)
                steer = gamepad.axis(joystickSteering)
        finally:
            # Ensure the background thread is always terminated when we are done
            gamepad.disconnect()
    action = {"speed": speed, "steer": steer}
    return action


GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
throttle = GPIO.PWM(18, 1000)
steering = GPIO.PWN(19, 1000)
throttle.start(0)
steering.start(50)


def perform_actions(actions, context):
    throttle.ChangeDutyCycle(actions["speed"])
    #map the steering values into 0-100 pwm values.
    pwm_steer = np.interp(actions["steer"], [-100, 100], [0, 100])
    steering.ChangeDutyCycle(pwm_steer)
    print(f"TAKING ACTIONS: Throttle: {actions['speed']}\tSteer: {pwm_steer}")


def get_frame(rState, wState, context):
    cam = context['camera']
    ret, frame = cam.read()
    return {}


def do_robot():
    robot: Robot = Robot()
    loop: RobotLoop = RobotLoop()
    loop.actuationHandler = perform_actions
    GPIO.setmode(GPIO.BOARD)
    context = {
        "iteration": 0,
        "camera": cv2.VideoCapture(0)
    }
    iteration = 0
    stop = False
    while not stop:
        rs2, ws2, actions = loop.loop_step(robot.RobotState, robot.WorldState, context, [], [get_gamepad])
        print(f'COMPUTED ACTIONS: {actions}')
        time.sleep(1)
        iteration += 1
    context['camera'].release() 
    cv2.destroyAllWindows()


if __name__ == '__main__':
    do_robot()
    GPIO.cleanup()
