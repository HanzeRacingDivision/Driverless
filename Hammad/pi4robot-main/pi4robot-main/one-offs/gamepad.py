import sys
sys.path.append("..") #Hack, doesn't work in VS Code debug.  Replace? #"inputs" is on PyPI.
from GamepadLib import Gamepad
import time
#Test it works with cv2 at the same time...
import cv2

# Gamepad settings
gamepadType = Gamepad.PS4
buttonExit = 'PS' #Right stick click.
joystickSpeed = 'LEFT-Y'
joystickSteering = 'RIGHT-X'
pollInterval = 0.1

def steering_control():
    # Wait for a connection
    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')

    # Start the background updating
    gamepad.startBackgroundUpdates()

    # Joystick events handled in the background
    try:
        while gamepad.isConnected():
            # Check for the exit button
            if gamepad.beenPressed(buttonExit):
                print('EXIT')
                break

            # Update the joystick positions
            # Speed control (inverted)
            speed = -gamepad.axis(joystickSpeed)
            # Steering control (not inverted)
            steering = gamepad.axis(joystickSteering)
            print('%+.1f %% speed, %+.1f %% steering' % (speed * 100, steering * 100))

            # Sleep for our polling interval
            time.sleep(pollInterval)
    finally:
        # Ensure the background thread is always terminated when we are done
        gamepad.disconnect()


if __name__ == '__main__':
    steering_control()