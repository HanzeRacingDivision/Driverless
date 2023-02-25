#! /usr/bin/python3
# Based on example code from Adafuit
import web
import time
import board
import busio
import sensorFusion as fusion
from sensorFusion import deltat, MadgwickUpdate
import adafruit_bno08x
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    QuaternionToEuler
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
# bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_SHAKE_DETECTOR)

urls = (
    '/(.*)', 'hello'
)
app = web.application(urls, globals())


class hello:
    def GET(self, name):
        if not name:
            name = 'World'
#        return 'Hello, ' + name + '!'
        
        accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
        acceleration = '"accel":[%0.6f,%0.6f,%0.6f]' % ( accel_x, accel_y, accel_z)
        gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
        gyro = '"gyro":[%0.6f,%0.6f,%0.6f]' % (gyro_x, gyro_y, gyro_z)
        mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
        magnetometer = '"mag":[%0.6f,%0.6f,%0.6f]' % (mag_x, mag_y, mag_z)
        qi, qj, qk, qr = bno.quaternion  # pylint:disable=no-member
        # quaternione = '"quat":[%0.6f,%0.6f,%0.6f,%0.6f]' % (quat_i, quat_j, quat_k, quat_real)

        (game_quat_i, game_quat_j, game_quat_k, game_quat_real) = bno.game_quaternion  # pylint:disable=no-member

        game_quaternione = '"game_quat":[%0.6f,%0.6f,%0.6f,%0.6f]' % (
            game_quat_i, game_quat_j, game_quat_k, game_quat_real)
        deltat = fusion.deltatUpdate()
        #fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
        fusion.MadgwickUpdate(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, deltat);  # else use the magwick

        if bno.shake:
            shake = '"shake": true'
        else:
            shake = '"shake": false'

        #print('game_quaternione: %s, shake: %s ' % (game_quaternione, shake))
        # return '{%s,%s,%s,%s,%s}' % (acceleration, gyro, magnetometer, game_quaternione, shake)
        # return '{"quaternione":[%f,%f,%f]}'%(1.0,1.0,1.0)


if __name__ == "__main__":
    app.run()
