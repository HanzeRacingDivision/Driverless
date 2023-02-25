import matplotlib.pyplot as plt
import numpy as np
import math
import madgwickahrs
import quaternion
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians
from statistics import mean
from madgwickahrs import MadgwickAHRS

# Notation used coming from: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
def prediction(X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
    X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t))
    P_t = np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose()))) + Q_t
    return X_hat_t, P_t

def update(X_hat_t, P_t, Z_t, R_t, H_t):
    K_prime = P_t.dot(H_t.transpose()).dot(np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
    X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
    P_t = P_t - K_prime.dot(H_t).dot(P_t)
    return X_t, P_t

def rotationMatrix(heading, attitude, bank):
    '''
    :returns: rotation array in numpy format
    [m00 m01 m02]
    [m10 m11 m12]
    [m20 m21 m22]
    '''
    ch = math.cos(heading)
    sh = math.sin(heading)
    ca = math.cos(attitude)
    sa = math.sin(attitude)
    cb = math.cos(bank)
    sb = math.sin(bank)
    m00 = ch * ca
    m01 = sh * sb - ch * sa * cb
    m02 = ch * sa * sb + sh * cb
    m10 = sa
    m11 = ca * cb
    m12 = -ca * sb
    m20 = -sh * ca
    m21 = sh * sa * cb + ch * sb
    m22 = -sh * sa * sb + ch * cb
    return np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]])

def getDistance(lat1, lon1, lat2, lon2):
    '''
    reference: http://code.activestate.com/recipes/577594-gps-distance-and-bearing-between-two-gps-points/
    '''
    R = 6371.0
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance * 1000.0

def getBearing(lat1,lon1,lat2,lon2):
    '''
    reference: http://code.activestate.com/recipes/577594-gps-distance-and-bearing-between-two-gps-points/
    '''
    dLon = lon2 - lon1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2) \
        - sin(lat1) * cos(lat2) * cos(dLon)
    return atan2(y, x)

accXAbsolute = []
accYAbsolute = []

print("Calculating absolute acc values...")
dt = (gpst0 - gpsTime) * 0.000000001
heading = MadgwickAHRS(sampleperiod=mean(dt))
# to get the heading they use the mean variance between the given inputs, filtered in madgwick

gyroscope = [gyro_z, gyro_y, gyro_x]
accelerometer = [accel_z, accel_y, accel_x]
magnetometer = [magZ, magY, magX]
heading.update(gyroscope, accelerometer, magnetometer)
ahrs = heading.quaternion.to_euler_angles()
roll = ahrs[0]
pitch = ahrs[1]
yaw = ahrs[2] + (3.0 * (math.pi / 180.0))  # adding magenetic declination
ACC = np.array([[accel_z], [accel_y], [accel_x]])
ACCABS = np.linalg.inv(rotationMatrix(yaw, pitch, roll)).dot(ACC)
accXAbsolute.append(-1 * ACCABS[0, 0])
accYAbsolute.append(-1 * ACCABS[1, 0])

# Transition matrix
F_t = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
# Initial State cov
P_t = np.identity(4) * 400
# Process cov
Q_t = np.array([[1000, 0, 100, 0], [0, 1000, 0, 100], [100, 0, 1000, 0], [0, 100, 0, 1000]]) * 0.65
# Control matrix
B_t = np.array([[0.5 * dt ** 2, 0, 0, 0], [0, 0.5 * dt ** 2, 0, 0], [0, 0, dt, 0], [0, 0, 0, dt]])
# Control vector
U_t = np.array([[accXAbsolute[0]], [accYAbsolute[0]], [accXAbsolute[0]], [accYAbsolute[0]]])
# Measurment Matrix
H_t = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
# Measurment cov
R_t = np.identity(2)
# Initial State
X_hat_t = np.array([[gpsLongitude], [gpsLatitude], [0], [0]])

Xfilter = []
Yfilter = []

print("Running kalman filter...")
for i in range(len(dt)):
    X_hat_t, P_hat_t = prediction(X_hat_t, P_t, F_t, B_t, U_t, Q_t)  # STATE PREDICTION (IMU)
    Z_t = np.array([[gpsLongitude], [gpsLatitude]])
    Z_t = Z_t.reshape(Z_t.shape[0], -1)
    X_t, P_t = update(X_hat_t, P_hat_t, Z_t, R_t, H_t)  # MEASUREMENT UPDATE (GPS)
    X_hat_t = X_t
    P_hat_t = P_t
    F_t = np.array([[1, 0, dt[i], 0], [0, 1, 0, dt[i]], [0, 0, 1, 0], [0, 0, 0, 1]])
    B_t = np.array([[0.5 * dt[i] ** 2, 0, 0, 0], [0, 0.5 * dt[i] ** 2, 0, 0], [0, 0, dt[i], 0], [0, 0, 0, dt[i]]])
    U_t = np.array([[accXAbsolute[i]], [accYAbsolute[i]], [accXAbsolute[i]], [accYAbsolute[i]]])
    Xfilter.append(X_t[0, 0])
    Yfilter.append(X_t[1, 0])
