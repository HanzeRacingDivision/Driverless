"""
Extended Kalman Filter SLAM.py example
author: Atsushi Sakai (@Atsushi_twi)
"""

import math
from math import sin, radians, degrees, copysign
import matplotlib.pyplot as plt
import numpy as np
from pygame import Vector2
import time

# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2 # Change in covariance

#  Simulation parameter
Q_sim = np.diag([0.2, np.deg2rad(1.0)])**2  # Sensor Noise
R_sim = np.diag([0.5, np.deg2rad(1.0)])**2  # Process Noise

#DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 100.0  # maximum observation range
M_DIST_TH = 0.5  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

show_animation = True
"""
xEST: belief of last position + LM locations will be (x_car, y_car, theta, x_1, y_1,.... x_n, y_n) depending on the number of LMs
PEst: uncertainty last position -> shape STATE_SIZE * STATE_SIZE // Covariance matrix
u: control applied since last position; should be (v, w) linear + angular velocity
z: measurements of landmarks at the current step; (distance, angle, num_id)
Explanation u(v, w):
Angular velocity: w = change in angle / change in time (how much time passed since last state)
change in angle = angle of car after motion - angle of car before motion
Linear velocity: v = distance / change in time      (if the car was going on straight line how much distance would it have travelled)
"""
def ekf_slam(xEst, PEst, u, z, DT):
    # Predict
    S = STATE_SIZE
    # caluclate Jacobbian for state + motion and Fx -> identity of size STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], u, DT)
    # computes new state/location based on current state and motion
    xEst[0:S] = motion_model(xEst[0:S], u, DT)
    # update covariance / how uncertain are we about the new location
    # after motion
    PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx
    initP = np.eye(2)

    # Update
    for iz in range(len(z[:, 0])):  # for each observation
        min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_lm(xEst)
        if min_id == nLM:
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        lm = get_landmark_position_from_state(xEst, min_id)
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)

        K = (PEst @ H.T) @ np.linalg.inv(S)
        xEst = xEst + (K @ y)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.2  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


# def observation(xTrue, xd, u, RFID, dt):
#     """
#     :param xTrue: the true pose of the system
#     :param xd:    the current noisy estimate of the system
#     :param u:     the current control input
#     :param RFID:  the true position of the landmarks

#     :returns:     Computes the true position, observations, dead reckoning (noisy) position,
#                   and noisy control function
#     """
#     xTrue = motion_model(xTrue, u, dt)

#     # add noise to gps x-y
#     z = np.zeros((0, 3))

#     for i in range(len(RFID[:, 0])):  # Test all beacons, only add the ones we can see (within MAX_RANGE)

#         dx = RFID[i, 0] - xTrue[0, 0]
#         dy = RFID[i, 1] - xTrue[1, 0]
#         d = math.sqrt(dx ** 2 + dy ** 2)
#         angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
#         if d <= MAX_RANGE:
#             dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
#             anglen = angle + np.random.randn() * Q_sim[1, 1]  # add noise
#             zi = np.array([dn, anglen, i])
#             z = np.vstack((z, zi))

#     # add noise to input
#     ud = np.array([[
#         u[0, 0] + np.random.randn() * R_sim[0, 0],
#         u[1, 0] + np.random.randn() * R_sim[1, 1]]]).T

#     xd = motion_model(xd, ud, dt)
#     return xTrue, z, xd, ud
# # changed so it only caluclates z and returns this
def observation(xTrue, u, cones_visible, DT):
    # add noise to gps x-y
    z = np.zeros((0, 3))
    # print("Before update SLAM.py")
    # print(xTrue)

    x_state = motion_model(xTrue, u, DT)

    # print("After update SLAM.py")
    # print(x_state)
    for i in range(len(cones_visible[:, 0])):
        dx = cones_visible[i, 0] - x_state[0, 0]
        dy = cones_visible[i, 1] - x_state[1, 0]
        d = math.hypot(dx, dy)
        if d < MAX_RANGE:
            # angle = radians(cones_visible[i, 2])
            angle = pi_2_pi(math.atan2(dy, dx) - x_state[2, 0])
            dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
            angle_n = angle + np.random.randn() * Q_sim[1, 1]  # add noise
            # dn = d
            # angle_n = angle
            zi = np.array([dn, angle_n, i])
            z = np.vstack((z, zi))
    # print("z")
    # print(z)
    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0],
        u[1, 0] + np.random.randn() * R_sim[1, 1]]]).T

    # xd = motion_model(xd, ud, DT)
    return x_state, z, ud

"""
x: (x,y,theta) previous location + orientation
u: linear + angular velocity
returns new state (x,y,theta)
"""
# def motion_model(x, u, DT):
#     # print("Dims EKF motion")
#     # print(x.shape)
#     # print(x)
#     # print(u.shape)
#     # print(u)
#     F = np.array([[1.0, 0, 0],
#                   [0, 1.0, 0],
#                   [0, 0, 1.0]])
#     B = np.array([[DT * math.cos(x[2, 0]), 0],
#                   [DT * math.sin(x[2, 0]), 0],
#                   [0.0, DT]])
#     x = (F @ x) + (B @ u)
#     return x


def motion_model(prev_pos, motion_commands, dt):
    start_time = time.time()

    car_orientation = prev_pos[2, 0]  # angle in radians
    print("car angel is in begining: ", car_orientation)
    car_velocity = motion_commands[0, 0]
    ang_velo = motion_commands[1, 0]
    motion_vector = Vector2()
    motion_vector.x = car_velocity
    motion_vector.y = 0
    position = Vector2()
    position.x = prev_pos[0, 0]
    position.y = prev_pos[1, 0]
    position += motion_vector.rotate(-car_orientation) * dt
    prev_pos[0, 0] = position.x
    prev_pos[1, 0] = position.y
    prev_pos[2, 0] += ang_velo * dt
    # print('car angle end:', prev_pos[2, 0])

    end_time = time.time()
    # print("Motion model update time:", end_time - start_time)
    return prev_pos

        # self.position += self.velocity.rotate(-self.angle) * dt
        # self.angle += degrees(self.angular_velocity) * dt
def calc_n_lm(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n

"""
calculates jacobbian of moton model
x: state (x,y,theta)
u: linear + angular velocity
returns Jacobian and Fx -> identity of size STATE_SIZE
"""
def jacob_motion(x, u, DT):
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))
    # formula to get first deriviatives (Jacobian)
    jF = np.array([[0.0, 0.0, -DT * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)

    G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx

    return G, Fx,


def calc_landmark_position(x, z):
    zp = np.zeros((2, 1))
    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp


def get_landmark_position_from_state(x, ind):
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_landmark_id(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = calc_n_lm(xAug)

    min_dist = []

    for i in range(nLM):
        # get where the landmark with given number is according to the state vector
        lm = get_landmark_position_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]

    return y, S, H


def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[5.0, 0.0],
                     [10.0, 5.0],
                     [20.0, 10.0],
                     [15.0, 15.0],
                     [10.0, 20.0],
                     [5.0, 15.0],
                     [0.0, 10.0],
                     [0.0, -10.0],
                     [-5.0, -15.0],
                     [-10.0, 20.0],
                     [-15.0, 15.0]])

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)
        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            # plot landmark
            for i in range(calc_n_lm(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b")
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k")
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

#------------------------------------------------------------------#
#-------------Handmade functions for the simulation----------------#
def get_linear_velocity(v_x, v_y):
    return math.sqrt(v_x**2 + v_y**2)

print(get_linear_velocity(2.2, 2.1))
# if __name__ == '__main__':
#     main()







