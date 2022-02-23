import numpy as np


#
# The prediction step of the Extended Kalman Filter
#
def predict(mu, cov, u, Rt):
    n_landmarks = len(mu) - 3

    # This function can possibly be replaced by the information from the car itself,
    # such that we have a new x, y and theta of the vehicle.
    # Define motion model f(mu,u)
    [dtrans, drot1, drot2] = u
    motion = np.array([[dtrans * np.cos(mu[2][0] + drot1)],
                       [dtrans * np.sin(mu[2][0] + drot1)],
                       [drot1 + drot2]])

    # This matrix is used to apply the new motion results to the mu matrix (such that only the first 3 rows are updated)
    F = np.append(np.eye(3), np.zeros((3, n_landmarks)), axis=1)

    # Predict new state
    mu_bar = mu + (F.T).dot(motion)

    # Define motion model Jacobian
    # (derivative of the calculations for the motion.
    J = np.array([[0, 0, -dtrans * np.sin(mu[2][0] + drot1)],
                  [0, 0, dtrans * np.cos(mu[2][0] + drot1)],
                  [0, 0, 0]])
    # create the G matrices (see documentation in the drive)
    G = np.eye(n_landmarks + 3) + (F.T).dot(J).dot(F)

    # Predict new covariance
    cov_bar = G.dot(cov).dot(G.T) + (F.T).dot(Rt).dot(F)

    # print('Predicted location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu_bar[0][0], mu_bar[1][0],
    #                                                                               mu_bar[2][0]))

    # return updated mu and covbar
    return mu_bar, cov_bar


#
# The update step of the Extended Kalman Filter
# Threshold for observed before is currently 1e6 (this number has to be high, as the uncertainty
# has been set to a high number for un
# threshold for static landmark is c_prob >= 0.5. Maybe we can assume all landmarks are static
#
def update(mu, cov, obs, c_prob, Qt):
    N = len(mu)

    for [r, theta, j] in obs:
        j = int(j)
        # if landmark has not been observed before
        if cov[2 * j + 3][2 * j + 3] >= 1e6 and cov[2 * j + 4][2 * j + 4] >= 1e6:
            # define landmark estimate as current measurement
            # aka, use the distance and angle to calculate its x and y location on the map.
            mu[2 * j + 3][0] = mu[0][0] + r * np.cos(theta + mu[2][0])
            mu[2 * j + 4][0] = mu[1][0] + r * np.sin(theta + mu[2][0])

        # if landmark is static
        if c_prob[j] >= 0.5:
            # See documentation for more info here.

            # compute expected observation
            delta = np.array([mu[2 * j + 3][0] - mu[0][0], mu[2 * j + 4][0] - mu[1][0]])
            q = delta.T.dot(delta)
            sq = np.sqrt(q)
            z_theta = np.arctan2(delta[1], delta[0])
            z_hat = np.array([[sq], [z_theta - mu[2][0]]])
            # z_hat = h() function

            # This matrix is used to apply the calculations only to the relevant landmark and used to transfer
            # the low Jacobian into the high Jacobian
            F = np.zeros((5, N))
            F[:3, :3] = np.eye(3)
            # x location of cone j
            F[3, 2 * j + 3] = 1
            # y location of cone j
            F[4, 2 * j + 4] = 1
            # Do the partial diff equations and create the low jacobian
            H_z = np.array([[-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
                            [delta[1], -delta[0], -q, -delta[1], delta[0]]], dtype='float')
            # do the dot product of F to create the high Jacobian
            H = 1 / q * H_z.dot(F)

            # calculate Kalman gain
            K = cov.dot(H.T).dot(np.linalg.inv(H.dot(cov).dot(H.T) + Qt))

            # calculate difference between expected and real observation
            z_dif = np.array([[r], [theta]]) - z_hat
            # normalize angular component!
            z_dif = (z_dif + np.pi) % (2 * np.pi) - np.pi

            # update state vector and covariance matrix
            mu = mu + K.dot(z_dif)
            cov = (np.eye(N) - K.dot(H)).dot(cov)

    # print('Updated location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu[0][0], mu[1][0], mu[2][0]))
    return mu, cov
