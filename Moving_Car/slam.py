from path import *

class Slam:
    def __init__(self, matrix_size, car):
        self.mu = np.zeros(matrix_size)
        self.mu[0] = car.true_position.x
        self.mu[1] = car.true_position.y

        self.cov = np.zeros((matrix_size, matrix_size))
        for i in range(3, matrix_size):
            self.cov[i, i] = 10 ** 10

        self.u = np.zeros(3)
        self.Rt = np.array([1e-6, 1e-6, 0])
        self.obs = []
        self.c_prob = []
        self.Qt = np.zeros((2, 2))

    def update_slam_vars(self, visible_left_cones, visible_right_cones, car):
        '''
        This function updates the variables necessary to run SLAM
            - obs
            - c_prob
            - Qt
            - u
        '''
        # updating obs, c_prob, and Qt
        cone_dists = []
        cone_angles = []
        self.obs = []
        self.c_prob = np.ones(len(self.mu))
        for i in range(len(visible_left_cones)):
            observed_car_dist = visible_left_cones[i].true_dist_car + np.random.normal(loc=0, scale=1e-10)
            observed_alpha = visible_left_cones[i].alpha + np.random.normal(loc=0, scale=1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_left_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)

        for i in range(len(visible_right_cones)):
            observed_car_dist = visible_right_cones[i].true_dist_car + np.random.normal(loc=0, scale=1e-10)
            observed_alpha = visible_right_cones[i].alpha + np.random.normal(loc=0, scale=1e-10)

            self.obs.append([observed_car_dist, observed_alpha, visible_right_cones[i].id])
            cone_dists.append(observed_car_dist)
            cone_angles.append(observed_alpha)

        if cone_dists and cone_angles:
            self.Qt[0, 0] = np.std(cone_dists) ** 2
            self.Qt[1, 1] = np.std(cone_angles) ** 2
        self.u = [car.velocity.x, car.angular_velocity, 0]

    def EKF_predict(self, dt):
        """
        The prediction step of the Extended Kalman Filter
        """
        n_landmarks = len(self.mu) - 3

        # This function can possibly be replaced by the information from the car itself,
        # such that we have a new x, y and theta of the vehicle.

        # Define motion model f(mu,u)
        [dtrans, drot1, drot2] = self.u

        motion = np.array([[dt * dtrans * np.cos(radians(self.mu[2]))],  # change in x coordinate
                           [dt * dtrans * -np.sin(radians(self.mu[2]))],  # change in y coordinate
                           [dt * degrees(drot1)]])  # change in theta

        # This matrix is used to apply the new motion results to the mu matrix (such that only the first 3 rows are updated)
        F = np.append(np.eye(3), np.zeros((3, n_landmarks)), axis=1)

        # Define motion model Jacobian
        # (derivative of the calculations for the motion.
        J = np.array([[0, 0, dt * dtrans * -np.sin(radians(self.mu[2]))],
                      [0, 0, dt * dtrans * -np.cos(radians(self.mu[2]))],
                      [0, 0, 0]])

        # create the G matrices (see documentation in the drive)
        G = np.eye(n_landmarks + 3) + F.T.dot(J).dot(F)  # slow line 1

        # Predict new state
        self.mu = self.mu + F.T.dot(motion)[:, 0]

        # Predict new covariance
        R_t = np.zeros((n_landmarks + 3, n_landmarks + 3))
        R_t[0, 0] = self.Rt[0]
        R_t[1, 1] = self.Rt[1]
        R_t[2, 2] = self.Rt[2]
        self.cov = G.dot(self.cov).dot(G.T) + R_t  # slow line 2

        # print('Predicted location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(mu_bar[0][0], mu_bar[1][0],
        #                                                                               mu_bar[2][0])


    def EKF_update(self, car, left_cones, right_cones):
        """
        The update step of the Extended Kalman Filter
        Threshold for observed before is currently 1e6 (this number has to be high, as the uncertainty
        has been set to a high number for un
        threshold for static landmark is c_prob >= 0.5. Maybe we can assume all landmarks are static
        """

        N = len(self.mu)
        for [r, theta, j] in self.obs:
            j = int(j)
            if self.cov[2 * j + 3, 2 * j + 3] >= 1e6 and self.cov[2 * j + 4, 2 * j + 4] >= 1e6:
                # define landmark estimate as current measurement
                # aka, use the distance and angle to calculate its x and y location on the map.
                self.mu[2 * j + 3] = self.mu[0] + r * np.cos(radians(theta) + radians(self.mu[2]))
                self.mu[2 * j + 4] = self.mu[1] + r * -np.sin(radians(theta) + radians(self.mu[2]))

            # if landmark is static
            if self.c_prob[j] >= 0.5:
                # See documentation for more info here.

                # compute expected observation
                delta = np.array([self.mu[2 * j + 3] - self.mu[0], self.mu[2 * j + 4] - self.mu[1]])

                q = delta.T.dot(delta)

                sq = np.sqrt(q)

                z_theta = np.arctan2(delta[1], delta[0])

                z_hat = np.array([[sq], [z_theta - radians(-self.mu[2])]])
                # z_hat = h() function

                # This matrix is used to apply the calculations only to the relevant landmark and used to transfer
                # the low Jacobian into the high Jacobian.
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
                K = self.cov.dot(H.T).dot(np.linalg.inv(H.dot(self.cov).dot(H.T) + self.Qt))

                # calculate difference between expected and real observation
                z_dif = np.array([[r], [radians(-theta)]]) - z_hat
                # normalize angular component!
                z_dif = (z_dif + np.pi) % (2 * np.pi) - np.pi

                # update state vector and covariance matrix
                self.mu = self.mu + K.dot(z_dif)[:, 0]
                self.cov = (np.eye(N) - K.dot(H)).dot(self.cov)

        # using SLAM to update landmark + car positions
        car.position.x = self.mu[0]
        car.position.y = self.mu[1]
        car.angle = self.mu[2]

        for left_cone in left_cones:
            if self.mu[2 * left_cone.id + 3] != 0:
                left_cone.position.x = self.mu[2 * left_cone.id + 3]
            if self.mu[2 * left_cone.id + 4] != 0:
                left_cone.position.y = self.mu[2 * left_cone.id + 4]

        for right_cone in right_cones:
            if self.mu[2 * right_cone.id + 3] != 0:
                right_cone.position.x = self.mu[2 * right_cone.id + 3]
            if self.mu[2 * right_cone.id + 4] != 0:
                right_cone.position.y = self.mu[2 * right_cone.id + 4]

        # print('Updated location\t x: {0:.2f} \t y: {1:.2f} \t theta: {2:.2f}'.format(self.mu[0], self.mu[1], self.mu[2]))

    def run(self, car, visible_left_cones, visible_right_cones, left_cones, right_cones, dt):
        self.update_slam_vars(visible_left_cones, visible_right_cones, car)
        self.EKF_predict(dt)
        self.EKF_update(car, left_cones, right_cones)
