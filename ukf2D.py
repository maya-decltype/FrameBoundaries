import numpy as np

#-- Unscented Kalman Filter for 2D tracking ---
#-- Paper: https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf

class UKF2D:
    def __init__(self, x0, dt, process_noise=0.01, measurement_noise=[1000.0, 100.0]):
        self.n = len(x0)  # states: [x, y, vx, vy, ax, ay]
        self.m = 2        # measurements: [x, y]
        self.dt = dt      # time between samples

        # --- UKF Scaling Parameters ---
        self.alpha = 1e-3  # Spread of the sigma points around the mean (typically small, e.g. 1e-3 or 0.1)
        self.kappa = 0     # Secondary scaling parameter (often 0 or 3 - n)
        self.beta = 2      # Incorporates prior knowledge about the distribution (beta = 2 is optimal for Gaussian). 

        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lambda_)  # Scaling factor 

        self.Wm = np.full(2 * self.n + 1, 1 / (2 * (self.n + self.lambda_)))
        self.Wc = np.copy(self.Wm)
        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)

        # Handle scalar or vector input for process noise
        if np.isscalar(process_noise):
            self.Q = np.eye(self.n) * process_noise
        else:
            self.Q = np.diag(process_noise)

        # Handle scalar or vector input for measurement noise
        if np.isscalar(measurement_noise):
            self.R = np.eye(self.m) * measurement_noise
        else:
            self.R = np.diag(measurement_noise)

        self.x = x0 #init state vector  #np.zeros(self.n)
        self.P = np.eye(self.n)

    def f(self, x):
        dt = self.dt
        # Model equations:
        x_new = np.zeros_like(x)
        x_new[0] = x[0] + x[2]*dt + 0.5*x[4]*dt**2  # X[k]  = X[k-1] + Vx[k-1] * dt + 0.5 * Ax[k-1] * dt^2 
        x_new[1] = x[1] + x[3]*dt + 0.5*x[5]*dt**2  # Y[k]  = Y[k-1] + Vy[k-1] * dt + 0.5 * Ay[k-1] * dt^2 
        x_new[2] = x[2] + x[4]*dt                   # Vx[k] = Vx[k-1] + Ax[k-1] * dt
        x_new[3] = x[3] + x[5]*dt                   # Vy[k] = Vy[k-1] + Ay[k-1] * dt
        x_new[4] = x[4]                             # Ax[k] = Ax[k-1]
        x_new[5] = x[5]                             # Ay[k] = Ay[k-1]
        return x_new

    def h(self, x):
        return x[:2]  # measure [x, y] only

    def sigma_points(self, x, P):
        # Compute the scaled square root of the covariance matrix using Cholesky decomposition
        # A is a lower triangular matrix such that A*A'=gamma^2*P
        A = self.gamma * np.linalg.cholesky(P)     
        
        # Initialize the output array for the sigma points (First sigma point is the mean itself):
        points = np.zeros((2 * self.n + 1, self.n)) 
        points[0] = x
        for i in range(self.n):
            points[i + 1] = x + A[i]          # Add the i-th row of A to x to get positive sigma point 
            points[self.n + i + 1] = x - A[i] # Subtruct the i-th row of A to x to get positive sigma point 
        return points

    def predict(self):
        sigmas = self.sigma_points(self.x, self.P)
        sigmas_f = np.array([self.f(s) for s in sigmas])
        x_pred = np.sum(self.Wm[:, None] * sigmas_f, axis=0)

        P_pred = self.Q.copy()
        for i in range(2 * self.n + 1):
            dx = sigmas_f[i] - x_pred
            P_pred += self.Wc[i] * np.outer(dx, dx)

        self.x = x_pred
        self.P = P_pred
        self._sigmas_f = sigmas_f

    def update(self, z):
        sigmas_z = np.array([self.h(s) for s in self._sigmas_f])
        z_pred = np.sum(self.Wm[:, None] * sigmas_z, axis=0)

        P_zz = self.R.copy()
        for i in range(2 * self.n + 1):
            dz = sigmas_z[i] - z_pred
            P_zz += self.Wc[i] * np.outer(dz, dz)

        P_xz = np.zeros((self.n, self.m))
        for i in range(2 * self.n + 1):
            dx = self._sigmas_f[i] - self.x
            dz = sigmas_z[i] - z_pred
            P_xz += self.Wc[i] * np.outer(dx, dz)

        K = P_xz @ np.linalg.inv(P_zz)
        self.x = self.x + K @ (z - z_pred)
        self.P = self.P - K @ P_zz @ K.T

    def step(self, z):
        self.predict()
        self.update(z)
        return self.x.copy()