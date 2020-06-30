import random
import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
from numpy import zeros, dot, eye
from copy import deepcopy
from filterpy.stats import logpdf
from filterpy.common import pretty_str, reshape_z


class ExtendedKalmanFilter(object):
    def __init__(self, dim_x=3, dim_z=3, dim_u=2, init_state=zeros((3, 1))):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = init_state  # state
        self.P = eye(dim_x)  # uncertainty covariance
        self.R = eye(dim_z)  # state uncertainty
        self.Q = eye(dim_x)  # process uncertainty
        self.y = zeros((dim_z, 1))  # residual

        z = np.array([None] * self.dim_z)
        self.z = reshape_z(z, self.dim_z, self.x.ndim)

        self.FJacobian = np.eye(self.dim_x)
        self.HJacobian = np.eye(self.dim_z)

        # identity matrix. Do not alter this.
        self._I = np.eye(dim_x)

        self._log_likelihood = math.log(sys.float_info.min)
        self._likelihood = sys.float_info.min
        self._mahalanobis = None

        # these will always be a copy of x,P after predict() is called
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # these will always be a copy of x,P after update() is called
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def predict_update(self, u, z):
        self.predict(u)
        self.update(z)
        return self.x_post

    def predict(self, u):
        # self.x_prior = self.Fx(self.x, u)
        self.P_prior = np.dot(self.FJacobian, np.dot(self.P, self.FJacobian.T)) + self.Q
        self.x = self.x_prior.copy()
        self.P = self.P_post.copy()
        return self.x_prior

    def update(self, z):
        # Innovation or measurement residual
        self.y = z - np.dot(self.HJacobian, self.x_prior)
        # Innovation (or residual) covariance
        S = np.dot(self.HJacobian, np.dot(self.P_prior, self.HJacobian.T)) + self.R
        # Near-optimal Kalman gain
        K = np.dot(self.P_prior, np.dot(self.HJacobian.T, linalg.inv(S)))
        # Update state estimate
        self.x_post = self.x_prior + np.dot(K, self.y)
        # Update covariance estimate
        self.P_post = np.dot((self._I - np.dot(K, self.HJacobian)), self.P_prior)
        # Save result
        self.x = self.x_post.copy()
        self.P = self.P_post.copy()
        pass

    def Fx(self, x, u):
        # Transition function
        d_r = u.reshape(2)[0]
        d_l = u.reshape(2)[1]
        d_theta = (d_r - d_l) / 0.54
        # print("receieved dl = %.3f,  dr = %.3f" % (d_l, d_r))

        # This is the common Odometry model
        X = x[0][0]
        Y = x[1][0]
        THETA = x[2][0]
        X_post = X + (d_l+d_r)/2*math.cos(THETA + d_theta)
        Y_post = Y + (d_l+d_r)/2*math.sin(THETA + d_theta)
        THETA_post = d_theta + THETA

        # # This is our own odometry model
        # X = -x[1][0]
        # Y = x[0][0]
        # THETA = x[2][0]
        # Radius = 0.0
        # dx, dy = 0.0, 0.0
        # if d_theta:
        #     # print("odo:",self.d_l,self.d_r)
        #     if (d_l + d_r) == 0:  # 原地转向或静止
        #         Radius = 0
        #     else:
        #         if d_l * d_r > 0:  # 转向中心在walker之外
        #             Radius = min(abs(d_l / d_theta), abs(d_r / d_theta)) + 0.27
        #         else:  # 转向中心在walker之内
        #             Radius = 0.27 - min(abs(d_l / d_theta), abs(d_r / d_theta))
        # else:
        #     # print('No Turning!')
        #     Radius = 0
        #     pass
        #
        # # 计算坐标变化dX, dY
        # if d_l == d_r:  # 直行
        #     dx = 0.0
        #     dy = d_l
        # else:
        #     if (d_l + d_r) == 0:  # 原地转向或静止
        #         dx, dy = 0.0, 0.0
        #     elif abs(d_l) > abs(d_r) and d_l > 0:  # 右前
        #         dx = Radius * (1 - math.cos(abs(d_theta)))
        #         dy = Radius * math.sin(abs(d_theta))
        #     elif abs(d_l) > abs(d_r) and d_l <= 0:  # 右后
        #         dx = Radius * (1 - math.cos(abs(d_theta)))
        #         dy = -Radius * math.sin(abs(d_theta))
        #     elif abs(d_r) > abs(d_l) and d_r > 0:  # 左前
        #         dx = Radius * (math.cos(abs(d_theta)) - 1)
        #         dy = Radius * math.sin(abs(d_theta))
        #     elif abs(d_r) > abs(d_l) and d_r <= 0:  # 左后
        #         dx = Radius * (math.cos(abs(d_theta)) - 1)
        #         dy = -Radius * math.sin(abs(d_theta))
        # # print('dx=', self.dx, 'm;  dy=', self.dy, 'm;  dθ=', self.d_theta / math.pi * 180, '°')
        #
        # X += dx * math.cos(THETA) - dy * math.sin(THETA)
        # Y += dx * math.sin(THETA) + dy * math.cos(THETA)
        #
        # X_post = Y
        # Y_post = -X
        # THETA_post = THETA + d_theta

        return np.array([[X_post],
                         [Y_post],
                         [THETA_post]])

    # @property
    # def log_likelihood(self):
    #     """
    #     log-likelihood of the last measurement.
    #     """
    #
    #     if self._log_likelihood is None:
    #         self._log_likelihood = logpdf(x=self.y, cov=self.S)
    #     return self._log_likelihood
    #
    # @property
    # def likelihood(self):
    #     """
    #     Computed from the log-likelihood. The log-likelihood can be very
    #     small,  meaning a large negative value such as -28000. Taking the
    #     exp() of that results in 0.0, which can break typical algorithms
    #     which multiply by this value, so by default we always return a
    #     number >= sys.float_info.min.
    #     """
    #     if self._likelihood is None:
    #         self._likelihood = math.exp(self.log_likelihood)
    #         if self._likelihood == 0:
    #             self._likelihood = sys.float_info.min
    #     return self._likelihood
    #
    # @property
    # def mahalanobis(self):
    #     """
    #     Mahalanobis distance of innovation. E.g. 3 means measurement
    #     was 3 standard deviations away from the predicted value.
    #
    #     Returns
    #     -------
    #     mahalanobis : float
    #     """
    #     if self._mahalanobis is None:
    #         self._mahalanobis = math.sqrt(float(dot(dot(self.y.T, self.SI), self.y)))
    #     return self._mahalanobis
    #
    # def __repr__(self):
    #     return '\n'.join([
    #         'KalmanFilter object',
    #         pretty_str('x', self.x),
    #         pretty_str('P', self.P),
    #         pretty_str('x_prior', self.x_prior),
    #         pretty_str('P_prior', self.P_prior),
    #         pretty_str('F', self.F),
    #         pretty_str('Q', self.Q),
    #         pretty_str('R', self.R),
    #         pretty_str('K', self.K),
    #         pretty_str('y', self.y),
    #         pretty_str('S', self.S),
    #         pretty_str('likelihood', self.likelihood),
    #         pretty_str('log-likelihood', self.log_likelihood),
    #         pretty_str('mahalanobis', self.mahalanobis)
    #         ])


if __name__ == "__main__":

    ekf = ExtendedKalmanFilter(dim_x=3, dim_u=2, dim_z=3, init_state=np.array([[0],
                                                                               [0],
                                                                               [0]]))

    pass
