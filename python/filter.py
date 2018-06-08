from copy import deepcopy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter,UnscentedKalmanFilter, MerweScaledSigmaPoints
from numpy import eye, array, asarray
import numpy as np
from numpy.linalg import inv

from math import sqrt




class FilterBase():
    """ Filter for one set of measurement
    This is the base class, no filtering is actually applied.
    This can be used to check the performance of the filter.
    """
    def __init__(self, system_measurement, system_status, delta_t, range_std):
        self.system_measurement = system_measurement
        self.system_status = system_status
        self.delta_t = delta_t  # 0.01 second per step
        if range_std <= 0:
            self.range_std = 0.001
        else:
            self.range_std = range_std
        points = MerweScaledSigmaPoints(n=3, alpha=.1, beta=2., kappa=-1.)
        self.kf = UnscentedKalmanFilter(3, 3, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        self.Q_factor = 5
        self.Q_factor_power = 0
        self.kf.Q = Q_discrete_white_noise(3, dt=self.delta_t, var=0.00001*(self.Q_factor**self.Q_factor_power))
        self.kf.R = np.diag([self.range_std**2,self.range_std**2,self.range_std**2])
        self.kf.x = array([float(self.system_measurement.d),
                           float(self.system_measurement.v),
                           float(self.system_measurement.a)])
        self.kf.P *= 0.00001  # Covariance matrix

    def fx(self, x, dt):
        """State transition"""
        F = eye(3) + array([[0, 1, 0.5],
                            [0, 0, 1],
                            [0, 0, 0]]) * dt
        return np.dot(F, x)


    def hx(self, x):
        """State to measurement"""
        return x

    def update_Q(self):
        y = self.kf.y
        S = self.kf.P + self.kf.R
        self.eps = np.dot(y.T, inv(S)).dot(y)
        if self.eps > 3 and self.Q_factor_power <=10:
            self.Q_factor_power += 1
            self.kf.Q *= self.Q_factor
        elif self.eps <= 3 and self.Q_factor_power > 0:
            self.Q_factor_power -= 1
            self.kf.Q /= self.Q_factor


    def update(self):
        self.system_status.d = self.system_measurement.d
        self.system_status.v = self.system_measurement.v
        self.system_status.a = self.system_measurement.a


class Filter(FilterBase):
    """ Filter for one set of measurement
    Unscented Kalman Filter:
    https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
    Adjustable Process Noise for maneuvering detection:
    https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/14-Adaptive-Filtering.ipynb
    """

    def update(self):
        z = array([float(self.system_measurement.d),
                   float(self.system_measurement.v),
                   float(self.system_measurement.a)])
        try:
            self.kf.predict()
        except:
            self.kf.P = eye(3)*0.00001
            self.kf.predict()
        self.kf.update(z)
        self.update_Q()
        self.system_status.d = float(self.kf.x[0])
        self.system_status.v = float(self.kf.x[1])
        self.system_status.a = float(self.kf.x[2])


class FrontCarFilter(FilterBase):
    """ Filter for two sets of measurement
    Sensor Fusion:
    https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
    """
    def __init__(self, system_measurement, received_status, system_status, delta_t, range_std):
        super().__init__(system_measurement, system_status, delta_t, range_std)
        self.received_status = received_status
        points = MerweScaledSigmaPoints(n=3, alpha=.1, beta=2., kappa=-1.)
        self.kf = UnscentedKalmanFilter(3, 6, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        self.Q_factor = 5
        self.Q_factor_power = 0
        self.kf.Q = Q_discrete_white_noise(3, dt=self.delta_t, var=0.00001*(self.Q_factor**self.Q_factor_power))
        self.kf.R = np.diag([self.range_std**2,self.range_std**2,self.range_std**2,self.range_std**2,self.range_std**2,self.range_std**2])
        self.kf.x = array([float(self.system_measurement.d),
                           float(self.system_measurement.v),
                           float(self.system_measurement.a)])
        self.kf.P *= 0.00001  # Covariance matrix
        self.H = np.concatenate((eye(3),eye(3)))
        self.HT = np.concatenate((eye(3),eye(3)),1)

    def hx(self, x):
        """State to measurement"""
        return np.concatenate((x,x))

    def update_Q(self):
        y = self.kf.y
        S = np.dot(self.H,self.kf.P).dot(self.HT) + self.kf.R
        self.eps = np.dot(y.T, inv(S)).dot(y)
        if self.eps > 3 and self.Q_factor_power <=10:
            self.Q_factor_power += 1
            self.kf.Q *= self.Q_factor
        elif self.eps <= 3 and self.Q_factor_power > 0:
            self.Q_factor_power -= 1
            self.kf.Q /= self.Q_factor

    def update(self):
        z = array([float(self.system_measurement.d),
                   float(self.system_measurement.v),
                   float(self.system_measurement.a),
                   float(self.received_status.d),
                   float(self.received_status.v),
                   float(self.received_status.a)])
        try:
            self.kf.predict()
        except:
            #print('p',self.kf.P)
            self.kf.P = eye(3)*0.00001
            self.kf.predict()
        try:
            self.kf.update(z)
        except Exception as e:
            #print('z',z)
            pass
        self.update_Q()
        self.kf.P += np.transpose(self.kf.P)
        self.kf.P /= 2
        self.system_status.d = float(self.kf.x[0])
        self.system_status.v = float(self.kf.x[1])
        self.system_status.a = float(self.kf.x[2])
