from copy import deepcopy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np

from math import sqrt


def HJacobian_at(system_measurement):
    """ compute Jacobian of H matrix at x """
    return array([[system_measurement.d, system_measurement.v, system_measurement.a]])


class FilterBase():
    """ Filter for one set of measurement
    """
    def __init__(self, system_measurement, system_status):
        self.system_measurement = system_measurement
        self.system_status = system_status
        self.last_system_status = deepcopy(system_status)
        self.dt = 1  # 1 second per step

    def update(self):
        self.system_status.d = self.system_measurement.d
        self.system_status.v = self.system_measurement.v
        self.system_status.a = self.system_measurement.a


class Filter(FilterBase):
    """ Filter for one set of measurement
    """
    def __init__(self, system_measurement, system_status):
        super().__init__(system_measurement, system_status)
        rk = ExtendedKalmanFilter(dim_x=3, dim_z=3)
        rk.x = array(self.system_measurement.d,
                     self.system_measurement.v,
                     self.system_measurement.a)
        rk.F = eye(3) + array([[0, 1, 1],
                               [0, 0, 1],
                               [0, 0, 0]]) * self.dt
        range_std = 0.1  # meters, noise range
        rk.R = np.diag([range_std**2])  # Measurement noise matrix
        rk.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=0.1)  # Process noise matrix
        rk.P *= 5  # Covariance matrix

    def update(self):
        rk.update(array([z]), HJacobian_at, hx)
        self.system_status.d = self.system_measurement.d
        self.system_status.v = self.system_measurement.v
        self.system_status.a = self.system_measurement.a


class FrontCarFilter(FilterBase):
    """ Filter for two sets of measurement
    """
    def __init__(self, system_measurement, received_status, system_status):
        super().__init__(system_measurement, system_status)
        self.received_status = received_status

    def update(self):
        self.system_status.d = (
            self.system_measurement.d + self.received_status.d) / 2
        self.system_status.v = (
            self.system_measurement.v + self.received_status.v) / 2
        self.system_status.a = (
            self.system_measurement.a + self.received_status.a) / 2