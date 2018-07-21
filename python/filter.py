#!/usr/bin/env python3
from status import Status
from acceleration_bound import acceleration_bound
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from numpy import eye, array, asarray
import numpy as np
from copy import deepcopy


class FilterBase(object):
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        self.vehicle.estimated_status = deepcopy(self.vehicle.ground_truth)

    def predict(self):
        pass


class FollowingVehicleFilterBase(FilterBase):
    """docstring for NoFilter"""
        
    def update(self):
        self.vehicle.front_estimated_status = Status(self.vehicle.z[0],self.vehicle.z[1],self.vehicle.z[2])
        self.vehicle.estimated_status = Status(self.vehicle.z[3],self.vehicle.z[4],self.vehicle.z[5])
        self.vehicle.following_estimated_status = Status(self.vehicle.z[6],self.vehicle.z[7],self.vehicle.z[8])


class LastVehicleFilterBase(FilterBase):
    """docstring for NoFilter"""
        
    def update(self):
        self.vehicle.front_estimated_status = Status(self.vehicle.z[0],self.vehicle.z[1],self.vehicle.z[2])
        self.vehicle.estimated_status = Status(self.vehicle.z[3],self.vehicle.z[4],self.vehicle.z[5])



class FollowingVehicleAverage(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(FollowingVehicleAverage, self).__init__(vehicle)
        self.H = self.vehicle.H

    def update(self):
        z = self.vehicle.z
        self.vehicle.front_estimated_status = Status(*(z[:3]+\
            self.H.dot(z[13:16])+\
            z[3:6]+np.concatenate((z[9:11],[0]))+\
            z[6:9]+np.concatenate((z[9:11],[0]))+np.concatenate((z[11:13],[0]))+\
            self.H.dot(z[16:19])+np.concatenate((z[9:11],[0]))+np.concatenate((z[11:13],[0])))/5)

        self.vehicle.estimated_status = Status(*(z[3:6]+\
            z[:3]-np.concatenate((z[9:11],[0]))+\
            z[6:9]+np.concatenate((z[11:13],[0]))+\
            self.H.dot(z[13:16])-np.concatenate((z[9:11],[0]))+\
            self.H.dot(z[16:19])+np.concatenate((z[11:13],[0])))/5)


        self.vehicle.following_estimated_status = Status(*(z[6:9]+\
            self.H.dot(z[16:19])+\
            z[3:6]-np.concatenate((z[11:13],[0]))+\
            z[:3]-np.concatenate((z[9:11],[0]))-np.concatenate((z[11:13],[0]))+\
            self.H.dot(z[13:16])-np.concatenate((z[9:11],[0]))-np.concatenate((z[11:13],[0])))/5)



class LastVehicleFilterAverage(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(LastVehicleFilterAverage, self).__init__(vehicle)
        self.H = self.vehicle.H

    def update(self):
        z = self.vehicle.z
        self.vehicle.front_estimated_status = Status(*(z[:3]+(z[3:6]+np.concatenate((z[6:8],[0])))+self.H.dot(z[8:]))/3)

        self.vehicle.estimated_status = Status(*(z[3:6]+(z[:3]-np.concatenate((z[6:8],[0])))+(self.H.dot(z[8:])-np.concatenate((z[6:8],[0]))))/3)


class FollowingVehicleNoHLAverage(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(FollowingVehicleNoHLAverage, self).__init__(vehicle)
        self.H = self.vehicle.H

    def _f(self,list_):
        return np.mean(np.sort(list_, axis=0)[1:-1,:], axis=0)


    def update(self):
        z = self.vehicle.z
        _z = [z[:3],
            self.H.dot(z[13:16]),
            z[3:6]+np.concatenate((z[9:11],[0])),
            z[6:9]+np.concatenate((z[9:11],[0]))+np.concatenate((z[11:13],[0])),
            self.H.dot(z[16:19])+np.concatenate((z[9:11],[0]))+np.concatenate((z[11:13],[0]))]

        self.vehicle.front_estimated_status = Status(*self._f(_z))


        _z = [z[3:6],
            z[:3]-np.concatenate((z[9:11],[0])),
            z[6:9]+np.concatenate((z[11:13],[0])),
            self.H.dot(z[13:16])-np.concatenate((z[9:11],[0])),
            self.H.dot(z[16:19])+np.concatenate((z[11:13],[0]))]

        self.vehicle.estimated_status = Status(*self._f(_z))


        _z = [z[6:9],
            self.H.dot(z[16:19]),
            z[3:6]-np.concatenate((z[11:13],[0])),
            z[:3]-np.concatenate((z[9:11],[0]))-np.concatenate((z[11:13],[0])),
            self.H.dot(z[13:16])-np.concatenate((z[9:11],[0]))-np.concatenate((z[11:13],[0]))]

        self.vehicle.following_estimated_status = Status(*self._f(_z))


class LastVehicleFilterNoHLAverage(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(LastVehicleFilterNoHLAverage, self).__init__(vehicle)
        self.H = self.vehicle.H

    def _f(self,list_):
        return np.mean(np.sort(list_, axis=0)[1:-1,:], axis=0)

    def update(self):
        z = self.vehicle.z
        _z = [z[:3],z[3:6]+np.concatenate((z[6:8],[0])),self.H.dot(z[8:])]
        self.vehicle.front_estimated_status = Status(*self._f(_z))

        _z = [z[3:6],z[:3]-np.concatenate((z[6:8],[0])),self.H.dot(z[8:])-np.concatenate((z[6:8],[0]))]
        self.vehicle.estimated_status = Status(*self._f(_z))





class FollowingVehicleUKF(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(FollowingVehicleUKF, self).__init__(vehicle)
        self.H = self.vehicle.H
        self.invH = self.vehicle.invH
        self.delta_t = self.vehicle.delta_t
        noises2 = self.vehicle.observation_noises.sigma**2
        for i in range(len(noises2)):
            if noises2[i] == 0.:
                noises2[i] = 1e-16
        points = MerweScaledSigmaPoints(n=9, alpha=.1, beta=2., kappa=0.1)
        self.kf = UnscentedKalmanFilter(9, 19, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        self.Q_factor = 10
        self.Q_factor_power = 1
        self.kf.Q = Q_discrete_white_noise(3, dt=0.1, var=.1, block_size=3)
        self.kf.R = np.diag(noises2)
        self.kf.x = np.concatenate((self.vehicle.ground_truth,self.vehicle.ground_truth,self.vehicle.ground_truth))
        self.kf.x[0] += self.vehicle.safty_dist
        self.kf.x[6] -= self.vehicle.safty_dist
        self.kf.P *= .1
        self.h = np.concatenate((eye(6),eye(6)))[:11]
        self.ht = np.concatenate((eye(6),np.concatenate((eye(5),np.zeros([1,5])))),1)

    def fx(self, x, dt):
        """State transition"""
        x[5] = self.vehicle.controller.next
        return np.concatenate((self.H.dot(x[:3]),self.H.dot(x[3:6]),self.H.dot(x[6:9])))


    def hx(self, x):
        """State to measurement"""
        return np.concatenate((x[:9],[x[0]-x[3],x[1]-x[4],x[3]-x[6],x[4]-x[7]],self.invH.dot(x[:3]),self.invH.dot(x[6:9])))


#    def update_Q(self):
#        y = self.kf.y
#        S = np.dot(self.h,self.kf.P).dot(self.ht) + self.kf.R
#        self.eps = np.dot(y.T, inv(S)).dot(y)
#        print(self.eps)
#        if self.eps > 4 and self.Q_factor_power <2:
#            self.Q_factor_power += 1
#            self.kf.Q *= self.Q_factor
#            #print(self.kf.Q,self.Q_factor)
#        elif self.eps <= 4 and self.Q_factor_power > 0:
#            self.Q_factor_power -= 1
#            self.kf.Q /= self.Q_factor

    def update(self):
        try:
            self.kf.update(self.vehicle.z)
        except:
            pass
        self.vehicle.front_estimated_status = Status(*self.kf.x[:3])
        self.vehicle.estimated_status = Status(*self.kf.x[3:6])
        self.vehicle.following_estimated_status = Status(*self.kf.x[6:9])

    def predict(self):
        try:
            self.kf.predict()
        except Exception as e:
            #print(e)
            #print('wa_o')
            self.kf.P = eye(9)*1e-16
            self.kf.predict()

        #self.update_Q()


        
class LastVehicleUKF(FilterBase):
    """docstring for NoFilter"""
        
    def __init__(self, vehicle):
        super(LastVehicleUKF, self).__init__(vehicle)
        self.H = self.vehicle.H
        self.invH = self.vehicle.invH
        self.delta_t = self.vehicle.delta_t
        noises2 = self.vehicle.observation_noises.sigma**2
        for i in range(len(noises2)):
            if noises2[i] == 0.:
                noises2[i] = 1e-16
        points = MerweScaledSigmaPoints(n=6, alpha=.1, beta=2., kappa=0.1)
        self.kf = UnscentedKalmanFilter(6, 11, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        self.Q_factor = 10
        self.Q_factor_power = 1
        self.kf.Q = Q_discrete_white_noise(3, dt=0.1, var=.1, block_size=2)
        self.kf.R = np.diag(noises2)
        self.kf.x = np.concatenate((self.vehicle.ground_truth,self.vehicle.ground_truth))
        self.kf.x[0] += self.vehicle.safty_dist
        self.kf.P *= .1
        self.h = np.concatenate((eye(6),eye(6)))[:11]
        self.ht = np.concatenate((eye(6),np.concatenate((eye(5),np.zeros([1,5])))),1)

    def fx(self, x, dt):
        """State transition"""
        x[5] = self.vehicle.controller.next
        return np.concatenate((self.H.dot(x[:3]),self.H.dot(x[3:])))


    def hx(self, x):
        """State to measurement"""
        return np.concatenate((x[:6],[x[0]-x[3],x[1]-x[4]],self.invH.dot(x[:3])))


#    def update_Q(self):
#        y = self.kf.y
#        S = np.dot(self.h,self.kf.P).dot(self.ht) + self.kf.R
#        self.eps = np.dot(y.T, inv(S)).dot(y)
#        print(self.eps)
#        if self.eps > 4 and self.Q_factor_power <2:
#            self.Q_factor_power += 1
#            self.kf.Q *= self.Q_factor
#            #print(self.kf.Q,self.Q_factor)
#        elif self.eps <= 4 and self.Q_factor_power > 0:
#            self.Q_factor_power -= 1
#            self.kf.Q /= self.Q_factor

    def update(self):
        try:
            self.kf.update(self.vehicle.z)
        except:
            pass
        self.vehicle.front_estimated_status = Status(*self.kf.x[:3])
        self.vehicle.estimated_status = Status(*self.kf.x[3:])

    def predict(self):
        try:
            self.kf.predict()
        except Exception as e:
            #print(e)
            #print('wa_la')
            self.kf.P = eye(6)*1e-16
            self.kf.predict()

            #self.update_Q()
        #self.kf.P /= 2
        #self.kf.P += np.transpose(self.kf.P)/2


#class FilterBase():
#    """ Filter for one set of measurement
#    This is the base class, no filtering is actually applied.
#    This can be used to check the performance of the filter.
#    """
#  
#    def __init__(self, vehicle):
#        self.vehicle = vehicle
#        self.delta_t = vehicle.delta_t  # 0.01 second per step
#        self.delta_t2 = vehicle.delta_t2
#        self.noises = self.vehicle.observation_observation_noises
#        points = MerweScaledSigmaPoints(n=3, alpha=.1, beta=2., kappa=-1.)
#        self.kf = UnscentedKalmanFilter(3, 3, self.delta_t, fx=self.fx, hx=self.hx, points=points)
#        self.Q_factor = 10
#        self.Q_factor_power = 1
#        self.kf.Q = Q_discrete_white_noise(3, dt=self.delta_t, var=1e-3)
#        self.kf.R = np.diag([self.range_std**2,self.range_std**2,self.range_std**2])
#        self.kf.x = array([float(self.system_measurement.d),
#                           float(self.system_measurement.v),
#                           float(self.system_measurement.a)])
#        self.kf.P *= 10  # Covariance matrix
#
#    def fx(self, x, dt):
#        """State transition"""
#        F = eye(3) + array([[0, 1, 0.5 * dt],
#                            [0, 0, 1],
#                            [0, 0, 0]]) * dt
#        return np.dot(F, x)
#
#
#    def hx(self, x):
#        """State to measurement"""
#        return x
#
#    def update_Q(self):
#        y = self.kf.y
#        S = self.kf.P + self.kf.R
#        self.eps = np.dot(y.T, inv(S)).dot(y)
#        if self.eps > 4 and self.Q_factor_power <2:
#            self.Q_factor_power += 1
#            self.kf.Q *= self.Q_factor
#            #print(self.kf.Q,self.Q_factor)
#        elif self.eps <= 4 and self.Q_factor_power > 0:
#            self.Q_factor_power -= 1
#            self.kf.Q /= self.Q_factor
#
#    def update(self):
#        self.system_status.d = self.system_measurement.d
#        self.system_status.v = self.system_measurement.v
#        self.system_status.a = self.system_measurement.a
#
#
#
#
#
#
#
#
##self.front_car_status
##self.system_status
##self.following_car_
##
##
##
#
#
#
#
#
#
#
#class Filter(FilterBase):
#    """ Filter for one set of measurement
#    Unscented Kalman Filter:
#    https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
#    Adjustable Process Noise for maneuvering detection:
#    https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/14-Adaptive-Filtering.ipynb
#    """
#
#    def update(self):
#        z = array([float(self.system_measurement.d),
#                   float(self.system_measurement.v),
#                   float(self.system_measurement.a)])
#        self.update_Q()
#        try:
#            self.kf.predict()
#        except:
#            self.kf.P = eye(3)*1e-10
#            self.kf.predict()
#        self.kf.update(z)
#        self.system_status.d = float(self.kf.x[0])
#        self.system_status.v = float(self.kf.x[1])
#        self.system_status.a = float(self.kf.x[2])
#