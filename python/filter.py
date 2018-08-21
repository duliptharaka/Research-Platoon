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
        self.H = self.vehicle.H
        self.invH = self.vehicle.invH
        self.delta_t = self.vehicle.delta_t

    def update(self):
        self.vehicle.estimated_status = deepcopy(self.vehicle.ground_truth)

    def predict(self):
        pass


class FollowingVehicleFilterBase(FilterBase):
    """docstring for NoFilter"""


    def dummy_front_status_list(self, z, H_dot_self, H_dot_front, H_dot_following):
        return np.array([
                z[:3],
                H_dot_front,
                z[3:6]+z[9:12],
                H_dot_self+z[9:12],
                z[6:9]+z[9:12]+z[12:15],
                H_dot_following+z[9:12]+z[12:15]
                ])

    def dummy_status_list(self, z, H_dot_self, H_dot_front, H_dot_following):
        return np.array([
                z[3:6],
                H_dot_self,
                z[:3]-z[9:12],
                z[6:9]+z[12:15],
                H_dot_front-z[9:12],
                H_dot_following+z[12:15]
                ])

    def dummy_following_status_list(self, z, H_dot_self, H_dot_front, H_dot_following):
        return np.array([
                z[6:9],
                H_dot_following,
                z[3:6]-z[12:15],
                H_dot_self-z[12:15],
                z[:3]-z[9:12]-z[12:15],
                H_dot_front-z[9:12]-z[12:15]
                ])

    def dummy_front_diff_list(self, z, H_dot_self, H_dot_front, H_dot_following):
        return np.array([
                z[9:12],
                z[:3]-z[3:6],
                z[:3]-H_dot_self,
                z[:3]-z[6:9]-z[12:15],
                z[:3]-H_dot_following-z[12:15],
                H_dot_front-z[3:6],
                H_dot_front-H_dot_self,
                H_dot_front-z[6:9]-z[12:15],
                H_dot_front-H_dot_following-z[12:15],
                ])

    def dummy_following_diff_list(self, z, H_dot_self, H_dot_front, H_dot_following):
        return np.array([
                z[12:15],
                z[3:6]-z[6:9],
                z[3:6]-H_dot_following,
                H_dot_self-z[6:9],
                H_dot_self-H_dot_following,
                z[:3]-z[9:12]-z[6:9],
                z[:3]-z[9:12]-H_dot_following,
                H_dot_front-z[9:12]-z[6:9],
                H_dot_front-z[9:12]-H_dot_following
                ])

    def update(self):
        z = self.vehicle.z
        self.vehicle.front_estimated_status = Status(*z[:3])
        self.vehicle.estimated_status = Status(*z[3:6])
        self.vehicle.following_estimated_status = Status(*z[6:9])
        self.vehicle.front_estimated_diff = Status(*z[9:12])
        self.vehicle.following_estimated_diff = Status(*z[12:15])


class LastVehicleFilterBase(FilterBase):
    """docstring for NoFilter"""


    def dummy_front_status_list(self, z, H_dot_self, H_dot_front):
        return np.array([
                z[:3],
                H_dot_front,
                z[3:6]+z[6:9],
                H_dot_self+z[6:9]
                ])

    def dummy_status_list(self, z, H_dot_self, H_dot_front):
        return np.array([
                z[3:6],
                H_dot_self,
                z[:3]-z[6:9],
                H_dot_front-z[6:9]
                ])

    def dummy_front_diff_list(self, z, H_dot_self, H_dot_front):
        return np.array([
                z[6:9],
                z[:3]-z[3:6],
                H_dot_front-z[3:6],
                z[:3]-H_dot_self,
                H_dot_front-H_dot_self
                ])
        
    def update(self):
        z = self.vehicle.z
        self.vehicle.front_estimated_status = Status(*z[:3])
        self.vehicle.estimated_status = Status(*z[3:6])
        self.vehicle.front_estimated_diff = Status(*z[6:9])



class FollowingVehicleFilterAverage(FollowingVehicleFilterBase):
    """docstring for NoFilter"""

    def mean_(self, list_):
        return np.mean(list_, axis=0)

    def update(self):
        z = self.vehicle.z
        H_dot_front = self.H.dot(z[15:18])
        H_dot_following = self.H.dot(z[18:21])
        H_dot_self = self.H.dot(self.vehicle.estimated_status)
        self.vehicle.front_estimated_status = Status(*self.mean_(self.dummy_front_status_list(z, H_dot_self, H_dot_front, H_dot_following)))
        self.vehicle.estimated_status = Status(*self.mean_(self.dummy_status_list(z, H_dot_self, H_dot_front, H_dot_following)))
        self.vehicle.following_estimated_status = Status(*self.mean_(self.dummy_following_status_list(z, H_dot_self, H_dot_front, H_dot_following)))
        self.vehicle.front_estimated_diff = Status(*self.mean_(self.dummy_front_diff_list(z, H_dot_self, H_dot_front, H_dot_following)))
        self.vehicle.following_estimated_diff = Status(*self.mean_(self.dummy_following_diff_list(z, H_dot_self, H_dot_front, H_dot_following)))


class LastVehicleFilterAverage(LastVehicleFilterBase):
    """docstring for NoFilter"""

    def mean_(self, list_):
        return np.mean(list_, axis=0)

    def update(self):
        z = self.vehicle.z
        H_dot_front = self.H.dot(z[9:12])
        H_dot_self = self.H.dot(self.vehicle.estimated_status)
        self.vehicle.front_estimated_status = Status(*self.mean_(self.dummy_front_status_list(z, H_dot_self, H_dot_front)))
        self.vehicle.estimated_status = Status(*self.mean_(self.dummy_status_list(z, H_dot_self, H_dot_front)))
        self.vehicle.front_estimated_diff = Status(*self.mean_(self.dummy_front_diff_list(z, H_dot_self, H_dot_front)))


class FollowingVehicleFilterNoHLAverage(FollowingVehicleFilterAverage):
    """docstring for NoFilter"""

    def mean_(self,list_):
        return np.mean(np.sort(list_, axis=0)[1:-1,:], axis=0)


class LastVehicleFilterNoHLAverage(LastVehicleFilterAverage):
    """docstring for NoFilter"""

    def mean_(self,list_):
        return np.mean(np.sort(list_, axis=0)[1:-1,:], axis=0)
        

class FollowingVehicleUKF(FollowingVehicleFilterBase):
    """https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb"""
        
    def __init__(self, vehicle):
        super(FollowingVehicleUKF, self).__init__(vehicle)
        #noises2 = self.vehicle.observation_noises.sigma**2
        #for i in range(len(noises2)):
        #    if noises2[i] == 0.:
        #        noises2[i] = 1e-16
        points = MerweScaledSigmaPoints(n=9, alpha=.1, beta=2., kappa=0.1)
        self.kf = UnscentedKalmanFilter(9, 108, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        #self.Q_factor = 10
        #self.Q_factor_power = 1
        self.kf.Q = Q_discrete_white_noise(3, dt=0.1, var=1e-2, block_size=3)
        self.kf.R *= max(np.median(self.vehicle.observation_noises.sigma**2),1e-16)
        self.kf.x = np.concatenate((self.vehicle.ground_truth,self.vehicle.ground_truth,self.vehicle.ground_truth))
        self.kf.x[0] += self.vehicle.safty_dist
        self.kf.x[6] -= self.vehicle.safty_dist
        self.kf.P *= .1
        #self.h = np.concatenate((eye(6),eye(6)))[:11]
        #self.ht = np.concatenate((eye(6),np.concatenate((eye(5),np.zeros([1,5])))),1)

    def fx(self, x, dt):
        """State transition"""

        return np.concatenate((self.H.dot(x[:3]),self.H.dot(x[3:6]),self.H.dot(x[6:9])))


    def hx(self, x):
        """State to measurement"""
        front_diff = x[:3]-x[3:6]
        following_diff = x[3:6]-x[6:9]
        return np.concatenate((
            x[:3],
            x[:3],
            x[:3],
            x[:3],
            x[:3],
            x[:3],
            x[3:6],
            x[3:6],
            x[3:6],
            x[3:6],
            x[3:6],
            x[3:6],
            x[6:9],
            x[6:9],
            x[6:9],
            x[6:9],
            x[6:9],
            x[6:9],
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff,
            following_diff
            ))

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
        _z = self.vehicle.z
        H_dot_front = self.H.dot(_z[15:18])
        H_dot_following = self.H.dot(_z[18:21])
        H_dot_self = self.H.dot(self.vehicle.estimated_status)
        z = np.concatenate((
        *self.dummy_front_status_list(_z, H_dot_self, H_dot_front, H_dot_following),
        *self.dummy_status_list(_z, H_dot_self, H_dot_front, H_dot_following),
        *self.dummy_following_status_list(_z, H_dot_self, H_dot_front, H_dot_following),
        *self.dummy_front_diff_list(_z, H_dot_self, H_dot_front, H_dot_following),
        *self.dummy_following_diff_list(_z, H_dot_self, H_dot_front, H_dot_following)
        ))

        try:
            self.kf.update(z)
        except Exception as e:
            pass
            #pass
        self.vehicle.front_estimated_status = Status(*self.kf.x[:3])
        self.vehicle.estimated_status = Status(*self.kf.x[3:6])
        self.vehicle.following_estimated_status = Status(*self.kf.x[6:9])
        self.vehicle.front_estimated_diff = self.vehicle.front_estimated_status-self.vehicle.estimated_status
        self.vehicle.following_estimated_diff = self.vehicle.estimated_status-self.vehicle.following_estimated_status

    def predict(self):
        try:
            self.kf.predict()
        except Exception as e:
            #print(e)
            #print('wa_o')
            self.kf.P = eye(9)*1e-16
            self.kf.predict()

        #self.update_Q()


        
class LastVehicleUKF(LastVehicleFilterAverage):
    """https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb"""
        
    def __init__(self, vehicle):
        super(LastVehicleUKF, self).__init__(vehicle)
        #noises2 = self.vehicle.observation_noises.sigma**2
        #for i in range(len(noises2)):
        #    if noises2[i] == 0.:
        #        noises2[i] = 1e-16
        points = MerweScaledSigmaPoints(n=6, alpha=.1, beta=2., kappa=0.1)
        self.kf = UnscentedKalmanFilter(6, 39, self.delta_t, fx=self.fx, hx=self.hx, points=points)
        #self.Q_factor = 10
        #self.Q_factor_power = 1
        self.kf.Q = Q_discrete_white_noise(3, dt=0.1, var=1e-2, block_size=2)
        #print(self.kf.R,'RRR')
        self.kf.R *= max(np.median(self.vehicle.observation_noises.sigma**2),1e-16)
        self.kf.x = np.concatenate((self.vehicle.ground_truth,self.vehicle.ground_truth))
        self.kf.x[0] += self.vehicle.safty_dist
        self.kf.P *= .1
        #self.h = np.concatenate((eye(6),eye(6)))[:11]
        #self.ht = np.concatenate((eye(6),np.concatenate((eye(5),np.zeros([1,5])))),1)

    def fx(self, x, dt):
        """State transition"""
        return np.concatenate((self.H.dot(x[:3]),self.H.dot(x[3:])))


    def hx(self, x):
        """State to measurement"""
        front_diff = x[:3]-x[3:6]
        return np.concatenate((
            x[:3],
            x[:3],
            x[:3],
            x[:3],
            x[3:6],
            x[3:6],
            x[3:6],
            x[3:6],
            front_diff,
            front_diff,
            front_diff,
            front_diff,
            front_diff
            ))

        #return np.concatenate((x[:6],[x[0]-x[3],x[1]-x[4]],self.invH.dot(x[:3])))


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
        _z = self.vehicle.z
        H_dot_front = self.H.dot(_z[9:12])
        H_dot_self = self.H.dot(self.vehicle.estimated_status)
        z = np.squeeze(np.concatenate((
        *self.dummy_front_status_list(_z, H_dot_self, H_dot_front),
        *self.dummy_status_list(_z, H_dot_self, H_dot_front),
        *self.dummy_front_diff_list(_z, H_dot_self, H_dot_front)
        )))

        try:
            self.kf.update(z)
        except:
            pass
        self.vehicle.front_estimated_status = Status(*self.kf.x[:3])
        self.vehicle.estimated_status = Status(*self.kf.x[3:])
        self.vehicle.front_estimated_diff = self.vehicle.front_estimated_status-self.vehicle.estimated_status

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
