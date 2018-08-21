#!/usr/bin/env python3
import numpy as np
from copy import deepcopy
from acceleration_bound import acceleration_bound


class Optimizer(np.ndarray):
    def __new__(cls,delta_t):
        """
        This function is created based on Dr. Pisu's idea of using objective function to get a balance among the control of the car and the extra gas consumption caused by rapid acceleration. That idea sounds good but does not make sense since the optimal way to save gas under this scenario is to keep the car at the desired safety spot, where the air drag is also close to minimum. When consider the gas consumption caused by rapid acceleration, the car will keep slow down, until a rapid acceleration and slowdown again and cause oscillations.
        This function does not handle many problems that a PID controller could handle.
        That means, this function should only performs worse than a simple PID controller with anti windup.
    
        Objective function: 
        
        D: distance to the safety spot if current car keep current speed
        V: difference of velocity
        A: target acceleration
        
        min(w1(D-0.5a*dt^2)^2+w2(V-a*dt)^2+w3(A-a)^2)
        (w1(D-0.5a*dt^2)^2+w2(V-a*dt)^2+w3(A-a)^2)d(a) = 0
        (w1*D^2-w1*D*dt^2*a+0.25*w1*dt^4*a^2
        +w2*V^2-2*w2*V*dt*a+w2*dt^2*a^2
        +w3*A^2-2*w3*A*a+w3*a^2)d(a)=0
        -w1*D*dt^2+0.5*w1*dt^4*a
        -2*w2*V*dt+2*w2*dt^2*a
        -2*w3*A+2*w3*a = 0
        a = (2*w1*D*dt^2+4*w2*V*dt+4*w3*A)/(w1*dt^4+4*w2*dt^2+4*w3)
        (2*w1*delta_d*delta_t**2+4*w2*delta_v*delta_t+4*w3*target_a)/(w1*delta_t**4+4*w2*delta_t**2+4*w3)

        usage: opt = Optimizer(0.1)
                 a = np.dot(H,(front_car_status-current_car_status))*opt
        """
        w1 = 0.9# D
        w2 = 0.09# V
        w3 = 0.01# A

        ret = np.array([(2*w1*delta_t**2)/(w1*delta_t**4+4*w2*delta_t**2+4*w3),
                             (4*w2*delta_t)/(w1*delta_t**4+4*w2*delta_t**2+4*w3),
                             (4*w3)/(w1*delta_t**4+4*w2*delta_t**2+4*w3)])

        return ret.view(cls)

class LeadVehicleController():
    def __init__(self, vehicle, acceleration_pattern):
        self.vehicle = vehicle
        self.acceleration_pattern = acceleration_pattern
        self.next = self.vehicle.ground_truth[2]
        self._g = self._generator()

    def _generator(self):
        for acceleration in self.acceleration_pattern:
            yield acceleration

    def update(self):
        self.next = next(self._g)


class FollowingVehicleController():
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.H = self.vehicle.H
        self.safty_dist = self.vehicle.safty_dist
        self.optimizer = Optimizer(vehicle.delta_t)
        self.next = self.vehicle.ground_truth[2]

    def update(self):
        #if (self.vehicle.following_estimated_diff[0] < (2*self.vehicle.safty_dist)) and self.vehicle.front_estimated_diff[0] >= self.vehicle.safty_dist:
        #front_estimate = deepcopy(self.vehicle.front_estimated_status)
        #front_estimate[2] = self.vehicle.controller_acceleration_reference 
        #self_estimate = deepcopy(self.vehicle.estimated_status)
        #self_estimate[2] = 0
        #following_estimate = deepcopy(self.vehicle.following_estimated_status)
        #following_estimate[2] = self.vehicle.controller_acceleration_reference 
        #delta = self.H.dot((front_estimate+following_estimate)/2-self_estimate)
        #self.next = acceleration_bound(self.optimizer.dot(delta))

        #else:
        front_estimate_diff = deepcopy(self.vehicle.front_estimated_diff)
        front_estimate_diff[2] = self.vehicle.controller_acceleration_reference 
        delta_front = self.H.dot(front_estimate_diff)
        delta_front[0] -= self.safty_dist


        following_estimate_diff = deepcopy(self.vehicle.following_estimated_diff)
        following_estimate_diff[2] = self.vehicle.controller_acceleration_reference 
        delta_following = self.H.dot(following_estimate_diff)
        delta_following[0] -= self.safty_dist

        if 0 <= delta_front[0] and delta_following[0] < self.safty_dist:
            delta = (delta_front+delta_following)/2
            delta[2] = self.vehicle.controller_acceleration_reference
            self.next = acceleration_bound(self.optimizer.dot(delta))
        else:
            self.next = acceleration_bound(self.optimizer.dot(delta_front))

class LastVehicleController():
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.H = self.vehicle.H
        self.safty_dist = self.vehicle.safty_dist
        self.optimizer = Optimizer(vehicle.delta_t)
        #print(self.optimizer)
        self.next = self.vehicle.ground_truth[2]

    def update(self):
        front_estimate_diff = deepcopy(self.vehicle.front_estimated_diff)
        front_estimate_diff[2] = self.vehicle.controller_acceleration_reference 
        delta = self.H.dot(front_estimate_diff)
        delta[0] -= self.safty_dist
        self.next = acceleration_bound(self.optimizer.dot(delta))
