#!/usr/bin/env python3
import numpy as np
from copy import deepcopy
from collections import defaultdict
from controller import LeadVehicleController, FollowingVehicleController, LastVehicleController
from fuel import fuel_consumption
from numpy.linalg import inv
from mpg import mpg
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import pickle
from plot_black_box import show_plt_black_box, save_plt_black_box
import random

def acceleration_bound(acceleration):
    return max(-1, min(1, acceleration))

class VehicleBase(object):
    def __init__(self, initial_status, observation_noises, actuator_noise, delta_t, safty_dist):
        self.ground_truth = deepcopy(initial_status)
        self.observation_noises = observation_noises
        self.actuator_noise = actuator_noise
        self.delta_t = delta_t
        self.safty_dist = safty_dist
        self.delta_t2 = self.delta_t*self.delta_t
        self.H = np.array([[1.,self.delta_t,0.5*self.delta_t2],[0.,1.,self.delta_t],[0.,0.,1.]])  # System function
        self.invH=inv(self.H)
        self.estimated_status = self.invH.dot(initial_status)
        self.front_car = None
        self.following_car = None
        self.controller_acceleration_reference = 0.
        self.black_box = defaultdict(lambda: [])
        self.crashed = False
        self.fuel = 0.
        self.travel_distance = 0.
        seed = random.randrange(4294967295)
        np.random.seed(seed=seed)

    def update(self):
        self.estimated_status[2] = self.controller.next
        self.ground_truth[2] = acceleration_bound(self.controller.next + self.actuator_noise.mean + self.actuator_noise.sigma * np.random.normal())
        self.fuel = fuel_consumption(self.ground_truth[2], self.ground_truth[1], max(0, self.front_car.ground_truth[0] - self.ground_truth[0]), self.delta_t)
        new_ground_truth=self.H.dot(self.ground_truth)
        self.travel_distance = new_ground_truth[0]-self.ground_truth[0]
        self.ground_truth = new_ground_truth

    def rear_ended(self):
        self.black_box['rear_ended'][-1] = 1


    def crash_reset(self):
        self.crashed = False
        if self.front_car.ground_truth[0] < self.ground_truth[0]:
            self.ground_truth[0] = self.front_car.ground_truth[0] - 0.1
            self.ground_truth[1] = min(self.front_car.ground_truth[1], self.ground_truth[1])
            self.ground_truth[2] = min(self.front_car.ground_truth[2], self.ground_truth[2]) 
            self.crashed = True
            self.front_car.rear_ended()


    def show_plt(self):
        show_plt_black_box(pd.DataFrame(self.black_box))

    def save_plt(self,fname):
        save_plt_black_box(fname, pd.DataFrame(self.black_box))


    def get_result(self):
        """Returns MPG and number of crashes.
        """
        total_fuel_usage = sum(self.black_box['fuel']) # Liter
        total_travel_distance = sum(self.black_box['travel_distance']) # Meter
        total_crashes = sum(self.black_box['crashes'])+sum(self.black_box['rear_ended'])
        return mpg(total_travel_distance,total_fuel_usage),total_crashes




class LeadVehicle(VehicleBase):
    def __init__(self, initial_status, acceleration_pattern, delta_t):
        super(LeadVehicle, self).__init__(initial_status, observation_noises=None, actuator_noise=None, delta_t=delta_t, safty_dist=1)
        self.filter = None
        self.controller = LeadVehicleController(self, acceleration_pattern)

    def update(self):
        self.estimated_status[2] = self.controller.next
        self.ground_truth[2] = self.controller.next
        self.ground_truth=self.H.dot(self.ground_truth)

    def crash_reset(self):
        pass

    def rear_ended(self):
        pass

    def get_reference_acceleration(self):
        pass

    def observe(self):
        pass

    def record(self):
        pass


class FollowingVehicle(VehicleBase):
    def __init__(self, initial_status, observation_noises, actuator_noise, delta_t, safty_dist):
        super(FollowingVehicle, self).__init__(initial_status, observation_noises, actuator_noise, delta_t, safty_dist)
        self.filter = None
        self.controller = FollowingVehicleController(self)

    def observe(self):
        self.z=np.array([
        self.front_car.ground_truth[0],
        self.front_car.ground_truth[1],
        self.front_car.ground_truth[2],
        self.ground_truth[0],
        self.ground_truth[1],
        self.ground_truth[2],
        self.following_car.ground_truth[0],
        self.following_car.ground_truth[1],
        self.following_car.ground_truth[2],
        self.front_car.ground_truth[0]-self.ground_truth[0],
        self.front_car.ground_truth[1]-self.ground_truth[1],
        self.front_car.ground_truth[2]-self.ground_truth[2],
        self.ground_truth[0]-self.following_car.ground_truth[0],
        self.ground_truth[1]-self.following_car.ground_truth[1],
        self.ground_truth[2]-self.following_car.ground_truth[2],
        self.front_car.estimated_status[0],
        self.front_car.estimated_status[1],
        self.front_car.estimated_status[2],
        self.following_car.estimated_status[0],
        self.following_car.estimated_status[1],
        self.following_car.estimated_status[2]]) + self.observation_noises.mean + self.observation_noises.sigma * np.random.normal(size=len(self.observation_noises.sigma))


    def record(self):
        self.black_box['front_real_d'].append(self.front_car.ground_truth[0])
        self.black_box['front_real_v'].append(self.front_car.ground_truth[1])
        self.black_box['front_real_a'].append(self.front_car.ground_truth[2])
        self.black_box['front_measured_d'].append(self.z[0])
        self.black_box['front_measured_v'].append(self.z[1])
        self.black_box['front_measured_a'].append(self.z[2])
        self.black_box['front_estimated_d'].append(self.front_estimated_status[0])
        self.black_box['front_estimated_v'].append(self.front_estimated_status[1])
        self.black_box['front_estimated_a'].append(self.front_estimated_status[2])
        self.black_box['real_d'].append(self.ground_truth[0])
        self.black_box['real_v'].append(self.ground_truth[1])
        self.black_box['real_a'].append(self.ground_truth[2])
        self.black_box['measured_d'].append(self.z[3])
        self.black_box['measured_v'].append(self.z[4])
        self.black_box['measured_a'].append(self.z[5])
        self.black_box['estimated_d'].append(self.estimated_status[0])
        self.black_box['estimated_v'].append(self.estimated_status[1])
        self.black_box['estimated_a'].append(self.estimated_status[2])
        self.black_box['following_real_d'].append(self.following_car.ground_truth[0])
        self.black_box['following_real_v'].append(self.following_car.ground_truth[1])
        self.black_box['following_real_a'].append(self.following_car.ground_truth[2])
        self.black_box['following_measured_d'].append(self.z[6])
        self.black_box['following_measured_v'].append(self.z[7])
        self.black_box['following_measured_a'].append(self.z[8])
        self.black_box['following_estimated_d'].append(self.following_estimated_status[0])
        self.black_box['following_estimated_v'].append(self.following_estimated_status[1])
        self.black_box['following_estimated_a'].append(self.following_estimated_status[2])
        self.black_box['real_front_distance'].append(self.front_car.ground_truth[0]-self.ground_truth[0])
        self.black_box['measured_front_distance'].append(self.z[9])
        self.black_box['estimated_front_distance'].append(self.front_estimated_diff[0])
        self.black_box['real_front_v_difference'].append(self.front_car.ground_truth[1]-self.ground_truth[1])
        self.black_box['measured_front_v_difference'].append(self.z[10])
        self.black_box['estimated_front_v_difference'].append(self.front_estimated_diff[1])
        self.black_box['real_following_distance'].append(self.ground_truth[0]-self.following_car.ground_truth[0])
        self.black_box['measured_following_distance'].append(self.z[12])
        self.black_box['estimated_following_distance'].append(self.following_estimated_diff[0])
        self.black_box['real_following_v_difference'].append(self.ground_truth[1]-self.following_car.ground_truth[1])
        self.black_box['measured_following_v_difference'].append(self.z[13])
        self.black_box['estimated_following_v_difference'].append(self.following_estimated_diff[1])
        self.black_box['reference_a'].append(self.controller_acceleration_reference)
        self.black_box['crashes'].append(1 if self.crashed else 0)
        self.black_box['rear_ended'].append(0)
        self.black_box['fuel'].append(self.fuel)
        self.black_box['travel_distance'].append(self.travel_distance)
        self.crashed = False



class LastVehicle(VehicleBase):
    def __init__(self, initial_status, observation_noises, actuator_noise, delta_t, safty_dist):
        super(LastVehicle, self).__init__(initial_status, observation_noises, actuator_noise, delta_t, safty_dist)
        self.filter = None
        self.controller = LastVehicleController(self)

    def observe(self):
        self.z=np.array([
        self.front_car.ground_truth[0],
        self.front_car.ground_truth[1],
        self.front_car.ground_truth[2],
        self.ground_truth[0],
        self.ground_truth[1],
        self.ground_truth[2],
        self.front_car.ground_truth[0]-self.ground_truth[0],
        self.front_car.ground_truth[1]-self.ground_truth[1],
        self.front_car.ground_truth[2]-self.ground_truth[2],
        self.front_car.estimated_status[0],
        self.front_car.estimated_status[1],
        self.front_car.estimated_status[2]]) + self.observation_noises.mean + self.observation_noises.sigma * np.random.normal(size=len(self.observation_noises.sigma))

    def record(self):
        self.black_box['front_real_d'].append(self.front_car.ground_truth[0])
        self.black_box['front_real_v'].append(self.front_car.ground_truth[1])
        self.black_box['front_real_a'].append(self.front_car.ground_truth[2])
        self.black_box['front_measured_d'].append(self.z[0])
        self.black_box['front_measured_v'].append(self.z[1])
        self.black_box['front_measured_a'].append(self.z[2])
        self.black_box['front_estimated_d'].append(self.front_estimated_status[0])
        self.black_box['front_estimated_v'].append(self.front_estimated_status[1])
        self.black_box['front_estimated_a'].append(self.front_estimated_status[2])
        self.black_box['real_d'].append(self.ground_truth[0])
        self.black_box['real_v'].append(self.ground_truth[1])
        self.black_box['real_a'].append(self.ground_truth[2])
        self.black_box['measured_d'].append(self.z[3])
        self.black_box['measured_v'].append(self.z[4])
        self.black_box['measured_a'].append(self.z[5])
        self.black_box['estimated_d'].append(self.estimated_status[0])
        self.black_box['estimated_v'].append(self.estimated_status[1])
        self.black_box['estimated_a'].append(self.estimated_status[2])
        self.black_box['real_front_distance'].append(self.front_car.ground_truth[0]-self.ground_truth[0])
        self.black_box['estimated_front_distance'].append(self.front_estimated_diff[0])
        self.black_box['measured_front_distance'].append(self.z[6])
        self.black_box['real_front_v_difference'].append(self.front_car.ground_truth[1]-self.ground_truth[1])
        self.black_box['measured_front_v_difference'].append(self.z[7])
        self.black_box['estimated_front_v_difference'].append(self.front_estimated_diff[1])
        self.black_box['reference_a'].append(self.controller_acceleration_reference)
        self.black_box['crashes'].append(1 if self.crashed else 0)
        self.black_box['fuel'].append(self.fuel)
        self.black_box['travel_distance'].append(self.travel_distance)
        self.crashed = False



    def get_result(self):
        """Returns MPG and number of crashes.
        """
        total_fuel_usage = sum(self.black_box['fuel']) # Liter
        total_travel_distance = sum(self.black_box['travel_distance']) # Meter
        total_crashes = sum(self.black_box['crashes'])
        return mpg(total_travel_distance,total_fuel_usage),total_crashes

