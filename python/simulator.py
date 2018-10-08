#!/usr/bin/env python3
from noises import Noises
from status import Status
from vehicle import LeadVehicle, FollowingVehicle, LastVehicle
from controller import LeadVehicleController
from filter import *
from platoon import CentralizedPlatoon, DecentralizedPlatoon
import random

class Simulator(object):
    """docstring for Simulator"""

    def __init__(self, 
        controller_type='d',  #  'c' or 'd' or 'lead_vehicle_pattern'
        platoon_sensor_nosie_matrix=None,  # number of following vehicles * nosies
        platoon_actuator_noise_matrix=None,   # number of following vehicles * nosies
        filter_type='no_filter',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
        acceleration_pattern_file='first_car_a10',
        velocity_pattern_file='first_car_v',
        delta_t=0.1,
        safty_dist=1):

        seed = random.randrange(4294967295)
        np.random.seed(seed=seed)

        self.controller_type = controller_type
        if platoon_sensor_nosie_matrix is not None:
            self.platoon_sensor_nosie_matrix = platoon_sensor_nosie_matrix
        else:
            self.platoon_sensor_nosie_matrix =  [
                Noises([0.]*21,[0.1]*15+[0.]*6),
                Noises([0.]*21,[0.1]*15+[0.]*6),
                Noises([0.]*21,[0.1]*15+[0.]*6),
                Noises([0.]*12,[0.1]*9+[0.]*3)
            ]   # number of following vehicles * nosies
        if platoon_actuator_noise_matrix is not None:
            self.platoon_actuator_noise_matrix = platoon_actuator_noise_matrix
        else:
            self.platoon_actuator_noise_matrix = [
                Noises([0.],[0.01]),
                Noises([0.],[0.01]),
                Noises([0.],[0.01]),
                Noises([0.],[0.01])
            ]   # number of following vehicles * nosies
        self.filter_type = filter_type
        self.delta_t = delta_t
        self.safty_dist = safty_dist
        self.platoon_length = len(self.platoon_sensor_nosie_matrix)
        if self.platoon_length != len(self.platoon_actuator_noise_matrix):
            print('platoon_sensor_nosie_matrix, and platoon_actuator_noise_matrix does not have the same length')

    
        self.acceleration_pattern = self.get_pattern_from_file(acceleration_pattern_file)#[0.0]*6990#
        self.velocity_pattern = self.get_pattern_from_file(velocity_pattern_file)#[24.5872]*6990#

        self.build_platoon()

    def random_init(self,status_):
        status_+= np.array([max(-1, np.random.normal()*0.1),max(-1,np.random.normal()*0.1),max(-1, np.random.normal()*0.1)])
        return status_


    def run(self,steps):
        for i in range(steps):
            self.platoon.update()

    def get_result(self):
        return self.platoon.get_result()

    def get_pattern_from_file(self, file):
        with open(file, 'r') as f:
            return [float(line) for line in list(f)]


    def build_platoon(self):
    
        vehicles=[None]*(self.platoon_length+1)
    
        init_status = Status((len(vehicles)-1)*self.safty_dist,self.velocity_pattern[0],self.acceleration_pattern[0])
        vehicles[0] = LeadVehicle(init_status, self.acceleration_pattern, self.delta_t)
        vehicles[0].filter = FilterBase(vehicles[0])
        
        for i in range(1,len(vehicles)-1):
            
            init_status[0] -= self.safty_dist
            init_status[1] = self.velocity_pattern[0]
            init_status[2] = self.acceleration_pattern[0]
            init_status = self.random_init(init_status)
            observation_noises = self.platoon_sensor_nosie_matrix[i-1]
            #Noises([unified_observation_noise_mean]*19,[unified_observation_noise_var]*11+[0]*8)
            actuator_noise = self.platoon_actuator_noise_matrix[i-1]
            #Noises([0],[0])
            vehicles[i] = FollowingVehicle(init_status, observation_noises, actuator_noise, self.delta_t, self.safty_dist)
            if self.filter_type == 'ukf':
                vehicles[i].filter = FollowingVehicleUKF(vehicles[i])
            elif self.filter_type == 'no_filter':
                vehicles[i].filter = FollowingVehicleFilterBase(vehicles[i])
            elif self.filter_type == 'average':
                vehicles[i].filter = FollowingVehicleFilterAverage(vehicles[i])
            elif self.filter_type == 'average_without_high_low':
                vehicles[i].filter = FollowingVehicleFilterNoHLAverage(vehicles[i])


    
        init_status[0] -= self.safty_dist
        init_status[1] = self.velocity_pattern[0]
        init_status[2] = self.acceleration_pattern[0]
        init_status = self.random_init(init_status)
        observation_noises = self.platoon_sensor_nosie_matrix[-1]
        #Noises([unified_observation_noise_mean]*11,[unified_observation_noise_var]*7+[0]*4)
        actuator_noise = self.platoon_actuator_noise_matrix[-1]
        #Noises([0],[0])
        vehicles[-1] = LastVehicle(init_status, observation_noises, actuator_noise, self.delta_t, self.safty_dist)
        if self.filter_type == 'ukf':
            vehicles[-1].filter = LastVehicleUKF(vehicles[-1])
        elif self.filter_type == 'no_filter':
            vehicles[-1].filter = LastVehicleFilterBase(vehicles[-1])
        elif self.filter_type == 'average':
            vehicles[-1].filter = LastVehicleFilterAverage(vehicles[-1])
        elif self.filter_type == 'average_without_high_low':
            vehicles[-1].filter = LastVehicleFilterNoHLAverage(vehicles[-1])
    
        #print('platoon?',self.controller_type)
        if self.controller_type == 'c':
            self.platoon = CentralizedPlatoon(vehicles)
        elif self.controller_type == 'd':
            self.platoon = DecentralizedPlatoon(vehicles)
        elif self.controller_type == 'lead_vehicle_pattern':
            self.platoon = CentralizedPlatoon(vehicles)
            for i in range(1,len(vehicles)):
                vehicles[i].controller == LeadVehicleController(vehicles[i], self.acceleration_pattern)




if __name__ == '__main__':
    s=Simulator(safty_dist=1)
    s.run(6990)
    print(s.platoon.get_result())
    #s.platoon.vehicles[1].show_plt()
    #s.platoon.vehicles[2].show_plt()
    #s.platoon.vehicles[3].show_plt()
    #s.platoon.vehicles[1].save_plt('example_v1.pdf')
    #s.platoon.vehicles[2].save_plt('example_v2.pdf')
    #s.platoon.vehicles[3].save_plt('example_v3.pdf')
    s.platoon.vehicles[0].save_black_box('black_box_v0.csv')
    s.platoon.vehicles[1].save_black_box('black_box_v1.csv')
    s.platoon.vehicles[2].save_black_box('black_box_v2.csv')
    s.platoon.vehicles[3].save_black_box('black_box_v3.csv')
    s.platoon.vehicles[4].save_black_box('black_box_v4.csv')
    input("Press Enter to exit...")