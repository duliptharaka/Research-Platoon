#!/usr/bin/env python3

from car import Status, LeadCar, DecentralizedFollowingCar, CentralizedFollowingCar
from copy import copy, deepcopy

def p_status(*statuses):
    """simple helper function to print difference between several statuses"""
    gt=statuses[0]
    for i in range(1,len(statuses)):
        print("{:10.4f},\t{:.4f},\t{:.4f}".format(statuses[i].d-gt.d, statuses[i].v-gt.v, statuses[i].a-gt.a))

class Results(object):
    """docstring for Results"""
    def __init__(self, arg):
        self.arg = arg
    def plot_to_pdf(self):
        pass
        

class Simulator(object):
    """Simulate Platoon"""
    def __init__(self,
        number_CentralizedFollowingCar=2,
        number_DecentralizedFollowingCar=2,
        sensor_noise_center=0,
        sensor_noise_sigma=0.001,
        actuator_noise_center=0,
        actuator_noise_sigma=0.0,
        safty_dist=5,
        a_file='first_car_a2',
        delta_t = 0.1
        ):
        init_status = Status(safty_dist*(number_CentralizedFollowingCar+number_DecentralizedFollowingCar), 13.4112, 0)
        self.lc = LeadCar(deepcopy(init_status), delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma, a_file)
        self.platoon = []
        pc = self.lc
        for i in range(number_CentralizedFollowingCar):
            init_status.d -= safty_dist
            self.platoon.append(CentralizedFollowingCar(deepcopy(init_status), delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma,  pc, safty_dist, self.lc))
            pc = self.platoon[-1]
        for i in range(number_DecentralizedFollowingCar):
            init_status.d -= safty_dist
            self.platoon.append(DecentralizedFollowingCar(deepcopy(init_status), delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma,  pc, safty_dist))
            pc = self.platoon[-1]
    def run(self, steps=10):
        for i in range(steps):
            self.lc.update()
            for c in self.platoon:
                c.update()
    def get_result(self):
        total_fuel_usage = 0 # Liter
        total_travel_distance = 0 # Meter
        total_crashes = 0
        for c in self.platoon:
            total_fuel_usage += c.black_box.fuel_consumption
            total_travel_distance += c.black_box.travel_distance
            total_crashes += c.black_box.crash_counter
            print(c.black_box.travel_distance, c.black_box.fuel_consumption)
        return (total_travel_distance*0.0006213712)/(total_fuel_usage*0.2641720524),total_crashes


class TestBed(object):
    """Performe a set of simulations"""
    def __init__(self):
        pass
    def run(self):
        for i in range(10):
            sensor_noise_sigma = i*0.05
            s = Simulator(sensor_noise_sigma=sensor_noise_sigma)
            s.run(6990)
            #s.lc.black_box.plot_record()
            #s.platoon[2].black_box.plot_record()
            print(s.get_result())


        

if __name__ == '__main__':
    t=TestBed()
    t.run()
    #s = Simulator(sensor_noise_sigma=0.1)
    #s.run(6990)
    #s.lc.black_box.plot_record()
    #s.platoon[2].black_box.plot_record()