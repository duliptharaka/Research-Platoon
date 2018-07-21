#!/usr/bin/env python3
from mpg import mpg
from collections import UserList


class PlatoonBase(UserList):
    def __init__(self, vehicles):
        super(PlatoonBase, self).__init__(vehicles)
        self.vehicles = self.data
        self._add_vehicles()

    def _add_vehicles(self):
        for i in range(len(self.vehicles)-1):
            self.vehicles[i].following_car = self.vehicles[i+1]
        for i in range(1, len(self.vehicles)):
            self.vehicles[i].front_car = self.vehicles[i-1]


    def get_result(self):
        """Returns MPG and average number of crashes per following car.
        """
        total_fuel_usage = 0. # Liter
        total_travel_distance = 0. # Meter
        total_crashes = 0.
        for vehicle in self.vehicles[1:]:
            total_fuel_usage += sum(vehicle.black_box['fuel'])
            total_travel_distance += sum(vehicle.black_box['travel_distance'])
            total_crashes += sum(vehicle.black_box['crashes'])
        return mpg(total_travel_distance,total_fuel_usage),total_crashes/(len(self.vehicles)-1)



class CentralizedPlatoon(PlatoonBase):

    def update(self):

        for vehicle in self.vehicles:
            vehicle.observe()

        for vehicle in self.vehicles:
            vehicle.filter.update()

        self.vehicles[0].controller.update()

        for vehicle in self.vehicles[1:]:
            vehicle.controller_acceleration_reference = self.vehicles[0].controller.next#0.5*(self.vehicles[0].controller.next + vehicle.front_estimated_status[2])
            vehicle.record()
            vehicle.controller.update()

        for vehicle in self.vehicles:
            vehicle.update()


        for vehicle in self.vehicles[1:]:
            vehicle.crash_reset()
            
        for vehicle in self.vehicles:
            vehicle.filter.predict()
        #for vehicle in self.vehicles:
        #    vehicle.record()


class DecentralizedPlatoon(PlatoonBase):

    def update(self):

        for vehicle in self.vehicles:
            vehicle.observe()  # Including neighboring vechicles' status estimation and the controller planned movement from last step, all ground truth for all cars.

        for vehicle in self.vehicles:
            vehicle.filter.update()
        #for vehicle in self.vehicles:
            #vehicle.record()
        for vehicle in self.vehicles[1:]:
            vehicle.controller_acceleration_reference = vehicle.front_estimated_status[2]

        for vehicle in self.vehicles:
            vehicle.record()

        for vehicle in self.vehicles:
            vehicle.controller.update()

        for vehicle in self.vehicles:
            vehicle.update()

        for vehicle in self.vehicles[1:]:
            vehicle.crash_reset()

        for vehicle in self.vehicles:
            vehicle.filter.predict()