import numpy as np
from copy import copy, deepcopy
from controller import Controller, LeadCarController, DecentralizedController, CentralizedController
from filter import Filter, FrontCarFilter,FilterBase
from fuel import fuel_consumption
from blackbox import BlackBox

class Status():
    """Store the status of a Car
    Can be used to implement real_status, estimated_system_status
    """

    def __init__(self, initial_d=0, initial_v=0, initial_a=0):
        self.d = initial_d
        self.v = initial_v
        self.a = initial_a

        
class Car():
    """
    Car class
    Car have ground_truth, estimated_system_status, update_public_status()
    ground_truth is the real real status
    estimated_system_status is the status the car knows, this may be corrupted by sensor
    noise.
    update_public_status() should return the the estimated_system_status, however, if the car
    is malicious, this function could return different value.
    black_box records nearly everything. Including the fuel usage.
    use functions provided by black_blox to absract or plot driving history.
    """

    def __init__(self,
                 initial_status, delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma
                 ):
        self.delta_t = delta_t
        self.ground_truth = deepcopy(initial_status)
        self.system_measurement = deepcopy(initial_status)
        self.estimated_system_status = deepcopy(initial_status)
        self.public_status = deepcopy(initial_status)
        self.desired_action = deepcopy(initial_status)

        self.sensor = Sensor(self.ground_truth, self.system_measurement, sensor_noise_center,sensor_noise_sigma)
        self.filter = Filter(self.system_measurement, self.estimated_system_status, delta_t, sensor_noise_sigma)

        self.controller = Controller(self.estimated_system_status, self.desired_action, delta_t)

        self.actuator = Actuator(
            self.desired_action, self.ground_truth, actuator_noise_center, actuator_noise_sigma, delta_t)
        self.black_box = BlackBox(self.ground_truth, self.system_measurement, self.estimated_system_status, delta_t, self.filter)

    def update(self):
        self.actuator.update()
        self.sensor.update()
        self.filter.update()
        self.black_box.record()
        self.update_public_status()
        self.controller.update()

    def update_public_status(self):
        for p in dir(self.public_status):
            if not p.startswith('__'):
                setattr(self.public_status, p, getattr(self.estimated_system_status, p))


class LeadCar(Car):
    def __init__(self,
                 initial_status, delta_t, 
                 sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma,
                 a_file=None
                 ):
        super().__init__(initial_status, delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma)
        self.controller = LeadCarController(
            self.estimated_system_status, self.desired_action, a_file, delta_t)


class DecentralizedFollowingCar(Car):
    """
    Car class
    Car have GroundTruth, SystemStateEstimator
    GroundTruth is the real real status
    SystemStateEstimator is the status the car knows, this may be corrupted by sensor
    noise.
    get_status this should return the the estimated_system_status, however, if the car
    is malicious, this function could return different value.
    """

    def __init__(self,
                 initial_status, delta_t,
                 sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma,
                 front_car, safty_dist
                 ):
        super().__init__(initial_status, delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma)
        self.front_car_ground_truth = front_car.ground_truth
        # Used as the input to the sensor to the front car
        self.front_car_measurement = deepcopy(front_car.system_measurement)
        self.front_car_transmit = front_car.public_status
        # This should not be referenced in case the attacker controller send modifies the data.
        self.estimated_front_car_status = deepcopy(front_car.estimated_system_status)

        self.front_car_sensor = Sensor(self.front_car_ground_truth, self.front_car_measurement, sensor_noise_center,sensor_noise_sigma)
        self.front_car_filter = FrontCarFilter(self.front_car_measurement, self.front_car_transmit, self.estimated_front_car_status, delta_t, sensor_noise_sigma)
        self.controller = DecentralizedController(
            self.estimated_system_status, self.estimated_front_car_status, self.desired_action, safty_dist, delta_t)
        self.black_box = BlackBox(self.ground_truth, self.system_measurement, self.estimated_system_status, delta_t, self.filter, self.front_car_ground_truth, self.front_car_measurement, self.estimated_front_car_status)

    def crash_rest(self):
        if self.front_car_ground_truth.d-self.ground_truth.d<=0:
            self.ground_truth.d = self.front_car_ground_truth.d - 0.01
            self.ground_truth.v = self.front_car_ground_truth.v
            self.ground_truth.a = self.front_car_ground_truth.a
            return True
        else:
            return False


    def update(self):
        self.actuator.update()
        ifcrashed = self.crash_rest()
        self.sensor.update()
        self.filter.update()
        self.update_public_status()
        self.front_car_sensor.update()
        self.front_car_filter.update()
        fuel = fuel_consumption(self.ground_truth.a, self.ground_truth.v, self.front_car_ground_truth.d-self.ground_truth.d, self.delta_t)
        self.black_box.record(fuel, ifcrashed)
        self.controller.update()


class CentralizedFollowingCar(DecentralizedFollowingCar):
    """
    Car class
    Car have GroundTruth, SystemStateEstimator
    GroundTruth is the real real status
    SystemStateEstimator is the status the car knows, this may be corrupted by sensor
    noise.
    get_status this should return the the estimated_system_status, however, if the car
    is malicious, this function could return different value.
    """

    def __init__(self,
                 initial_status, delta_t,
                 sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma,
                 front_car,
                 safty_dist,
                 lead_car
                 ):
        super().__init__(initial_status, delta_t, sensor_noise_center,sensor_noise_sigma, actuator_noise_center, actuator_noise_sigma, front_car, safty_dist)
        self.lead_car_ground_truth = lead_car.ground_truth
        # Used as the input to the sensor to the front car
        self.lead_car_measurement = deepcopy(lead_car.system_measurement)
        self.lead_car_transmit = lead_car.public_status
        # This should not be referenced in case the attacker controller send modifies the data.
        self.lead_car_status = deepcopy(lead_car.estimated_system_status)

        self.lead_car_filter = Filter(
            self.lead_car_transmit, self.lead_car_status, delta_t, sensor_noise_sigma)

        self.controller = CentralizedController(
            self.estimated_system_status, self.estimated_front_car_status, self.lead_car_status, self.desired_action, safty_dist, delta_t)

    def update(self):
        self.actuator.update()
        ifcrashed = self.crash_rest()
        self.sensor.update()
        self.filter.update()
        self.update_public_status()
        self.front_car_sensor.update()
        self.front_car_filter.update()
        self.lead_car_filter.update()
        fuel = fuel_consumption(self.ground_truth.a, self.ground_truth.v, self.front_car_ground_truth.d-self.ground_truth.d, self.delta_t)
        self.black_box.record(fuel, ifcrashed)
        self.controller.update()


class Sensor():
    """The sensor of a Car
    Add sensor noise and return noisy measurement.
    """

    def __init__(self, ground_truth, system_measurement, sensor_noise_center,sensor_noise_sigma):
        self.sensor_noise_center = sensor_noise_center
        self.sensor_noise_sigma = sensor_noise_sigma
        self.ground_truth = ground_truth
        self.system_measurement = system_measurement

    def update(self):
        """
        Read from real status. Inject noise.
        Be careful not to modify the original value when add noise.
        """
        self.system_measurement.d = self.ground_truth.d + \
            np.random.normal(self.sensor_noise_center, self.sensor_noise_sigma)
        self.system_measurement.v = self.ground_truth.v + \
            np.random.normal(self.sensor_noise_center, self.sensor_noise_sigma)
        self.system_measurement.a = self.ground_truth.a + \
            np.random.normal(self.sensor_noise_center, self.sensor_noise_sigma)


class Actuator():
    """ Get desired acceleration. Add noise and modify the real status
    """

    def __init__(self, desired_action, ground_truth, actuator_noise_center, actuator_noise_sigma, delta_t):
        self.actuator_noise_center = actuator_noise_center
        self.actuator_noise_sigma = actuator_noise_sigma
        self.desired_action = desired_action
        self.ground_truth = ground_truth
        self.delta_t = delta_t

    def update(self):
        _a = self.desired_action.a + \
            np.random.normal(0, self.actuator_noise_sigma) + self.actuator_noise_center
        _a = min(1, max(_a, -1))
        if abs(_a)<0.0005:
            _a = 0.0
        self.ground_truth.d += self.ground_truth.v * self.delta_t + 0.5 * _a * self.delta_t * self.delta_t
        self.ground_truth.v += _a * self.delta_t
        self.ground_truth.a = _a
