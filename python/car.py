import numpy as np
from copy import copy, deepcopy
from controller import Controller, LeadCarController, DecentralizedController, CentralizedController
from filter import Filter, FrontCarFilter


class Status():
    """Store the status of a Car
    Can be used to implement real_status, system_status
    """

    def __init__(self, initial_d=0, initial_v=0, initial_a=0, _desired_a=0):
        self.d = initial_d
        self.v = initial_v
        self.a = initial_a


class Car():
    """
    Car class
    Car have GroundTruth, SystemStateEstimator
    GroundTruth is the real real status
    SystemStateEstimator is the status the car knows, this may be corrupted by sensor
    noise.
    get_status this should return the the system_status, however, if the car
    is malicious, this function could return different value.
    """

    def __init__(self,
                 initial_status,
                 sensor_noise, actuator_noise
                 ):
        self.ground_truth = deepcopy(initial_status)
        self.system_measurement = deepcopy(initial_status)
        self.system_status = deepcopy(initial_status)
        self.public_status = deepcopy(initial_status)
        self.desired_action = deepcopy(initial_status)

        self.system_state_estimator = SystemStateEstimator(
            self.ground_truth, self.system_measurement, self.system_status, sensor_noise)  # Sensor measure system ground truth+noise, save to system_measurement, filter estimate system status

        self.controller = Controller(self.system_status, self.desired_action)

        self.actuator = Actuator(
            self.desired_action, self.ground_truth, actuator_noise)

    def update(self):
        self.actuator.update()
        self.system_state_estimator.update()
        self.update_public_status()
        self.controller.update()

    def update_public_status(self):
        for p in dir(self.public_status):
            if not p.startswith('__'):
                setattr(self.public_status, p, getattr(self.system_status, p))


class LeadCar(Car):
    def __init__(self,
                 initial_status,
                 sensor_noise, actuator_noise
                 ):
        super().__init__(initial_status, sensor_noise, actuator_noise)
        self.controller = LeadCarController(
            self.system_status, self.desired_action)


class DecentralizedFollowingCar(Car):
    """
    Car class
    Car have GroundTruth, SystemStateEstimator
    GroundTruth is the real real status
    SystemStateEstimator is the status the car knows, this may be corrupted by sensor
    noise.
    get_status this should return the the system_status, however, if the car
    is malicious, this function could return different value.
    """

    def __init__(self,
                 initial_status,
                 sensor_noise, actuator_noise,
                 front_car, safty_dist
                 ):
        super().__init__(initial_status, sensor_noise, actuator_noise)
        self.front_car_ground_truth = front_car.ground_truth
        # Used as the input to the sensor to the front car
        self.front_car_measurement = deepcopy(front_car.system_measurement)
        self.front_car_transmit = front_car.public_status
        # This should not be referenced in case the attacker controller send modifies the data.
        self.front_car_status = deepcopy(front_car.system_status)

        self.front_car_state_estimator = FrontCarStateEstimator(
            self.front_car_ground_truth, self.front_car_measurement, self.front_car_transmit, self.front_car_status, sensor_noise)

        self.controller = DecentralizedController(
            self.system_status, self.front_car_status, self.desired_action, safty_dist)

    def update(self):
        self.actuator.update()
        self.system_state_estimator.update()
        self.update_public_status()
        self.front_car_state_estimator.update()
        self.controller.update()


class CentralizedFollowingCar(DecentralizedFollowingCar):
    """
    Car class
    Car have GroundTruth, SystemStateEstimator
    GroundTruth is the real real status
    SystemStateEstimator is the status the car knows, this may be corrupted by sensor
    noise.
    get_status this should return the the system_status, however, if the car
    is malicious, this function could return different value.
    """

    def __init__(self,
                 initial_status,
                 sensor_noise, actuator_noise,
                 front_car, safty_dist,
                 lead_car
                 ):
        super().__init__(initial_status, sensor_noise,
                         actuator_noise, front_car, safty_dist)
        self.lead_car_ground_truth = lead_car.ground_truth
        # Used as the input to the sensor to the front car
        self.lead_car_measurement = deepcopy(lead_car.system_measurement)
        self.lead_car_transmit = lead_car.public_status
        # This should not be referenced in case the attacker controller send modifies the data.
        self.lead_car_status = deepcopy(lead_car.system_status)

        self.lead_car_state_estimator = Filter(
            self.lead_car_transmit, self.lead_car_status)

        self.controller = CentralizedController(
            self.system_status, self.front_car_status, self.lead_car_status, self.desired_action, safty_dist)

    def update(self):
        self.actuator.update()
        self.system_state_estimator.update()
        self.update_public_status()
        self.front_car_state_estimator.update()
        self.lead_car_state_estimator.update()
        self.controller.update()


class Sensor():
    """The sensor of a Car
    Will store data influenced by noise
    """

    def __init__(self, ground_truth, system_measurement, sensor_noise=0):
        self.sensor_noise = sensor_noise
        self.ground_truth = ground_truth
        self.system_measurement = system_measurement

    def update(self):
        """
        Read from real status. Inject noise.
        Be careful not to modify the original value when add noise.
        """
        self.system_measurement.d = self.ground_truth.d + \
            np.random.normal(0, self.sensor_noise)
        self.system_measurement.v = self.ground_truth.v + \
            np.random.normal(0, self.sensor_noise)
        self.system_measurement.a = self.ground_truth.a + \
            np.random.normal(0, self.sensor_noise)


class SystemStateEstimator():
    """Store the system measurement and estimation status
    Only used to store data. Read from sensor, update filter.
    """

    def __init__(self, ground_truth, system_measurement, system_status, sensor_noise):
        self.sensor = Sensor(ground_truth, system_measurement, sensor_noise)
        self.filter = Filter(system_measurement, system_status)

    def update(self):
        self.sensor.update()
        self.filter.update()



class FrontCarStateEstimator(SystemStateEstimator):
    """Store the status of the front Car
    This is used to gather all information about the front car
    .
    """

    def __init__(self, front_car_ground_truth, front_car_measurement, front_car_transmit, front_car_status, sensor_noise):
        super().__init__(front_car_ground_truth,
                         front_car_measurement, front_car_status, sensor_noise)
        self.filter = FrontCarFilter(
            front_car_measurement, front_car_transmit, front_car_status)


class Actuator():
    """ Get desired acceleration. Add noise and modify the real status
    """

    def __init__(self, desired_action, ground_truth, noise_level=0):
        self.noise_level = noise_level
        self.desired_action = desired_action
        self.ground_truth = ground_truth

    def update(self):
        _a = self.desired_action.a + \
            np.random.normal(0, self.noise_level)
        _a = min(1, max(_a, -1))
        self.ground_truth.d += self.ground_truth.v + 0.5 * _a
        self.ground_truth.v += _a
        self.ground_truth.a = _a
