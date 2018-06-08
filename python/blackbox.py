from matplotlib import pyplot as plt

class BlackBox(object):
    """GPS Tracker"""
    def __init__(self, ground_truth, system_measurement, estimated_system_status, delta_t, filter, front_car_ground_truth=None, front_car_measurement=None, estimated_front_car_status=None):
        self.ground_truth = ground_truth
        self.system_measurement = system_measurement
        self.estimated_system_status = estimated_system_status
        self.delta_t = delta_t
        self.filter = filter
        self.ground_truth_a = []
        self.ground_truth_d = []
        self.ground_truth_v = []
        self.system_measurement_a = []
        self.system_measurement_d = []
        self.system_measurement_v = []
        self.estimated_system_status_a = []
        self.estimated_system_status_d = []
        self.estimated_system_status_v = []
        self.last_position = self.ground_truth.d
        self.travel_distance = 0
        self.crash_counter = 0
        self.eps = []
        if type(front_car_ground_truth)==type(ground_truth):
            self.front_car_ground_truth = front_car_ground_truth
            self.front_car_measurement = front_car_measurement
            self.estimated_front_car_status = estimated_front_car_status
            self.ground_truth_delta_d = []
            self.system_measurement_delta_d = []
            self.estimated_system_status_delta_d = []
            self.fuel_consumption = 0

    def record(self, fuel=0, crashed=False):
        self.ground_truth_a.append(self.ground_truth.a)
        self.ground_truth_d.append(self.ground_truth.d)
        self.ground_truth_v.append(self.ground_truth.v)
        self.system_measurement_a.append(self.system_measurement.a)
        self.system_measurement_d.append(self.system_measurement.d)
        self.system_measurement_v.append(self.system_measurement.v)
        self.estimated_system_status_a.append(self.estimated_system_status.a)
        self.estimated_system_status_d.append(self.estimated_system_status.d)
        self.estimated_system_status_v.append(self.estimated_system_status.v)
        self.eps.append(self.filter.eps)
        try:
            self.ground_truth_delta_d.append(self.front_car_ground_truth.d-self.ground_truth.d)
            self.system_measurement_delta_d.append(self.front_car_measurement.d-self.system_measurement.d)
            self.estimated_system_status_delta_d.append(self.estimated_front_car_status.d-self.estimated_system_status.d)
            self.fuel_consumption += fuel
        except AttributeError:
            pass
        self.travel_distance += abs(self.ground_truth.d - self.last_position)
        self.last_position = self.ground_truth.d
        if crashed:
            self.crash_counter += 1

    def plot_record(self):
        plt.figure(1)
        plt.subplot(311)
        plt.scatter(list(range(len(self.ground_truth_a))), self.ground_truth_a,marker='o', label='Real Acceleration')
        plt.scatter(list(range(len(self.system_measurement_a))), self.system_measurement_a,marker='^', label='Measured Acceleration')
        plt.scatter(list(range(len(self.estimated_system_status_a))), self.estimated_system_status_a,marker='x', label='Filtered Acceleration')
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (m/s^2)")

        plt.subplot(313)
        plt.scatter(list(range(len(self.ground_truth_v))), self.ground_truth_v,marker='o', label='Real Acceleration')
        plt.scatter(list(range(len(self.system_measurement_v))), self.system_measurement_v,marker='^', label='Measured Acceleration')
        plt.scatter(list(range(len(self.estimated_system_status_v))), self.estimated_system_status_v,marker='x', label='Filtered Acceleration')
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m/s)")

        plt.subplot(312)
        plt.ylim(ymax=10)
        plt.plot(list(range(len(self.eps))), self.eps, label='eps')
        plt.legend()
        plt.xlabel("Time (s)")

        #plt.subplot(313)
        #plt.scatter(list(range(len(self.ground_truth_delta_d))), self.ground_truth_delta_d,marker='o', label='Real Acceleration')
        #plt.scatter(list(range(len(self.system_measurement_delta_d))), self.system_measurement_delta_d,marker='^', label='Measured Acceleration')
        #plt.scatter(list(range(len(self.estimated_system_status_delta_d))), self.estimated_system_status_delta_d,marker='x', label='Filtered Acceleration')
        #plt.legend()
        #plt.xlabel("Time (s)")
        #plt.ylabel("Delta Distance (m)")
        plt.show()

    def save_record(self, file_name='BlackBoxRecord.pdf'):
        pass


