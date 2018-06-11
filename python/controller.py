def optimal_a(delta_d, delta_v, target_a, delta_t):
    """
    This controller is created based on Dr. Pisu's idea of using objective function to get a balance among the control of the car and the extra gas consumption caused by rapid acceleration. That idea sounds good but does not make sense since the optimal way to save gas under this scenario is to keep the car at the desired safety spot, where the air drag is also close to minimum. When consider the gas consumption caused by rapid acceleration, the car will keep slow down, until a rapid acceleration and slowdown again and cause oscillations.
    This controller does not handle many problems that a PID controller could handle.
    That means, this controller should only performs worse than a simple PID controller with anti windup.

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
    """
    w1 = 0.333334 # D
    w2 = 0.333333 # V
    w3 = 0.333333 # A
    return (2*w1*delta_d*delta_t**2+4*w2*delta_v*delta_t+4*w3*target_a)/(w1*delta_t**4+4*w2*delta_t**2+4*w3)


class Controller():
    """ controller base class
    Cruise at initial speed
    """

    def __init__(self, system_status, desired_action, cruise_speed=25, delta_t=0.01):
        self.system_status = system_status
        self.desired_action = desired_action
        self.cruise_speed = cruise_speed
        self.delta_t = delta_t


    def update(self):
        # Cruise under current speed
        self.desired_action.a = 0
        self.desired_action.d = self.system_status.d + self.cruise_speed * self.delta_t #\
            #self.system_status.v + 0.5 * self.desired_action.a
        self.desired_action.v = self.cruise_speed #self.system_status.v + self.desired_action.a

class LeadCarController(Controller):
    """Lead car controller
    If no driving pattern given, car will cruise.
    """
    def __init__(self, system_status, desired_action, a_file=None, delta_t=0.1):
        super().__init__(system_status, desired_action, delta_t=delta_t)
        if a_file:
            self.a_file = self.file_reader(a_file)

    def file_reader(self, file):
        with open(file, 'r') as f:
            for line in f:
                yield float(line)

    def update(self):
        # Cruise under current speed
        try:
            self.desired_action.a = next(self.a_file)
            self.desired_action.d = self.system_status.d + self.desired_action.v * self.delta_t + 0.5*self.desired_action.a * self.delta_t
            self.desired_action.v = self.desired_action.v + self.desired_action.a * self.delta_t
        except AttributeError as e:
            super().update()


class DecentralizedController(Controller):
    """Decentralized controller
    """

    def __init__(self, system_status, front_car_status, desired_action, safty_dist, delta_t):
        super().__init__(system_status, desired_action, delta_t=delta_t)
        self.front_car_status = front_car_status
        self.safty_dist = safty_dist
        self.delta_t = delta_t

    def update(self):
        _delta_d = self.front_car_status.d + self.front_car_status.v * self.delta_t + 0.5 * self.front_car_status.a * self.delta_t * self.delta_t - \
            self.safty_dist - \
            self.system_status.d - self.system_status.v * self.delta_t
        _delta_v = self.front_car_status.v + self.front_car_status.a * self.delta_t - self.system_status.v
        _target_a = self.front_car_status.a
        self.desired_action.a = optimal_a(_delta_d, _delta_v, _target_a, self.delta_t)
        self.desired_action.d = self.system_status.d + \
            self.system_status.v * self.delta_t + 0.5 * self.desired_action.a * self.delta_t * self.delta_t
        self.desired_action.v = self.system_status.v + self.desired_action.a * self.delta_t


class CentralizedController(Controller):
    """Decentralized controller
    """

    def __init__(self, system_status, front_car_status, lead_car_status, desired_action, safty_dist, delta_t):
        super().__init__(system_status, desired_action, delta_t=delta_t)
        self.front_car_status = front_car_status
        self.lead_car_status = lead_car_status
        self.safty_dist = safty_dist
        self.delta_t = delta_t

    def update(self):
        _delta_d = self.front_car_status.d + self.front_car_status.v * self.delta_t +  0.5 * self.front_car_status.a * self.delta_t * self.delta_t- \
            self.safty_dist - self.system_status.d - self.system_status.v * self.delta_t
        _delta_v = self.lead_car_status.v + self.lead_car_status.a * self.delta_t  - self.system_status.v
        _target_a = self.lead_car_status.a
        self.desired_action.a = optimal_a(_delta_d, _delta_v, _target_a, self.delta_t)
        self.desired_action.d = self.system_status.d + \
            self.system_status.v * self.delta_t + 0.5 * self.desired_action.a * self.delta_t * self.delta_t
        self.desired_action.v = self.system_status.v + self.desired_action.a * self.delta_t
