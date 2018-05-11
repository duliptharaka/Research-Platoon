def optimal_a(delta_d, target_v, target_a):
    """
    This controller is created based on Dr. Pisu's idea of using objective function to get a balance among the control of the car and the extra gas consumption caused by rapid acceleration. That idea sounds good but does not make sense since the optimal way to save gas under this scenario is to keep the car at the desired safety spot, where the air drag is also close to minimum. When consider the gas consumption caused by rapid acceleration, the car will keep slow down, until a rapid acceleration and slowdown again and cause oscillations.
    This controller does not handle many problems that a PID controller could handle.
    That means, this controller should only performs worse than a simple PID controller with anti windup.

    Objective function: 
    
    D: distance to the safety spot if current car keep current speed
    V: difference of velocity
    A: target acceleration
    
    min(w3(D-0.5a)^2+w1(V-a)^2+w2(A-a)^2)
    (w3(D-0.5a)^2+w1(V-a)^2+w2(A-a)^2)d(a) = 0
    a = (4*A*w3+2*D*w1+4*V*w2)/(4*w3+w1+4*w2)
    """
    w1 = 0.3333
    w2 = 0.3333
    w3 = 0.3333
    return (2*delta_d*w1 + 4*target_v*w2 + 4*target_a*w3)/(4*w3+w1+4*w2)


class Controller():
    """ controller base class
    Cruise at initial speed
    """

    def __init__(self, system_status, desired_action):
        self.system_status = system_status
        self.desired_action = desired_action

    def update(self):
        # Cruise under current speed
        self.desired_action.a = 0
        self.desired_action.v = self.system_status.v
        self.desired_action.d = self.system_status.d + self.desired_action.v


class LeadCarController(Controller):
    """Lead car controller
    """

    def update(self):
        self.desired_action.a = 0
        self.desired_action.d = self.system_status.d + \
            self.system_status.v + 0.5 * self.desired_action.a
        self.desired_action.v = self.system_status.v + self.desired_action.a


class DecentralizedController(Controller):
    """Decentralized controller
    """

    def __init__(self, system_status, front_car_status, desired_action, safty_dist):
        super().__init__(system_status, desired_action)
        self.front_car_status = front_car_status
        self.safty_dist = safty_dist

    def update(self):
        _delta_d = self.front_car_status.d + self.front_car_status.v + 0.5 * self.front_car_status.a - \
            self.safty_dist - \
            self.system_status.d - self.system_status.v
        _delta_v = self.front_car_status.v + self.front_car_status.a - self.system_status.v
        _target_a = self.front_car_status.a
        self.desired_action.a = optimal_a(_delta_d, _delta_v, _target_a)
        self.desired_action.d = self.system_status.d + \
            self.system_status.v + 0.5 * self.desired_action.a
        self.desired_action.v = self.system_status.v + self.desired_action.a


class CentralizedController(Controller):
    """Decentralized controller
    """

    def __init__(self, system_status, front_car_status, lead_car_status, desired_action, safty_dist):
        super().__init__(system_status, desired_action)
        self.front_car_status = front_car_status
        self.lead_car_status = lead_car_status
        self.safty_dist = safty_dist

    def update(self):
        _delta_d = self.front_car_status.d + self.front_car_status.v - \
            self.safty_dist - self.system_status.d - self.system_status.v
        _delta_v = self.lead_car_status.v - self.system_status.v
        _target_a = self.lead_car_status.a
        self.desired_action.a = optimal_a(_delta_d, _delta_v, _target_a)
        self.desired_action.d = self.system_status.d + \
            self.system_status.v + 0.5 * self.desired_action.a
        self.desired_action.v = self.system_status.v + self.desired_action.a
