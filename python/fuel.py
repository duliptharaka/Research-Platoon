"""
Calculates the fuel in liter will used in the following one second.
We use 2001 civic sedan as the reference car to adjust the following model.
City 32 mpg/ hwy 39 mpg.
Car fuel consumption when idle: https://www.energy.gov/eere/vehicles/fact-861-february-23-2015-idle-fuel-consumption-selected-gasoline-and-diesel-vehicles

Assume car idle consumption is 0.00016824052 liter per second
Car cruse at 55 mile with 39 mpg fuel consumption:  0.001509 liter per second

At 55 mph, 20 deg C, the adjusted model have combined 34 mpg / hwy 38.8 mpg when drive solo, combined 36.4 mpg/ hwy 42.9 mpg in platoon(1 m between cars).
"""


def fuel(accelration, velocty, distance_to_front_car):
    __rho_a = 1.2041  # Density of air at 20 deg C
    __CdA = 0.682  # Drag area in m^2 of the 2001 civic sedan https://en.wikipedia.org/wiki/Automobile_drag_coefficient
    __E_g = 34.8 * 1e6  # Energy J in a liter of gasoline  https://en.wikipedia.org/wiki/Fuel_efficiency
    __eta = .295  # Estimated Civic engine efficiency https://en.wikipedia.org/wiki/Fuel_efficiency
    __air_drag = (__rho_a * __CdA) / (2 * __E_g * __eta)

    _delta_d = velocty + 0.5 * accelration
    f_air = (
        velocty**2 *
        _drag_reduction_ratio(distance_to_front_car) *
        _delta_d
    ) * __air_drag
    f_car = ((1 + accelration) * 15 + velocty / 10) ** 3 / \
        10000000 + (velocty - 30) ** 4 / 50000000 + 0.0004
    return f_air + f_car


def _drag_reduction_ratio(distance_to_front_car):
    # Estimated with http://www.academia.edu/7968462/The_Aerodynamic_Performance_Of_Platoons_A_Final_Report
    __length_of_vehicle = 4.43  # This is only used to calculate air reduction. Not for collision.
    return 1 - 1 / (((distance_to_front_car / __length_of_vehicle) * 2 + 1.6)**2)
