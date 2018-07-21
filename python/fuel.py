"""
Calculates the fuel in liter will used in the following delta_t second.
References:
We use 2001 civic sedan as the reference car to adjust the following model.
City 32 mpg/ hwy 39 mpg.
Car fuel consumption when idle 0.16gal/hr: https://www.energy.gov/eere/vehicles/fact-861-february-23-2015-idle-fuel-consumption-selected-gasoline-and-diesel-vehicles
Assume car idle consumption is 0.0001682405237333323 liter per second
Assume car cruse at 55 mile/hr(24.587199692837032 meter/s) with 39 mpg fuel consumption:  0.0014828892316239228 liter per second

Adjusted model:

At 55 mph, 20 deg C, the adjusted model have combined 36.2949 mpg / hwy 39 mpg when drive solo, 
combined 38.5843 mpg/ hwy 41.9 mpg in platoon(1 m between cars).

>>> fuel_consumption(0,0,99999,1) # idle
0.00016903313049357677
>>> fuel_consumption(0,24.587199692837032,99999,1) # 55 mile/hr
0.0014849488283655043
>>> mpg(5)
6.4202668811137764
>>> mpg(15)
17.424567739522754
>>> mpg(25)
26.891325024870568
>>> mpg(35)
33.988972676896779
>>> mpg(45)
38.002785948776214
>>> mpg(50)
38.825954560221334
>>> mpg(55)
38.945907716557414
>>> mpg(60)
38.470640026716879
>>> mpg(65)
37.522917259789864
>>> mpg(75)
34.688991497872806
>>> mpg(100)
26.100533756171416
>>> mpg(200)
8.3828756577025274

"""
import numpy as np

def meter_mile(travel_distant):
    return (travel_distant*0.0006213712)

def liter_galon(fuel_usage):
    return (fuel_usage*0.26417205235815)

def galon_liter(fuel_usage):
    return (fuel_usage*3.7854117839999772)

def mile_meter(travel_distant):
    return (travel_distant*1609.3439798947877)

def mph_mps(mph):
    return mile_meter(mph)/3600

def mpg_lpm(mpg):
    return galon_liter(1/mile_meter(mpg))

def mpg_lps(mpg,mile_p_hour):
    return galon_liter(mile_p_hour/mpg)/3600

def lps_mpg(liter_p_second, meter_p_second):
    return meter_mile(meter_p_second)/liter_galon(liter_p_second)

def mpg(mph):
    return lps_mpg(fuel_consumption(0,mph_mps(mph),9999,1),mph_mps(mph))


def _drag_reduction_ratio(distance_to_front_car):
    # Estimated with http://www.academia.edu/7968462/The_Aerodynamic_Performance_Of_Platoons_A_Final_Report
    __length_of_vehicle = 4.445  # This is only used to calculate air reduction. Not for collision.
    return 1 - 0.42537*2.718**(-0.86798*(distance_to_front_car / __length_of_vehicle))
    

def fuel_consumption(accelration, velocty, distance_to_front_car, delta_t):
    __rho_a = 1.2  # Estimated density of air at 20 deg C above sea level
    __CdA = 0.682  # Drag area in m^2 of the 2001 civic sedan https://en.wikipedia.org/wiki/Automobile_drag_coefficient
    __E_drag = (0.5*__rho_a*velocty**3*__CdA*_drag_reduction_ratio(distance_to_front_car)*delta_t) # Energy cost by air drag
    __E_gas = 34.8 * 1e6  # Energy J in a liter of gasoline  https://en.wikipedia.org/wiki/Fuel_efficiency
    __E_vehicle = (((accelration*10)**2*1500) + np.log2(max(2,velocty*1e4+2))*3000 + 6500)/5.1*delta_t # Estimated vehicle energy profile
    __eta = .35  # Estimated Civic energy efficiency https://en.wikipedia.org/wiki/Energy-efficient_driving
    fuel = (__E_vehicle + __E_drag)/(__E_gas *__eta)
    return fuel

