"""
Calculates the fuel in liter will use in the following delta_t second.
References:
We use 2001 civic sedan as the reference car to adjust the following model.
City 32 mpg/ hwy 39 mpg.
Car fuel consumption when idle: 0.16gal/hr  https://www.energy.gov/eere/vehicles/fact-861-february-23-2015-idle-fuel-consumption-selected-gasoline-and-diesel-vehicles
Assume car fuel consumption when idle is 0.0001682405237333323 liter per second
Assume car fuel consumption when cruse at 55 mph(24.587199692837032 meter/s) is 39 mpg(0.0014828892316239228 liter per second)

Adjusted model (At 20 deg C, slightly above sea level):
Drive solo:
 combined:          36.2949 mpg 
 cruse at 55 mph:   39 mpg 

In platoon(1 m between cars):
 combined:          38.5843 mpg
 cruse at 55 mph:   41.9 mpg 

Sample points:
>>> fuel_consumption(0,0,99999,1) # idle
0.00016903313049357677
>>> fuel_consumption(0,24.587199692837032,99999,1) # 55 mile/hr
0.0014849488283655043
>>> mile_per_galon(5) # 5 mile/hr
6.4202668811137764
>>> mile_per_galon(15) # 15 mile/hr
17.424567739522754
>>> mile_per_galon(25)
26.891325024870568
>>> mile_per_galon(35)
33.988972676896779
>>> mile_per_galon(45)
38.002785948776214
>>> mile_per_galon(50)
38.825954560221334
>>> mile_per_galon(55)
38.945907716557414
>>> mile_per_galon(60)
38.470640026716879
>>> mile_per_galon(65)
37.522917259789864
>>> mile_per_galon(75)
34.688991497872806
>>> mile_per_galon(100)
26.100533756171416
>>> mile_per_galon(200)
8.3828756577025274

"""
import numpy as np

def _drag_reduction_ratio(distance_to_front_car):
    # All units are metric unit.
    # Estimated with http://www.academia.edu/7968462/The_Aerodynamic_Performance_Of_Platoons_A_Final_Report
    __length_of_vehicle = 4.445  # This is only used to calculate air reduction. Not for collision.
    return 1 - 0.42537*2.718**(-0.86798*(distance_to_front_car / __length_of_vehicle))
    

def fuel_consumption(accelration, velocty, distance_to_front_car, delta_t):
    # All units are metric unit.
    __rho_a = 1.2  # Estimated density of air at 20 deg C above sea level
    __CdA = 0.682  # Drag area in m^2 of the 2001 civic sedan https://en.wikipedia.org/wiki/Automobile_drag_coefficient
    __E_drag = (0.5*__rho_a*velocty**3*__CdA*_drag_reduction_ratio(distance_to_front_car)*delta_t) # Energy cost by air drag
    __E_gas = 34.8 * 1e6  # Energy J in a liter of gasoline  https://en.wikipedia.org/wiki/Fuel_efficiency
    __E_vehicle = (((accelration*10)**2*1500) + np.log2(max(2,velocty*1e4+2))*3000 + 6500)/5.1*delta_t # Estimated vehicle energy profile
    __eta = .35  # Estimated Civic energy efficiency https://en.wikipedia.org/wiki/Energy-efficient_driving
    fuel = (__E_vehicle + __E_drag)/(__E_gas *__eta)
    return fuel # Liter


# Unit conversions:
def meter_to_mile(meter):
    return (meter*0.0006213712)

def liter_to_galon(liter):
    return (liter*0.26417205235815)

def galon_to_liter(galon):
    return (galon*3.7854117839999772)

def mile_to_meter(mile):
    return (mile*1609.3439798947877)

def mile_per_hour_to_meter_per_second(mile_per_hour):
    return mile_to_meter(mile_per_hour)/3600

def meter_per_second_to_mile_per_hour(meter_per_second):
    return meter_to_mile(meter_per_second*3600)

def mile_per_galon_to_liter_per_meter(mile_per_galon):
    return galon_to_liter(1/mile_to_meter(mile_per_galon))

def mile_per_galon_to_liter_per_second(mile_per_galon,mile_per_hour):
    return galon_to_liter(mile_per_hour/mile_per_galon)/3600

def liter_per_second_to_mile_per_galon(liter_per_second, meter_per_second):
    return meter_to_mile(meter_per_second)/liter_to_galon(liter_per_second)

def mile_per_galon(mile_per_hour):
    return liter_per_second_to_mile_per_galon(fuel_consumption(0,mile_per_hour_to_meter_per_second(mile_per_hour),9999,1),mile_per_hour_to_meter_per_second(mile_per_hour))

