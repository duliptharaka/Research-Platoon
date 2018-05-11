This is a python3 implementation of the platoon project.
Hopefully the code will be so much easier to read and understand and maintainable.

Objects we need:

Car:
    RealStatus:
        # Real GPS, speed, acceleration, should be updated by the actuator may be corrupted by actuator noise.
    SystemStatus:
        # System GPS, speed, acceleration, should be updated by the sensor, may be corrupted by sensor noise.
    FronCarStatus:
        # Stores the measured and received status of front car.
        # Can be modified by the sensor, data receiver, and filter
    Sensor:
        measure():
            # Read from real local GPS, speed, t0_acceleration
            SensorNoiseGenerator
            # Update local GPS, speed, t0_acceleration
    AttackDataInjector:
        update():
            modify any data if necessary
    DataReceiver:
        update():
            Read front car's public GPS data, speed, acceleration,
    Filter:
    Controller:
        update():
            # Calculate t1_acceleration
    Actuator:
        update()
            # update RealStatus + noise
    update():
        AttackDataInjector.update()
        Sensor.update()
        DataReceiver.update()
        Filter.apply_filter()
        Controller.update()
        Actuator.update()


    get_status():
            # Called by other Cars or controller.
            # return public GPS, speed, t0_acceleration, maybe influenced by 
            # AttackDataInjector.update()

LeadCar(Car):
    update():
        DataLoader
            # update_control_t1_acceleration
            
            # update_public_t1_acceleration
            ActuatorNoiseGenerator
            # update_real_t1_acceleration
            

CentralizedController:
    DataReceiver:
    Filter:
    Controller:
        AttackDataInjector
    DataTransmitter:
        push_control_signal_to_cars
    update():
        DataReceiver.receive_car_transmit_data()
        Filter.apply_filter()
        Controller.global_control()
        DataTransmitter.push_control_signal_to_cars()

Platoon:
    CentralizedController:
    CarController:
    LeadCar:
    FollowingCar:
    cars = [LeadCar]+FollowingCars

MPG:

Engine:
    load Platoon
    DataCollector:
        car_position_vs_time
        inter_car_distance_vs_time
        car_velocity_vs_time
        car_acceleration_vs_time
        collision_count_vs_time
    update():
        read_t0_car_data() # Read data from cars
        calculate_t1_car_positions()
        calculate_t1_inter_car_distance()
        calculate_t1_car_velocity()
        calculate_t1_car_acceleration()
        calculate_t1_car_gas_usage()
        collision_detection()
            reset_car_position()
        write_t1_car_data() # Write data back to cars' environmental parameters
        DataCollector.log_data()
        Platoon.read_data_from_cars()
        Platoon.CentralizedController.update()
        for car in Platoon.cars:
            car.update()
    run():
        while:
            self.update()
    Figure Plotter

Main:
    Engine.generate_cars()
    Engine.run()
    Engine.Plot
