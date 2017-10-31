% Clean up workspace
clear all;
close all;
% Comment out for MATLAB
graphics_toolkit('gnuplot');

load('velocity_profile.mat');
lead_velocity = v_profile; 
lead_fuel = [];
lead_fuel_drag = [];
lead_dist = 0;
lead_accel = [];
for idx = 2:size(lead_velocity,1)
  a = lead_velocity(idx) - lead_velocity(idx-1);
  lead_dist = lead_dist + (lead_velocity(idx-1) + lead_velocity(idx))/2;
  fc = fuel_consumption(a, lead_velocity(idx-1));
  dc = fuel_consumption_drag(a, lead_velocity(idx-1), 1e8, (lead_velocity(idx-1) + lead_velocity(idx))/2);
  lead_fuel = vertcat(lead_fuel,fc);
  lead_fuel_drag = vertcat(lead_fuel_drag,dc);
  lead_accel = vertcat(lead_accel,a);
end
sum(lead_fuel_drag)
sum(lead_fuel)
sum(lead_fuel_drag)/(sum(lead_fuel)+sum(lead_fuel_drag))
lead_mpg = (lead_dist*0.000621371)/(sum(lead_fuel) + sum(lead_fuel_drag))

% Simulation Variables
w_1 = 1;
w_2 = 1-w_1;
SF = 5;
% Fuel consumption difference due to acceleration decisions
% Lots of jitter is causing issues.
% TODO: Units of the fuel_consumption function
accel_tolerance = 0.0005;

% Store the results of the simulation
car_traveled = [];
car_fuel = [];
car_dists = [];
car_vels = [];
car_accels = [];
car_mpg = [];

% Iterate through each car in the platoon
for car_idx=1:5
  % Store the results for each car in a vector
  dist_traveled_vec = [];
  delta_dist_vec = [];
  velocity_vec = [v_profile(1)];
  accel_vec = [];
  fuel_vec = [];
  fuel_drag_vec = [];
  
  % Set the starting distance
  dist = SF;
  % Set the starting velocity
  velocity = v_profile(1);

  % Iterate through each timestep
  % This i is actually 'i+1'
  % but it's easier to calculate distance 
  for i=2:size(v_profile,1) 
    % Solving the objective function based on infromation from time i - 1
    optimal_a = w_1*(dist - SF)/(w_1/2 + 2*w_2);
    
    % High pass filter.
    if abs(optimal_a) < accel_tolerance
      optimal_a = 0;  
    end
    
    % Calculate new velocity for i
    velocity = velocity + optimal_a;
    % Calculate new distance given lead car's acceleration and
    % Following car's acceleration
    % Calculating distance at i+1
    dist = dist + distance(v_profile(i),v_profile(i-1),1);
    dist = dist - (velocity + optimal_a/2);
    
    % Store the results of this step
    dist_traveled_vec = vertcat(dist_traveled_vec, (velocity + optimal_a/2));
    delta_dist_vec = vertcat(delta_dist_vec,dist);
    velocity_vec = vertcat(velocity_vec,velocity);
    accel_vec = vertcat(accel_vec,optimal_a);
    fc = fuel_consumption(optimal_a, velocity - optimal_a);
    dc = fuel_consumption_drag(optimal_a, velocity - optimal_a, dist, (velocity + optimal_a/2));
    fuel_vec = vertcat(fuel_vec,fc);
    fuel_drag_vec = vertcat(fuel_drag_vec, dc);
  end
  
  % Store this car's results as a matrix
  % For car i, the rows of column i in matrix car_, represent all the values for that car
  car_traveled = horzcat(car_traveled,sum(dist_traveled_vec));
  car_fuel = horzcat(car_fuel, fuel_vec);
  car_dists = horzcat(car_dists,delta_dist_vec);
  car_vels = horzcat(car_vels,velocity_vec);
  car_accels = horzcat(car_accels,accel_vec);
  
  car_mpg = horzcat(car_mpg, (sum(dist_traveled_vec)*0.000621371)/(sum(fuel_vec) + sum(fuel_drag_vec)));
  
  sum(fuel_drag_vec)
  sum(fuel_vec)
  sum(fuel_drag_vec)/(sum(fuel_vec) + sum(fuel_drag_vec))
  printf("---\n");
  % Set the new velocity profile
  v_profile = velocity_vec;
end

car_mpg
mean(car_mpg)
%improvements = (lead_mpg - car_mpg)./car_mpg

% Plotting
f1 = figure;
subplot(2,1,2);
plot(car_dists, '-', 'linewidth',2);
axis([250 350 3.5 7.5]);
set(gca, 'FontSize', 14,'linewidth',3,'fontname','times');
xlabel('Time (s)','FontSize',14);
ylabel('\Delta Distance (m)','FontSize',14);
legend('Car 1','Car 2', 'Car 3', 'Car 4', 'Car 5','location','eastoutside');
subplot(2,1,1);
plot(lead_velocity, '-', 'linewidth',2);
axis([250 350 0 30]);
set(gca, 'FontSize', 14,'linewidth',3,'fontname','times');
xlabel('Time (s)','FontSize',14);
ylabel('Velocity (m/s)','FontSize',14);
legend('Car 0','location','eastoutside');
%print(f1, 'media/part1.eps','-color', '-loose', '-deps');


f2 = figure;
subplot(2,1,1);
plot(v_profile(2:end),'-','color', 'black', 'linewidth',2);
axis([0 700 0 35]);
set(gca, 'FontSize', 14,'linewidth',3,'fontname','times');
xlabel('Time (s)','FontSize',14);
ylabel('Velocity (m/s)','FontSize',14);
legend('Car 1','location','southeast');
subplot(2,1,2);
plot(car_accels(:,1),'-','color', 'black','linewidth',2);
set(gca, 'FontSize', 14,'linewidth',3,'fontname','times');
xlabel('Time (s)','FontSize',14);
ylabel('Acceleration (m/s^2)','FontSize',14);
legend('Car 1');
axis([0 700 -1.5 1.5]);
%print(f2, 'media/accleration.eps','-color', '-loose', '-deps');
