% Clean up workspace
clear all;
close all;

% Number of trials
num_trials = 10;

% Simulation variables
sigma_a = 0.0005;
sigma_m = 0.0005;
SF = 5;

car_mpgs = [];
car_collisions = [];

for idx=1:num_trials
  car_results = decentralized_platoon(sigma_a, sigma_m, SF);
  car_mpgs = vertcat(car_mpgs, car_results(1,:));
  car_collisions = vertcat(car_collisions, car_results(2,:));
end

% Stats!
% mean(mean(car_mpgs));
% var(mean(car_mpgs,1));