clc;
clear all;

% Load the velocity and acceleration
load('first_car_v');
load('first_car_a');

% Parameters
step = 701;
delta_t = 1;
time = zeros(step,1);
avg_first_car_v = zeros(step,1);
v_us = zeros(step,1);
v_us_old = zeros(step,1);
avg_first_car_a = zeros(step-1,1);
avg_first_car_v(1) = first_car_v(1);


% Calculate the average speed on a 3-sec time window
for i = 2:step
    time(i) = i;
    avg_first_car_v(i) = (first_car_v(i-1) + first_car_v(i) + first_car_v(i+1))/3;
end

% Change unit for velocity
for i = 1:step
    v_us_old(i) = first_car_v(i)/0.44704;
    v_us(i) = avg_first_car_v(i)/0.44704;   
end

% Calculate the acceleration from the averaged speed
for i = 1:step-1
    avg_first_car_a(i) = (avg_first_car_v(i+1)-avg_first_car_v(i))/delta_t;
end

avg_first_car_v(step) = [];

% Save into file
save('avg_first_car_v.mat','avg_first_car_v');
save('avg_first_car_a.mat','avg_first_car_a');

% Plot the results
subplot(2,1,1)
plot(time, v_us_old(1:step,:), 'b', time, v_us, 'r');
xlabel('time[second]')
ylabel('velocity [mile/hour]');
legend('real velocity','average velocity on a 3-sec window');

subplot(2,1,2)
plot(time(1:step-1,:), first_car_a(1:step-1,:),'b', time(1:step-1,:), avg_first_car_a(1:step-1,:), 'r');
xlabel('time [second]');
ylabel('acceleration [m/s^2]');
legend('real acceleration','average acceleration');

