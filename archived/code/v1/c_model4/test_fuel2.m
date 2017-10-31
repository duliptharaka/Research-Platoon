clc;
clear all;

w1 = 1.442*(10^(-6));
w2 = 5.67*(10^(-6));
w3 = 1.166*(10^(-6));
w4 = 39.269*(10^(-6));
w5 = 58.284*(10^(-6));
w6 = 19.279*(10^(-6));
w7 = 82.426*(10^(-6));
w8 = 185.36*(10^(-6));
% r = 0.014;
% Japan
% w1 = 0.208*(10^(-6));
% w2 = 40.899*(10^(-6));
% w3 = -0.532*(10^(-6));
% w4 = 45.419*(10^(-6));
% w5 = 86.518*(10^(-6));
% w6 = 23.923*(10^(-6));
% w7 = 26.602*(10^(-6));
% w8 = 160.64*(10^(-6));
% FTP
% w1 = 0.222*(10^(-6));
% w2 = 4.332*(10^(-6));
% w3 = 1.25*(10^(-6));
% w4 = 36.217*(10^(-6));
% w5 = 57.877*(10^(-6));
% w6 = 24.245*(10^(-6));
% w7 = 77.482*(10^(-6));
% w8 = 171.2*(10^(-6));

step = 700;
delta_t = 1;
load('first_car_v.mat');
time = zeros(step,1);
v_us = zeros(step,1);
first_car_a = zeros(step-1,1);
fuel = zeros(step-1,1);
mpg = zeros(step-1,1);

% Average the velocity before calculating the fuel cunsumption
avg_v = zeros(step,1);
avg_v(1) = first_car_v(1);
for i = 2:step
    avg_v(i) = (first_car_v(i-1) + first_car_v(i) + first_car_v(i))/3;
end




for i =1:step-1
    time(i) = i;
    v = avg_v(i);
    v_us(i) = v/ 0.44704;
    first_car_a(i) = avg_v(i+1)-avg_v(i);
    a = first_car_a(i);
    one_d = v*delta_t + 0.5*a*(delta_t^2);
    fuel(i) = w1 * (v^2) + w2 * (a^2) + w3 * (v^2) * a + w4 * v * (a^2) + w5 * v * a + w6 * v + w7 * a + w8;
    % fuel density is 0.75kg/L
    % 1kg fuel = 0.264172/0.75 gallon
%     if fuel(i)<0
%         fuel(i) = 0.01;
%     end
    mpg(i) = (one_d * 0.000621371)/(fuel(i)*0.264172/0.75);
end
time(step) = step;
v_us(step) = first_car_v(step)/0.44704;

subplot(3,1,1)
plot(time, v_us);
xlabel('time[second]');
ylabel('velocity [mile/hour]');

subplot(3,1,2)
plot(time(1:step-1,:), first_car_a(1:step-1,:));
xlabel('time[second]');
ylabel('acceleration [m/s]');

% subplot(3,1,3)
% plot(time(1:step-1,:),fuel);
% xlabel('time[second]');
% ylabel('Fuel consumption rate [kg/s]');

subplot(3,1,3)
plot(time(1:step-1,:),mpg);
xlabel('time[second]');
ylabel('mpg [mile/gallon]');

avg_mpg = mean(mpg);
    