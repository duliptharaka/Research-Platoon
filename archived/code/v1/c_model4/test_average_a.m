clc;
clear all;

% Load the velocity and acceleration
load('first_car_v');
load('first_car_a');

% Parameters
step = 700;
delta_t = 1;
% Fourth-order runge-kutta methods
% f(t2,a2) = (v2-v1)/(t2-t1)
% k1 = delta_t*f(t2,a2);
% k2 = delta_t*f(t2+0.5*delta_t,a2+0.5*k1);
% k3 = delta_t*f(t2+0.5*delta_t,a2+0.5*k2);
% k4 = delta_t * f(t2+delta_t,a2+k3)
% a3 = a2 + (k1+k2+k3+k4)/6

calc_a = zeros(step-1,1);
calc_a(1) = first_car_a(1);
time = zeros(step-1,1);
time(1) = 1;
for i = 2:step-1
    time(i) = i;
    calc_a(i) = (2*(first_car_v(i+1) - first_car_v(i))/delta_t + calc_a(i-1))/3;
end

% Calculate fuel consumption
w1 = 0.442*(10^(-6));
w2 = 5.67*(10^(-6));
w3 = 1.166*(10^(-6));
w4 = 39.269*(10^(-6));
w5 = 58.284*(10^(-6));
w6 = 19.279*(10^(-6));
w7 = 82.426*(10^(-6));
w8 = 185.36*(10^(-6));
real_fuel = zeros(step-1,1);
calc_fuel = zeros(step-1,1);
real_mpg = zeros(step-1,1);
calc_mpg = zeros(step-1,1);
for i =1:step-1
    v = first_car_v(i);
    a = first_car_a(i);
    one_d = v*delta_t + 0.5*a*(delta_t^2);
    real_fuel(i) = w1 * (v^2) + w2 * (a^2) + w3 * (v^2) * a + w4 * v * (a^2) + w5 * v * a + w6 * v + w7 * a + w8;
    real_mpg(i) = (one_d * 0.000621371)/(real_fuel(i)*0.264172/0.75);
    a = calc_a(i);
    one_d = v*delta_t + 0.5*a*(delta_t^2);
    calc_fuel(i) = w1 * (v^2) + w2 * (a^2) + w3 * (v^2) * a + w4 * v * (a^2) + w5 * v * a + w6 * v + w7 * a + w8;
    calc_mpg(i) = (one_d * 0.000621371)/(calc_fuel(i)*0.264172/0.75);    
end

plot(time,first_car_a(1:step-1,:),time,calc_a);
xlabel('time [second]');
ylabel('acceleration [m/s^2]');
legend('real acceleration','calculated acceleration');

% % average = 30.7033
% average_mpg = mean(calc_mpg);
% 
% subplot(3,1,1)
% plot(time,first_car_a(1:step-1,:),time,calc_a);
% xlabel('time [second]');
% ylabel('acceleration [m/s^2]');
% legend('real acceleration','calculated acceleration');
% 
% subplot(3,1,2)
% plot(time,real_fuel,time,calc_fuel);
% xlabel('time [second]');
% ylabel('fuel consumption rate[kg/s]');
% legend('real fuel consumption','calculated fuel consumption');
% 
% subplot(3,1,3)
% plot(time,real_mpg,time,calc_mpg);
% xlabel('time [second]');
% ylabel('mpg [mile/gallon]');
% legend('real mpg','calculated mpg');
% 
% 
% suptitle('acceleration and fuel consumption comparison when w1 = 0.442*10^{-6}');
% 
% 
