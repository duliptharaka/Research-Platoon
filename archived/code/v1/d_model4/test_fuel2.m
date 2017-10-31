clc;
clear all;

m1 = 0.442*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));

m9 = 1.442*(10^(-6));
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
load('first_car_a.mat');
time = zeros(step,1);
y = zeros(step,1);
y1 = zeros(step,1);
v_us = zeros(step,1);
avg_v_us = zeros(step,1);
avg_a = zeros(step-1,1);
fuel_before = zeros(step-1,1);
fuel_before2 = zeros(step-1,1);
fuel_after = zeros(step-1,1);
fuel_after2 = zeros(step-1,1);
mpg_before = zeros(step-1,1);
mpg_before2 = zeros(step-1,1);
mpg_after = zeros(step-1,1);
mpg_after2 = zeros(step-1,1);
first_car_s = zeros(step-1,1);
avg_s = zeros(step-1,1);


% Average the velocity before calculating the fuel cunsumption
avg_v = zeros(step,1);
avg_v(1) = first_car_v(1);
for i = 2:step
    avg_v(i) = (first_car_v(i-1) + first_car_v(i) + first_car_v(i))/3;
end

% Convert velocity to us unit
for i = 1:step
    v_us(i) = first_car_v(i)/ 0.44704;
    avg_v_us(i) = avg_v(i)/0.44704;
end

% Calculate fuel consumption
for i = 1:step-1
    time(i) = i;
    y(i) = 0;
    y1(i) = -1;
    v = first_car_v(i);
    a = first_car_a(i);
    first_car_s(i) = v*delta_t + (a/2)*delta_t^2;
    f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
    fuel_before(i) = integral(f,0,1);   
    mpg_before(i) = (first_car_s(i) * 0.000621371)/(fuel_before(i)*0.264172/0.75);
    f = @(x) m9*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
    fuel_before2(i) = integral(f,0,1);  
    mpg_before2(i) = (first_car_s(i) * 0.000621371)/(fuel_before2(i)*0.264172/0.75);   
    
    v = avg_v(i);
    avg_a(i) = (avg_v(i+1) - avg_v(i))/delta_t;
    a = avg_a(i);
    avg_s(i) = v*delta_t + (a/2)*delta_t^2;
    f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
    fuel_after(i) = integral(f,0,1);  
    mpg_after(i) = (avg_s(i) * 0.000621371)/(fuel_after(i)*0.264172/0.75);
    f = @(x) m9*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
    fuel_after2(i) = integral(f,0,1);  
    mpg_after2(i) = (avg_s(i) * 0.000621371)/(fuel_after2(i)*0.264172/0.75);    
    
end
time(step) = step;

% Calculate mpg

mpg_before_avg = (sum(first_car_s) * 0.000621371)/(sum(fuel_before)*0.264172/0.75);
mpg_after_avg = (sum(avg_s) * 0.000621371)/(sum(fuel_after)*0.264172/0.75);
mpg_before_avg2 = (sum(first_car_s) * 0.000621371)/(sum(fuel_before2)*0.264172/0.75);
mpg_after_avg2 = (sum(avg_s) * 0.000621371)/(sum(fuel_after2)*0.264172/0.75);




subplot(2,2,1);
plot(time, v_us(1:step,:), 'b', time, avg_v_us, 'r');
set(gca,'fontsize',12)
ylim([25 75]);
xlabel('time [second]');
ylabel('velocity [mile/hour]');
legend('Original HWFET', 'Data smoothed in 3-sec time window');
title('Velocity comparison before and after the data smoothing');

subplot(2,2,2);
plot(time(1:step-1,:), first_car_a(1:step-1,:)-avg_a, 'k');
set(gca,'fontsize',12)
ylim([-0.3 0.3]);
xlabel('time [second]');
ylabel('acceleration [m/s^2]');
%legend('Acceleration difference');
title('Acceleration difference before and after the data smoothing');

subplot(2,2,3);
plot(time(1:step-1,:),first_car_a(1:step-1,:), 'b', time(1:step-1,:),y1(1:step-1,:), 'c--');
set(gca,'fontsize',12)
xlabel('time [second]');
ylabel('acceleration [m/s^2]');
%legend('Original HWFET');
title('Acceleration of the original HWFET');

subplot(2,2,4);
plot(time(1:step-1,:), avg_a, 'r', time(1:step-1,:),y1(1:step-1,:), 'c--');
set(gca,'fontsize',12)
xlabel('time [second]');
ylabel('acceleration [m/s^2]');
%legend('Data smoothed in 3-sec time window');
title('Accleration of the smoothed data');


figure 

subplot(2,2,1);
plot(time(1:step-1,:),fuel_before, 'b', time(1:step-1,:),fuel_after, 'r', time(1:step-1,:), y(1:step-1,:), 'g--');
ylim([-0.5*10^(-3) 3*10^(-3)]);
xlabel('time[second]');
ylabel('fuel consumption rate [kg/s]');
legend('Original HWFET', 'Data smoothed in 3-sec time window');
title('fuel consumption with literature* coefficients');

subplot(2,2,3);
plot(time(1:step-1,:),mpg_before, 'b', time(1:step-1,:),mpg_after, 'r');
ylim([-50 200]);
xlabel('time[second]');
ylabel('mpg [mile/gallon]');
legend('Original HWFET', 'Data smoothed in 3-sec time window');
title('mpg with literature* coefficients');

subplot(2,2,2);
plot(time(1:step-1,:),fuel_before2, 'b', time(1:step-1,:),fuel_after2, 'r', time(1:step-1,:), y(1:step-1,:), 'g--');
ylim([-0.5*10^(-3) 3*10^(-3)]);
xlabel('time[second]');
ylabel('fuel consumption rate [kg/s]');
legend('Original HWFET', 'Data smoothed in 3-sec time window');
title('fuel consumption with modified coefficients');

subplot(2,2,4);
plot(time(1:step-1,:),mpg_before2, 'b', time(1:step-1,:),mpg_after2, 'r');
ylim([-50 200]);
xlabel('time[second]');
ylabel('mpg [mile/gallon]');
legend('Original HWFET', 'Data smoothed in 3-sec time window');
title('mpg with modified coefficients');











    