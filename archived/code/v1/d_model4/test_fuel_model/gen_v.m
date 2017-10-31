clc;
clear all;

step = 1000;
d_t = 1;
object = [50,55,60,65,70];
v_avg = zeros(1,5);
const_output = zeros(5,1);
sine_output = zeros(5,1);





time = zeros(step, 1);
const_fuel = zeros(step-1, 1);
const_s = zeros(step, 5);
const_v = zeros(step, 5);
const_v_us = zeros(step, 5);
const_a = zeros(step-1,5);
sine_v = zeros(step, 5);
sine_a = zeros(step-1,5);
sine_fuel = zeros(step-1, 1);
sine_s = zeros(step, 5);
sine_v_us = zeros(step, 5);
sine_average = zeros(step-1,1);

m1 = 1.442*(10^(-6));
%m2 = -5.67*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));

for j = 1:5
    v_avg(j) = object(j)*0.44704;
    % 1 mph = 0.447 04 m/s
    for i = 1:step
        time(i) = i;
        const_v(i,j) = v_avg(j);
        const_v_us(i,j) = v_avg(j)/0.44704;
        sine_v(i,j) = v_avg(j)*sin(2*pi*time(i)/100) + v_avg(j);   
        sine_v_us(i,j) = sine_v(i,j)/0.44704;
    end
end

for j = 1:5
    for i = 1:step-1
        const_a(i,j) = 0;
        const_s(i,j) = const_v(i,j)*d_t;     
        v = const_v(i,j);
        a = const_a(i,j);
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        const_fuel(i, j) = integral(f,0,1);   
        
        sine_a(i,j) = (sine_v(i+1,j) - sine_v(i,j))/d_t;
        sine_s(i,j) = sine_v(i,j)*d_t + (d_t^2/2)*sine_a(i,j);  
        v = sine_v(i,j);
        a = sine_a(i,j);
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        sine_fuel(i, j) = integral(f,0,1);  
    end
end

% Calculate average mpg
const_mpg  = zeros(1,5);
sine_mpg = zeros(1,5);
for j = 1:5
    const_mpg(j) = (sum(const_s(:,j)) * 0.000621371)/(sum(const_fuel(:,j))*0.264172/0.75);
    sine_mpg(j) = (sum(sine_s(:,j)) * 0.000621371)/(sum(sine_fuel(:,j))*0.264172/0.75);   
end

% 
% for j = 1:5
%     const_dist_cul = 0;
%     const_fuel_cul = 0;
%     sine_dist_cul = 0;
%     sine_fuel_cul = 0;
%     for i = 1:step-1
%         % For the constant driving pattern
%         const_a(i) = 0;
%         one_d = const_v(i,j)*d_t;
%         const_dist_cul = const_dist_cul + one_d;
%         %const_fuel(i) = fuel_first(const_v(i), const_a(i));
%         v = const_v(i,j);
%         a = const_a(i,j);
%         f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
%         const_fuel(i) = integral(f,0,1);
%         const_fuel_cul = const_fuel_cul + const_fuel(i);
%         const_mpg(i,j) = (const_dist_cul * 0.000621371)/(const_fuel_cul*0.264172/0.75);
% 
%         % For the sine driving pattern
%         sine_a(i,j) = (sine_v(i+1,j) - sine_v(i,j))/d_t;
%         one_d = sine_v(i,j)*d_t + (d_t^2/2)*sine_a(i,j);
%         sine_dist_cul = sine_dist_cul + one_d;
%         %sine_fuel(i) = fuel_first(sine_v(i), sine_a(i));
%         v = sine_v(i,j);
%         a = sine_a(i,j);
%         sine_fuel(i) = integral(f,0,1);
%         sine_fuel_cul = sine_fuel_cul + sine_fuel(i);
%         sine_mpg(i,j) = ( sine_dist_cul* 0.000621371)/(sine_fuel_cul*0.264172/0.75);
%     end
% %     sine_mean_mpg = mean(sine_mpg);
% %     for i = 1:step-1
% %         sine_average(i) = sine_mean_mpg;
% %     end
%     const_output(j) = (const_dist_cul * 0.000621371)/(const_fuel_cul*0.264172/0.75);
%     sine_output(j) = (sine_dist_cul * 0.000621371)/(sine_fuel_cul*0.264172/0.75);
% end

% subplot(2,2,1)
% plot(time, const_v_us(:,1),time, const_v_us(:,2),time, const_v_us(:,3),time, const_v_us(:,4),time, const_v_us(:,5));
% xlabel('time [second]');
% ylabel('velocity [mile/hour]');
% legend('average = 50 mile/hour', 'average = 55 mile/hour', 'average = 60 mile/hour', 'average = 65 mile/hour', 'average = 70 mile/hour');
% title('constant driving pattern');
% ylim([45 75]);
% 
% subplot(2,2,2)
% plot(time, sine_v_us(:,1), time, sine_v_us(:,2), time, sine_v_us(:,3), time, sine_v_us(:,4), time, sine_v_us(:,5));
% legend('average = 50 mile/hour', 'average = 55 mile/hour', 'average = 60 mile/hour', 'average = 65 mile/hour', 'average = 70 mile/hour');
% xlabel('time [second]');
% ylabel('velocity [mile/hour]');
% title('sine driving pattern');


% subplot(2,2,3)
% plot(time(1:step-1,:), const_mpg(1:step-1,1), time(1:step-1,:), const_mpg(1:step-1,2),time(1:step-1,:), const_mpg(1:step-1,3),time(1:step-1,:), const_mpg(1:step-1,4),time(1:step-1,:), const_mpg(1:step-1,5) );
% legend('average = 50 mile/hour', 'average = 55 mile/hour', 'average = 60 mile/hour', 'average = 65 mile/hour', 'average = 70 mile/hour');
% xlabel('time [second]');
% ylabel('mpg [mile/gallon]');
% ylim([45 85]);
% title('fuel consumption of constant driving pattern with literature coefficients');
% 
% subplot(2,2,4)
% plot(time(1:step-1,:), sine_mpg(:,1), time(1:step-1,:), sine_mpg(:,2), time(1:step-1,:), sine_mpg(1:step-1,3), time(1:step-1,:), sine_mpg(1:step-1,4), time(1:step-1,:), sine_mpg(1:step-1,5));
% legend('average = 50 mile/hour', 'average = 55 mile/hour', 'average = 60 mile/hour', 'average = 65 mile/hour', 'average = 70 mile/hour');
% xlabel('time [second]');
% ylabel('mpg [mile/gallon]');
% title('fuel consumption of sine driving pattern with literature coefficients');
% 
