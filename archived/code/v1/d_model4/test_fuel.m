clc;
clear all;

w1 = 0.442*(10^(-6));
w2 = -5.67*(10^(-6));
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



v_test = [12;15;18;21;24;27];
[m,n] = size(v_test);
% 12.51
v_min = 28*0.44704;
% 26.8224
v_max = 60*0.44704;
a_max = 8000/8052.9706513958;
a_min = -12000/8052.9706513958;
a_test = zeros(100,1);
fuel = zeros(m,100);

delta_t = 1;

for j = 1:m;
    for i = 1:100
        a_test(i) = a_min + i*((a_max-a_min)/100);
        a = a_test(i);
        v = v_test(j);
        %one_d = v*delta_t + 0.5*a*(delta_t^2);
        %a_hat = phi(d) * (-0.5/M) * C_D * pho * A_v * (v^2) - miu * g * cos(grade(s)) + a;
        % mL/s
        fuel(j,i) = w1 * (v^2) + w2 * (a^2) + w3 * (v^2) * a + w4 * v * (a^2) + w5 * v * a + w6 * v + w7 * a + w8;
        % Convert to mpg
        %fuel(i,j) = (one_d* 0.000621371 )/(fuel(i,j)*delta_t/1000*0.264172);
    end
end
    
% Plot the result
plot(a_test, fuel(1,:),a_test,fuel(2,:),a_test,fuel(3,:),a_test,fuel(4,:),a_test,fuel(5,:),a_test,fuel(6,:));
%surf(a_test,v_test,fuel);
xlabel('Acceleration [m/s^2]');
%ylabel('Fuel consumption [mL/s]');
legend('26.8 mile/hour','33.6 mile/hour','40.3 mile/hour','47.0 mile/hour' ,'53.7 mile/hour','60.4 mile/hour');
ylabel('Fuel consumption rate[kg/s]');


    
    
    
    