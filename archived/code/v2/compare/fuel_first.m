function [ output ] = fuel_first(v, s, a)
%COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
% b0 = 0.1569+5;
% b1 = 2.45 * (10^(-2));
% b2 = -7.415 * (10^(-4));
% b3 = 5.975 * (10^(-5));
% c0 = 0.07224;
% c1 = 9.681 * (10^(-2));
% c2 = 1.075 * (10^(-3));
% M = 1200;
% C_D = 0.32;
% pho = 1.184;
% A_v = 2.5;
% miu = 0.015;
% g = 9.8;
% % target velocity
% V_d = 30;
% %h_d = 1.3;
% h_d=80;
% w1 = 4;
% w3 = 1.25*100;
% %w3 = 1.25 * (10^(2));
% alpha = 0.2;
% % Critical distance
% R = 30;
% % Subject to change
% r = 3;
% w4 = 2;
% w5 = 1.25* (10^(2));
% 
% %a_hat = (-0.5/M) * C_D * pho * A_v * (v^2) - miu * g * cos(grade(s)) + a;
% 
% %output = (b0 + b1*v+b2*(v^2) + b3*(v^3) + a*(c0+c1*v+c2*(v^2)));
% control_a = a - (-0.5/M) * C_D * pho * A_v * (v^2) + miu * g * cos(grade(s));
% output = ((b0 + b1*v+b2*(v^2) + b3*(v^3) + a*(c0+c1*v+c2*(v^2))*heaviside(control_a) + 0.2*heaviside(-control_a);
% Fuel consumption
m1 = 1.442*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));
f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
output = integral(f,0,1);
%output = m1 * (v^2) + m2 * (a^2) + m3 * (v^2) * a + m4 * v * (a^2) + m5 * v * a + m6 * v + m7 * a + m8;
end

