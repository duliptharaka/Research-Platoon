function [ p1,p2,p3,p4,p5, fuel, all] = cost_value_middle(x, v, v_front,v_behind, d_front, TotalS, TotalF, delta_t)
a = x(1);
a_front = x(2);
a_behind = x(3);
s = v*delta_t + (a/2)*delta_t^2;
v_next = v + a*delta_t;
v_front_next = v_front + a_front*delta_t;
v_behind_next = v_behind + a_behind*delta_t;
d_front_next = d_front + (v_front-v)*delta_t + (delta_t^2/2)*(a_front-a);

% Paramters
% r = 3;
% alpha = 0.2;
% w1 = 4*(10^7);
% w3 = 1250;
% w4 = 2;
% w5 = 125;
% R = 30;
% h_d = 3;

[ w1, w3, w4, w5, r, alpha, R, h_d ] = read_coff();

w2 = r * exp(-alpha*d_front_next);
R_e = R + h_d*v_next - d_front_next;

w6 = r * exp(-alpha*(2*R-d_front_next));
R_b = R + h_d*v_next - (2*R-d_front_next);

% w6 = r * exp(-alpha*(2*R-d_front))*0.05;
% R_b = R + h_d*v - (2*R-d_front);

% M = 1200;
% C_D = 0.32;
% pho = 1.184;
% A_v = 2.5;
% miu = 0.015;
% g = 9.8;
% Convert control a to the combined a
%a = phi(d_front) * (-0.5/M) * C_D * pho * A_v * (v^2) - miu * g * cos(grade(s)) + control_a;

% Fuel consumption
m1 = 1.442*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));
%fuel = m1 * (v^2) + m2 * (a^2) + m3 * (v^2) * a + m4 * v * (a^2) + m5 * v * a + m6 * v + m7 * a + m8;
f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
fuel = integral(f,0,1);

%p1 = w1*fuel/s;
% p1 = w1*(TotalF + fuel)/(TotalS + s);
% p2 = w2*(R_e^2) + w6*(R_b^2);
% p3 = w3*((v_next - v_front_next)^2);
% p4 = w4*a^2;
% p5 = w5*((v_next - v_behind_next)^2);
% all = p1 + p2 + p3 + p4 + p5;

p1 = w1*fuel/s;
p2 = w2*(R_e^2) + 200*((d_front_next+700)^0.5)+5*((d_front_next+700)^2);
p3 = w3*((v_next - v_front_next)^2)+ 1000*abs(v_front_next-v_next);
p4 = w4*a^2;
p5 = w5*((v_next - v_behind_next)^2);
all = p1 + p2 + p3 + p4 + p5;


end

