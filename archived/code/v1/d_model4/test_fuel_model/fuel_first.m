function [ output ] = fuel_first(t, start_t, start_v, start_a)
v = start_v + start_a*(t-start_t);
a = start_a;
% Fuel consumption
m1 = 1.442*(10^(-6));
m2 = 5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));
output = m1 * (v*v) + m2 * (a*a) + m3 * (v*v) * a + m4 * v * (a*a) + m5 * v * a + m6 * v + m7 * a + m8;
end

