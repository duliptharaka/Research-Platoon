function [ s2,v2,d2] = update_last( s1,v1,d1,a1,v_front, a_front, delta_t)
%UPDATE Summary of this function goes here
%   Detailed explanation goes here
s2 = v1*delta_t + (a1/2) * (delta_t^2) + s1;
v2 = v1 + a1 * delta_t;
d2 = d1 + (v_front*delta_t + (a_front/2)*((delta_t)^2)) - (v1*delta_t + (a1/2)*((delta_t)^2));
end

