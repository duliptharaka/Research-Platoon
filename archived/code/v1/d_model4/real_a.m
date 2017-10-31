function [ output ] = real_a( a, s, v, d)

M = 1200;
C_D = 0.32;
pho = 1.184;
A_v = 2.5;
miu = 0.015;
g = 9.8;
output = phi(d) * (-0.5/M) * C_D * pho * A_v * (v^2) - miu * g * cos(grade(s)) + a;

end

