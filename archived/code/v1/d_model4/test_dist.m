clc;
clear all;
step = 100;
d = zeros(1,step);
output = zeros(1,step);
alpha = 0.2;
r = 3;
R = 30;
h_d = 3;
v = 30;
for i = 1:step
    d(i) = i;
    w2 = r * exp(-alpha*d(i));
    R_e = R + h_d*v - d(i);
    w22 = r * exp(-alpha*(2*R-d(i)))*0.05;
    R_b = R + h_d*v - (2*R-d(i));
    output(i) = w2*R_e^2+w22*R_b^2;
end
plot(d,output);
ylim([0 2000]);
xlim([0 80]);


