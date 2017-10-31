clc;
clear all;
    
fx = [0.5644 0.6473 0.7258 0.7999 0.8697 0.9353 0.9967 1.0540 1.1072 1.1564 ...
  1.2016 1.2429 1.2803 1.3138 1.3435 1.3695 1.3917 1.4102 1.4250 1.4362 ...
  1.4438 1.4477 1.4482 1.4450 1.4384 1.4283 1.4147 1.3977 1.3773 1.3535 ...
  1.3263 1.2957 1.2618 1.2246 1.1841 1.1403 1.0932 1.0429 0.9893 0.9325 0.8725];
fxx = linspace(0,1,41);
x = 0:0.25:10;
f = @(xq)interp1(x,fx,xq);
tspan = [0 1];
x0 = 2;
[t,y] = ode45(@(t,x)f(x),tspan,x0);
plot(fxx,fx,t,y);