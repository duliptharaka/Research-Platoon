% x = -4:1:4;             % The range of x values.
% y = -4:1:4;             % The range of y values.
% [X,Y] = meshgrid (x,y); % This generates the actual grid of x and y values.
% Z=Y.^2+X.^2;            % The function we're plotting.
% surf(X,Y,Z)
clear all;
w1 = 0.442*(10^(-6));
w2 = -5.67*(10^(-6));
w3 = 1.166*(10^(-6));
w4 = 39.269*(10^(-6));
w5 = 58.284*(10^(-6));
w6 = 19.279*(10^(-6));
w7 = 82.426*(10^(-6));
w8 = 185.36*(10^(-6));
%v = [12;15;18;21;24;27];
v_min = 0;
v_max = 25;
a_max = 3;
a_min = -2;
a1 = zeros(100,1);
v1 = zeros(100,1);
fuel = zeros(100,100);
for i =1:100
    v1(i) = v_min + i*((v_max-v_min)/100);    
    for j=1:100
        a1(j) = a_min + j*((a_max-a_min)/100);
        v = v1(i);
        a= a1(j);
        fuel(j,i) = w1 * (v^2) + w2 * (a^2) + w3 * (v^2) * a + w4 * v * (a^2) + w5 * v * a + w6 * v + w7 * a + w8;
    end
end
%[V,A] = meshgrid(v,a);
%Z = w1 * (V.^2) + w2 * (A.^2) + w3 * (V.^2) * A + w4 * V * (A.^2) + w5 * V * A + w6 * V + w7 * A + w8;

surf(a1,v1,fuel)
xlabel('Acceleration [m/s^2]');
ylabel('Velocity [m/s]');
zlabel('Fuel consumption rate [kg/s]');
axis([-2 3 0 25 -5*10^-3 20*10^-3]);