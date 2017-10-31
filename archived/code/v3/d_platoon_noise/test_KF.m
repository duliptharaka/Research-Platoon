clc;
clear all;
% Read into u and w
s = importdata('first_car_s_3.mat');
v = importdata('first_car_v.mat');
a = importdata('first_car_a.mat');
n = 100;
t = (1:n)';
% Convert to metric
real_x = zeros(2,n);
est_x = zeros(2,n);
obs_x = zeros(2,n);
real_a = zeros(n);
y = zeros(2,n);
for i = 1:n
    %real_x(1,i) = s(i)/0.000621371;
    %real_x(2,i) = v(i)*0.44704;
    %real_x(1,i) = s(i);
    %real_x(2,i) = v(i);
    real_a(i) = a(i);
end
real_x(1,1) = s(1);
real_x(2,1) = v(1);

dt = 1;
A = [1,dt;0,1];
B = [dt^2/2;dt];
C = [1,0;0,1];
%Q = 2.3;
%R = 1;
q = 0.3;
r = 0.1;
dm = 2;
Q = q*eye(dm);
R= r*eye(dm);


p = [0,0;0,0];
obs_x(:,1) = C*real_x(:,1) + sqrt(r)*randn(2,1);
est_x(:,1) = obs_x(:,1);
for i = 1:(n-1)
    % Real value and observation   
    real_x(:,i+1) = A*real_x(:,i) + B*real_a(i) + sqrt(q)*randn(2,1);
    obs_x(:,i) = C*real_x(:,i) + sqrt(r)*randn(2,1);
    % Predict
    it_x = A*est_x(:,i) + B*real_a(i);
    p = A*p*inv(A) + Q;

    % update
    k = p*transpose(C)*inv(C*p*transpose(C) + R);
    est_x(:,i+1) = it_x + k*(obs_x(:,i) - C*it_x);
    p = (eye(2) - k*C)*p;
end

% Plot the results
figure
subplot(2,1,1)
plot(t,real_x(1,:),t,obs_x(1,:), t, est_x(1,:));    
legend('real','observation','estimation');
title('position (m)');
subplot(2,1,2)
plot(t,real_x(2,:),t,obs_x(2,:), t, est_x(2,:));    
legend('real','observation','estimation');
title('velocity (m/s)');




%Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
% innovation gain M
%[kalmf,L,P,M] = kalman(Plant,Q,R);
% The first output of the Kalman filter KALMF is the plant output estimate y_e = Cx[n|n]
% and the remaining outputs are the state estimates.
%kf = kalmf(1,:);
