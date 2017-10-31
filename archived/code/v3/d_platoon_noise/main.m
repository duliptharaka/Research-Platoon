% Main function of the platooning system simulation

% Clear the workspace
clc;
clear all;
% Delete the folder with last results
if exist('result','dir')
    delete(strcat(pwd,'/result/*.mat'));
    rmdir(strcat(pwd,'/result'));
    mkdir('result')    
else
    mkdir('result') 
end
% Create the matrix to record cpu time
start = zeros(1,4);
start(1) = cputime;
% Load the driving pattern of the first car from the files
load(fullfile(pwd,'avg_first_car_a.mat'));
load(fullfile(pwd,'avg_first_car_v.mat'));
first_car_v = avg_first_car_v;
first_car_a = avg_first_car_a;
% Initilize paramters
% % update interval is dt = 1 second
dt = 1; 
% % total time considered is 700 seconds, limited by the first car driving pattern
step = 699; 
% % max/min velocity and acceleration allowed, based on the first driving pattern
v_min = 28*0.44704; % 12.5
v_max = 60*0.44704; % 26.8
a_min = -1;
a_max = 1;
% % Coefficients of the fuel consumption
m1 = 1.442*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));
% Kalman filter paramaters
middle_A = [1 dt 0 0 0 0; 0 1 0 0 0 0; 0 -dt 1 0 dt 0; 0 0 0 1 dt 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
middle_B = [0 dt^2/2 0; 0 dt 0; dt^2/2 -dt^2/2 0; dt^2/2 0 0; dt 0 0; 0 0 dt];
middle_C = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 1];
middle_start_P = zeros(6,6);
last_A = [1 dt 0 0 0; 0 1 0 0 0; 0 -dt 1 0 dt; 0 0 0 1 dt; 0 0 0 0 1];
last_B = [0 dt^2/2; 0 dt; dt^2/2 -dt^2/2; dt^2/2 0; dt 0];
last_C = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1; 0 0 0 0 1];
last_start_P = zeros(5,5);
%for it = 3:2:13
count_ind = 1;
for number = 3:2:9
    display(['Start optimization for models with ' num2str(number) ' cars!']);
    % Initiate matrix and varibles
    output_v_behind = zeros(step+1, number);
    output_s_behind = zeros(step+1, number);
    output_a_behind = zeros(step+1, number);
    output_fuel = zeros(step, number);
    output_cost = zeros(step, number-1, 5);
    % Initiate car position so that
    % all cars are 40 meters (0.0248548 miles) apart
    % all cars start at the same veolocity/acceleration as the first car
    start_s = zeros(1,number);
    start_v = zeros(1,number);
    start_a = zeros(1,number);
    for j = 1:number
        start_s(j) = 40*(number-1) - (j-1)*40;
        start_v(j) = first_car_v(1);
        start_a(j) = first_car_a(1);
    end
    % Generate position for the first car
    first_car_s = zeros(700,1);
    first_car_s(1) = start_s(1);
    for i=2:(step+1)
        first_car_s(i) = first_car_s(i-1) + first_car_v(i-1)*dt + (first_car_a(i-1)/2)*(dt^2);
    end
    % Calculate fuel consumption for the first car
    first_car_fuel = zeros(step, 1);
    for j = 1:step
        v = first_car_v(j);
        a = first_car_a(j);
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        first_car_fuel(j,1) = integral(f,0,1);
    end
    output_fuel(:,1) = first_car_fuel;
    output_v_behind(1,:) = start_v;
    output_s_behind(1,:) = start_s;
    output_a_behind(1,:) = start_a;

    

    % SQP options 
    options = optimset('Display', 'off','Algorithm','sqp');
    %options = optimset('Algorithm','sqp');
    % -----------------------------------------------------------------------
    % Start the main function
    % Do step = 1 separately because it is a little different for KF
    i = 1;
    % Optimization for the middle car (2,3,4th car in a 5-car model)
    middle_state_last = zeros(6, 1, number-2);
    middle_control_last = zeros(3, 1,number-2);
    middle_pred = zeros(6, 1, number-2);
    middle_P_last = zeros(6, 6, number-2);
    middle_state = zeros(6, 1, number-2);
    middle_obs = zeros(8, 1, number-2);    
    middle_est = zeros(6, 1, number-2);
    middle_P = zeros(6, 6, number-2);
    middle_K_F = zeros(6, 8, number-2);
    start_s_new = zeros(1, number);
    start_v_new = zeros(1, number);
    start_a_new = zeros(1, number);    
    for j = 2:(number-1)
        % % initial state variable
        middle_state_last(:,:,j-1) = [start_s(j); start_v(j); start_s(j-1)-start_s(j); start_s(j-1); start_v(j-1); start_v(j+1)];
        middle_control_last(:,:,j-1) = [start_a(j-1); start_a(j); start_a(j+1)];
        % % calculate state updates (add plant noise)
        middle_state(:,:,j-1) = middle_A * middle_state_last(:,:,j-1) + middle_B * middle_control_last(:,:,j-1) + gen_plant_noise('middle');
        % % get the observation (add sensor noise)
        middle_obs(:,:,j-1) = middle_C * middle_state(:,:,j-1) + gen_sensor_noise('middle');
        % % minimize cost function
        v_front = (middle_obs(5,1,j-1) + middle_obs(6,1,j-1))/2;
        v_behind = (middle_obs(7,1,j-1) + middle_obs(8,1,j-1))/2;
        d_front = (middle_obs(3,1,j-1) + (middle_obs(4,1,j-1) - middle_obs(1,1,j-1)))/2;
        v = middle_obs(2,1,j-1);
        s = middle_obs(1,1,j-1);
        g = @(x)one_step_middle(x, v, v_front, v_behind, d_front, dt);
        % optimization parameter
        lb = [max(a_min,(v_min-v)/dt), max(a_min,(v_min-v_front)/dt), max(a_min,(v_min-v_behind)/dt)];
        ub = [min(a_max,(v_max-v)/dt), min(a_max,(v_max-v_front)/dt),min(a_max,(v_max-v_behind)/dt)];
        A = [1,-1,0; -1,0,1];
        b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt^2);2*(start_s(j)-start_s(j+1)+(v-v_behind)*dt)/(dt^2)];
        Aeq = [];
        beq = [];
        x0 = [0;0;0];
        x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);
        % Retrieve each element of cost function
        [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle(x, v, v_front, v_behind, d_front, dt);
        %disp('middle-first time')           
        %disp ([num2str(p1,'%.5f'),'   ',num2str(p2,'%.5f'),'   ',num2str(p3,'%.5f'),'   ',num2str(p4,'%.5f'),'   ',num2str(p5,'%.5f') ,'   ',num2str(one_fuel,'%.5f'), '   ', num2str(all,'%.5f')]);
        % retrieve result
        output_fuel(i,j) = one_fuel;
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;   
        output_cost(i,j-1,5) = p5;  
        % add plant noise
        %x(1) = x(1) + normrnd(0,25*10^(-8));
        %start_s_new(j) = s + v*dt + (x(1)/2)*(dt^2);
        %start_v_new(j) = v + x(1)*dt;
        %start_a_new(j) = x(1);
        start_s_new(j) = s;
        start_v_new(j) = v;
        start_a_new(j) = x(1);
    end

    % For the last car
    j = j+1;
    % % initial state variable
    last_state_last = [start_s(j); start_v(j); start_s(j-1)-start_s(j); start_s(j-1); start_v(j-1)];
    last_control_last = [start_a(j-1); start_a(j)];
    % % calculate state updates (add plant noise)
    last_state = last_A * last_state_last + last_B * last_control_last + gen_plant_noise('last');
    % % get the observation (add sensor noise)
    last_obs = last_C * last_state + gen_sensor_noise('last');

    % % Minimize cost function
    v_front = (last_obs(5) + last_obs(6))/2;
    d_front = (last_obs(3) + (last_obs(4)-last_obs(1)))/2;
    v = last_obs(2);
    s = last_obs(1);
    h = @(x)one_step_last(x, v, v_front, d_front, dt);
    % optimization parameter
    lb = [max(a_min,((v_min-v)/dt)), max(a_min,((v_min-v_front)/dt))];
    ub = [min(a_max,((v_max-v)/dt)), min(a_max,((v_max-v_front)/dt))];
    A = [1,-1];
    b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt^2)];
    Aeq = [];
    beq = [];
    x0 = [0;0];
    x = fmincon(h, x0, A, b, Aeq, beq, lb, ub, [], options);
    % Retrieve each element of cost function
    [p1,p2,p3,p4,one_fuel, all] = cost_value_last(x, v, v_front, d_front, dt);
    output_fuel(i,j) = one_fuel;
    output_cost(i,j-1,1) = p1;
    output_cost(i,j-1,2) = p2;        
    output_cost(i,j-1,3) = p3;
    output_cost(i,j-1,4) = p4;
    %start_s_new(j) = s + v*dt + (x(1)/2)*(dt^2);
    %start_v_new(j) = v + x(1)*dt;
    %start_a_new(j) = x(1);
    start_s_new(j) = s;
    start_v_new(j) = v;
    start_a_new(j) = x(1);    
    % For the first car
    start_s_new(1) = first_car_s(2);
    start_v_new(1) = first_car_v(2);
    start_a_new(1) = first_car_a(2);
    % update parameter
    start_s = start_s_new;
    start_v = start_v_new;
    start_a = start_a_new;
    output_v_behind(i+1,:) = start_v;
    output_s_behind(i+1,:) = start_s;
    output_a_behind(i+1,:) = start_a;
    
    %---------------------------------------------------------------------------
    % For step from 2, KF is a little bit different
    for i=2:step
        start_s_new = zeros(1, number);
        start_v_new = zeros(1, number);
        start_a_new = zeros(1, number); 
        start_s_new(1) = first_car_s(i+1);
        start_v_new(1) = first_car_v(i+1); 
        start_a_new(1) = first_car_a(i+1);   
        for j = 2:(number-1)
            % % initial state variable (add one column to the existing state variable)
            middle_state_last(:, i:i, j-1)= [start_s(j); start_v(j); start_s(j-1)-start_s(j); start_s(j-1); start_v(j-1); start_v(j+1)];
            middle_control_last(:,:,j-1) = [start_a(j-1); start_a(j); start_a(j+1)];
            % % calculate state updates (add plant noise)     
            middle_state(:,i:i,j-1) = middle_A * middle_state_last(:,i,j-1) + middle_B * middle_control_last(:,:,j-1) + gen_plant_noise('middle');
            % % get the observation (add sensor noise)
            middle_obs(:,i:i,j-1) = middle_C * middle_state(:,i,j-1) + gen_sensor_noise('middle');
            % % minimize cost function
            v_front = (middle_obs(5,i,j-1) + middle_obs(6,i,j-1))/2;
            v_behind = (middle_obs(7,i,j-1) + middle_obs(8,i,j-1))/2;
            d_front = (middle_obs(3,i,j-1) + (middle_obs(4,i,j-1) - middle_obs(1,i,j-1)))/2;
            v = middle_obs(2,i,j-1);
            s = middle_obs(1,i,j-1);
            g = @(x)one_step_middle(x, v, v_front, v_behind, d_front, dt);
            % optimization parameter
            lb = [max(a_min,(v_min-v)/dt), max(a_min,(v_min-v_front)/dt), max(a_min,(v_min-v_behind)/dt)];
            ub = [min(a_max,(v_max-v)/dt), min(a_max,(v_max-v_front)/dt),min(a_max,(v_max-v_behind)/dt)];
            A = [1,-1,0; -1,0,1];
            b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt^2);2*(start_s(j)-start_s(j+1)+(v-v_behind)*dt)/(dt^2)];
            Aeq = [];
            beq = [];
            x0 = [0;0;0];
            x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);
            % Retrieve each element of cost function
            [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle(x, v, v_front, v_behind, d_front, dt);
            %disp('middle')           
            %disp ([num2str(p1,'%.5f'),'   ',num2str(p2,'%.5f'),'   ',num2str(p3,'%.5f'),'   ',num2str(p4,'%.5f'),'   ',num2str(p5,'%.5f') ,'   ',num2str(one_fuel,'%.5f'), '   ', num2str(all,'%.5f')]);
            output_fuel(i,j) = one_fuel;
            output_cost(i,j-1,1) = p1;
            output_cost(i,j-1,2) = p2;        
            output_cost(i,j-1,3) = p3;
            output_cost(i,j-1,4) = p4;   
            output_cost(i,j-1,5) = p5;  
            %start_s_new(j) = s + v*dt + (x(1)/2)*(dt^2);
            %start_v_new(j) = v + x(1)*dt;
            %start_a_new(j) = x(1);
            start_s_new(j) = s;
            start_v_new(j) = v;
            start_a_new(j) = x(1);
        end
        % For the last car
        j = j+1;
        % % initial state variable (add one column)
        last_state_last(:,i:i) = [start_s(j); start_v(j); start_s(j-1)-start_s(j); start_s(j-1); start_v(j-1)];
        last_control_last = [start_a(j-1); start_a(j)];
        % % calculate state updates (add plant noise)
        last_state(:,i:i) = last_A * last_state_last(:,i) + last_B * last_control_last + gen_plant_noise('last');
        % % get the observation (add sensor noise)
        last_obs(:,i:i) = last_C * last_state(:,i) + gen_sensor_noise('last');
        % % Minimize cost function
        v_front = (last_obs(5,i) + last_obs(6,i))/2;
        d_front = (last_obs(3,i) + (last_obs(4,i)-last_obs(1,i)))/2;
        v = last_obs(2,i);
        s = last_obs(1,i);
        h = @(x)one_step_last(x, v, v_front, d_front, dt);
        % optimization parameter
        lb = [max(a_min,((v_min-v)/dt)), max(a_min,((v_min-v_front)/dt))];
        ub = [min(a_max,((v_max-v)/dt)), min(a_max,((v_max-v_front)/dt))];
        A = [1,-1];
        b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*dt)/(dt^2)];
        Aeq = [];
        beq = [];
        x0 = [0;0];
        x = fmincon(h, x0, A, b, Aeq, beq, lb, ub, [], options);
        % Retrieve each element of cost function
        [p1,p2,p3,p4,one_fuel, all] = cost_value_last(x, v, v_front, d_front, dt);
        output_fuel(i,j) = one_fuel;
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;
        %start_s_new(j) = s + v*dt + (x(1)/2)*(dt^2);
        %start_v_new(j) = v + x(1)*dt;
        %start_a_new(j) = x(1);
        start_s_new(j) = s;
        start_v_new(j) = v;
        start_a_new(j) = x(1);
        % update parameter
        start_s = start_s_new;
        start_v = start_v_new;
        start_a = start_a_new;
        output_v_behind(i+1,:) = start_v;
        output_s_behind(i+1,:) = start_s;
        output_a_behind(i+1,:) = start_a;    
    end
    [output_s_behind, output_v_behind, output_a_behind, output_fuel] = change_unit(output_s_behind, output_v_behind, output_a_behind, output_fuel, dt);
    save(fullfile(strcat(pwd,'/result'),strcat('output_v_behind_', num2str(number), '.mat')), 'output_v_behind', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_a_behind_', num2str(number), '.mat')), 'output_a_behind', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_s_behind_', num2str(number), '.mat')), 'output_s_behind', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_fuel_', num2str(number), '.mat')), 'output_fuel', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_cost_', num2str(number), '.mat')), 'output_cost', '-mat');
    start(count_ind) = cputime - start(count_ind);
    count_ind = count_ind + 1;
end
% Save the CPU time for each optimization
save(fullfile(pwd,'elapsed_time.mat'),'start','-mat');