% Clear the workspace
clc;
clear all;
diary('output');

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

% Initilize paramters
delta_t = 1;
step = 700-1;
v_min = 28*0.44704;
v_max = 60*0.44704;
%a_min = -12000/8052.9706513958;
%a_max = 8000/8052.9706513958;
a_min = -1;
a_max = 1;

% Coefficients of the fuel consumption
m1 = 1.442*(10^(-6));
m2 = -5.67*(10^(-6));
m3 = 1.166*(10^(-6));
m4 = 39.269*(10^(-6));
m5 = 58.284*(10^(-6));
m6 = 19.279*(10^(-6));
m7 = 82.426*(10^(-6));
m8 = 185.36*(10^(-6));

% Start the main loop
% Goals: Calculate ith car's acceleration at time t
% Assumptions:
%(1) (i-1)th car shares its acceleration at time (t-1); 
%(2) (i+1)th car shares its acceleration at time (t-1); 
%(3) ith car measures its distance to (i-1)th car;
%(4) ith car measures its distance to (i+1)th car;
%(5) Only consider one step to the future during optimization

% For the first car velocity
load(fullfile(pwd,'avg_first_car_a.mat'));
load(fullfile(pwd,'avg_first_car_v.mat'));
first_car_v = avg_first_car_v;
first_car_a = avg_first_car_a;

% Main loop for platooning with 3,5,7,9 cars
count_ind = 1;
for it = 3:2:9
%for it = 3:3
    number = it;
    display(['Start optimization for models with ' num2str(number) ' cars!']);
    % Initiate matrix and varibles
    output_v_behind = zeros(step+1, number);
    output_s_behind = zeros(step+1, number);
    output_a_behind = zeros(step, number);
    output_fuel = zeros(step, number);
    output_cost = zeros(step, number-1, 5);
    % Initiate car position so that
    % all cars are 40 meters (0.0248548 miles) apart
    % all cars start at the same speed as the first car
    start_s = zeros(1,number);
    start_v = zeros(1,number);
    start_a = zeros(1,number);
    for i = 1:number
        start_s(i) = 40*(number-1) - (i-1)*40;
        start_v(i) = first_car_v(1);
        start_a(i) = first_car_a(1);
    end
    output_v_behind(1,:) = start_v;
    output_s_behind(1,:) = start_s;
    % Generate position for the first car
    first_car_s = zeros(700,1);
    first_car_s(1) = start_s(1);
    for i=2:(step+1)
        first_car_s(i) = first_car_s(i-1) + first_car_v(i-1)*delta_t + (first_car_a(i-1)/2)*(delta_t^2);
    end
    save(fullfile(pwd,strcat('first_car_s_', num2str(number), '.mat')), 'first_car_s', '-mat');
    % Calculate fuel consumption for the first car
    first_car_fuel = zeros(step, 1);
    for j = 1:step
        v = first_car_v(j);
        a = first_car_a(j);
        %first_car_fuel(j,1) = fuel_first(first_car_v(j), first_car_s(j), first_car_a(j));
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        first_car_fuel(j,1) = integral(f,0,1);
    end
    output_fuel(:,1) = first_car_fuel;
    % Optimization
    options = optimset('Display', 'off','Algorithm','sqp');
    %options = optimset('Algorithm','sqp');
    for i=1:step
        % Next step parameter
        start_s_new = zeros(1, number);
        start_v_new = zeros(1, number);
        start_a_new = zeros(1, number);
        start_s_new(1) = start_s(1) + start_v(1)*delta_t + (first_car_a(i)/2)*(delta_t^2);
        start_v_new(1) = first_car_v(i+1); 
        start_a_new(1) = first_car_a(i);     
        %disp('step')
        %disp(i)
        % Optimization for the middle car (2,3,4th car in a 5-car model)
        for j = 2:(number-1)
            %x = sym('a',[3,1]);
            v_front = start_v(j-1);
            v_behind = start_v(j+1);
            d_front = start_s(j-1) - start_s(j);
            d_behind = start_s(j) - start_s(j+1);
            v = start_v(j);
            s = start_s(j);
            g = @(x)one_step_middle(x, v, v_front, v_behind, d_front, delta_t);
            % optimization parameter
            lb = [max(a_min,(v_min-v)/delta_t), max(a_min,(v_min-v_front)/delta_t), max(a_min,(v_min-v_behind)/delta_t)];
            ub = [min(a_max,(v_max-v)/delta_t), min(a_max,(v_max-v_front)/delta_t),min(a_max,(v_max-v_behind)/delta_t)];
            A = [1,-1,0; -1,0,1];
            b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*delta_t)/(delta_t^2);2*(start_s(j)-start_s(j+1)+(v-v_behind)*delta_t)/(delta_t^2)];
            Aeq = [];
            beq = [];
            x0 = [0;0;0];
            x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);
            % Display each element of cost function
            [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle(x, v, v_front, v_behind, d_front, delta_t);
            %disp('middle')           
            %disp ([num2str(p1,'%.5f'),'   ',num2str(p2,'%.5f'),'   ',num2str(p3,'%.5f'),'   ',num2str(p4,'%.5f'),'   ',num2str(p5,'%.5f') ,'   ',num2str(one_fuel,'%.5f'), '   ', num2str(all,'%.5f')]);
            % retrieve result
            % convert to real acceleration
            %x(1) = real_a(x(1), s, v, d_front);
            output_fuel(i,j) = one_fuel;
            output_cost(i,j-1,1) = p1;
            output_cost(i,j-1,2) = p2;        
            output_cost(i,j-1,3) = p3;
            output_cost(i,j-1,4) = p4;   
            output_cost(i,j-1,5) = p5;   
            start_s_new(j) = s + v*delta_t + (x(1)/2)*(delta_t^2);
            start_v_new(j) = v + x(1)*delta_t;
            start_a_new(j) = x(1);
        end
        % Optimization for the last car
        j = j+1;
        %y = sym('a',[3,1]);
        v_front = start_v(j-1);
        d_front = start_s(j-1) - start_s(j);
        v = start_v(j);
        s = start_s(j);
        h = @(x)one_step_last(x, v, v_front, d_front, delta_t);
        % optimization parameter
        lb = [max(a_min,((v_min-v)/delta_t)), max(a_min,((v_min-v_front)/delta_t))];
        ub = [min(a_max,((v_max-v)/delta_t)), min(a_max,((v_max-v_front)/delta_t))];
        A = [1,-1];
        b = [2*(start_s(j-1)-start_s(j)+(v_front-v)*delta_t)/(delta_t^2)];
        Aeq = [];
        beq = [];
        x0 = [0;0];
        x = fmincon(h, x0, A, b, Aeq, beq, lb, ub, [], options);
        % Display each element of cost function
        [p1,p2,p3,p4,one_fuel, all] = cost_value_last(x, v, v_front, d_front, delta_t);
        %disp('last')
        %disp ([num2str(p1,'%.5f'),'   ', num2str(p2,'%.5f'),'   ',num2str(p3,'%.5f'),'   ',num2str(p4,'%.5f'),'   ',num2str(one_fuel,'%.5f'), '   ', num2str(all,'%.5f')]);
        % retrieve result
        % convert to real acceleration
        %x(1) = real_a(x(1), s, v, d_front);
        output_fuel(i,j) = one_fuel;
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;
        start_s_new(j) = s + v*delta_t + (x(1)/2)*(delta_t^2);
        start_v_new(j) = v + x(1)*delta_t;
        start_a_new(j) = x(1);
        % update parameter
        start_s = start_s_new;
        start_v = start_v_new;
        output_v_behind(i+1,:) = start_v;
        output_s_behind(i+1,:) = start_s;
        output_a_behind(i,:) = start_a_new;
    end

    [output_s_behind, output_v_behind, output_a_behind, output_fuel] = change_unit(output_s_behind, output_v_behind, output_a_behind, output_fuel, delta_t);
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


