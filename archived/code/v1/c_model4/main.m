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
first_car_v = zeros(step+1,1);
first_car_a = zeros(step,1);
first_car_v = avg_first_car_v;
first_car_a = avg_first_car_a;

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
        x = sym('a',[number,1]); 
        g = @(x) one_step_all(x, start_v, start_s, number, delta_t);       
        lb = zeros(number,1);
        ub = zeros(number,1);
        for j = 1:number
            lb(j) = max(a_min,((v_min-start_v(j))/delta_t));
            ub(j) = min(a_max,((v_max-start_v(j))/delta_t));
        end
        A = zeros(number-1, number);
        b = zeros(number-1, 1);
        for j = 1:(number-1)
            A(j, j) = -1;
            A(j, j+1) = 1;
            b(j) = 2*(start_s(j)-start_s(j+1)+(start_v(j)-start_v(j+1))*delta_t)/(delta_t^2);
        end
        Aeq = [];
        beq = [];
        x0 = zeros(number,1);       
        x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);  
        % retrieve result       
        for j = 2:(number-1)
            [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle([x(j);x(j-1);x(j+1)], start_v(j), start_v(j-1), start_v(j+1), start_s(j-1) - start_s(j), delta_t);
            output_fuel(i,j) = one_fuel;
            %output_cost(i,j) = all;
            output_cost(i,j-1,1) = p1;
            output_cost(i,j-1,2) = p2;        
            output_cost(i,j-1,3) = p3;
            output_cost(i,j-1,4) = p4;   
            output_cost(i,j-1,5) = p5;  
        end
        j = j+1;
        [p1,p2,p3,p4,one_fuel, all] = cost_value_last([x(j);x(j-1)], start_v(j), start_v(j-1), start_s(j-1) - start_s(j), delta_t);
        output_fuel(i,j) = one_fuel;
        %output_cost(i,j) = all;        
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;         
        % Update parameters for the next step
        for j=2:number
            start_s_new(j) = start_s(j) + start_v(j)*delta_t + (x(j)/2)*(delta_t^2);
            start_v_new(j) = start_v(j) + x(j)*delta_t;
            start_a_new(j) = x(j);
        end        
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


