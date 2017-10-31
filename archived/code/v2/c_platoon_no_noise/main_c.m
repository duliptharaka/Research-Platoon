 % Clear the workspace
clc;
clear all;

% seeds the random number generator based on the current time. 
% Thus, rand, randi, and randn produce a different sequence of numbers after each time you call rng.
rng(1);

% Number of total vehicles in the platooning system
TotalCar = 9;


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
dt = 1;
step = 699;
v_min = 28*0.44704;
v_max = 60*0.44704;
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
first_car_v = avg_first_car_v;
first_car_a = avg_first_car_a;

count_ind = 1;
for number = 3:2:TotalCar
    display(['Start optimization for models with ' num2str(number) ' cars!']);
    % Initiate matrix and varibles
    output_v = zeros(step+1, number);
    output_s = zeros(step+1, number);
    output_a = zeros(step+1, number);
    output_fuel = zeros(step, number);
    output_cost = zeros(step, number-1, 5);
    % Initiate car position so that
    % all cars are 40 meters (0.0248548 miles) apart
    % all cars start at the same speed as the first car
    start_s = zeros(1,number);
    start_v = zeros(1,number);
    start_a = zeros(1,number);
    % Record the cumulative distance and fuel consumption
    TotalS = zeros(1,number-1);
    TotalF = zeros(1,number-1);
    for i = 1:number
        start_s(i) = 40*(number-1) - (i-1)*40;
        start_v(i) = first_car_v(1);
        start_a(i) = first_car_a(1);
    end
    % Generate position for the first car
    first_car_s = zeros(700,1);
    first_car_s(1) = start_s(1);
    for i=2:(step+1)
        first_car_s(i) = first_car_s(i-1) + first_car_v(i-1)*dt + (first_car_a(i-1)/2)*(dt^2);
    end
    %save(fullfile(pwd,strcat('first_car_s_', num2str(number), '.mat')), 'first_car_s', '-mat');
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
    output_v(1,:) = start_v;
    output_s(1,:) = start_s;
    output_a(1,:) = start_a;      
    
    % Optimization
    options = optimset('Display', 'off','Algorithm','sqp');
    % Apply Kalman filter to step 1 separately----------------------------
    i = 1;
    % Initiate the paramters in Kalman filter
    kf_A = zeros(2*number, 2*number);
    kf_B = zeros(2*number, number);
    kf_C = zeros(4*number-5, 2*number);
    for j = 1:number
        kf_A(2*j-1,2*j-1) = 1;
        kf_A(2*j,2*j) = 1;
        kf_A(2*j-1, 2*j) = dt;
        kf_B(2*j-1, j) = dt^2/2;
        kf_B(2*j,j) = dt;
    end
    for j=1:(number-2)
        kf_C(4*j-3, 2*j+1) = 1;
        kf_C(4*j-2, 2*j+1) = -1;
        kf_C(4*j-2, 2*j-1) = 1;
        kf_C(4*j-1, 2*j+2) = 1;
        kf_C(4*j, 2*j+2) = 1;
    end
    kf_C(4*j+1, 2*number-1) = 1;
    kf_C(4*j+2, 2*number-3) = 1;
    kf_C(4*j+2, 2*number-1) = -1;
    kf_C(4*j+3, 2*number) = 1;
    % % initial state variable
    state_last = zeros(2*number, step);
    control_last = zeros(number, 1);
    state = zeros(2*number, step);
    obs = zeros(4*number-5, step);
    pred = zeros(2*number, step);
    est = zeros(2*number, step);
    P_last = zeros(2*number, 2*number);
    P = zeros(2*number, 2*number);
    K_F = zeros(2*number,4*number-5);
    for j = 1:number
        state_last(2*j-1, 1) = start_s(j);
        state_last(2*j,1) = start_v(j);
        control_last(j) = start_a(j);
    end
    % % calculate state updates (add plant noise)
    state(:,1) = kf_A * state_last(:,1) + kf_B * control_last;
    % % get the observation (add sensor noise)
    obs(:,1) = kf_C * state(:,1);
    % Get the estimated v and s at t = 2
    for j = 1:(number-2)
        TotalS(j) = obs(4*j-3,1) - start_s(j+1);
        v = start_v(j+1);
        a = start_a(j+1);
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        TotalF(j) = integral(f,0,1);                    
    end
    TotalS(j+1) = obs(4*j+1,1) - start_s(j+2);
    v = start_v(j+2);
    a = start_a(j+2);
    f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
    TotalF(j+1) = integral(f,0,1);   
    % Get the estimated v and s at t = 2
    %start_s(1) = start_s(1) + start_v(1)*dt + (first_car_a(2)/2)*(dt^2);
    start_s(1) = first_car_s(2);
    start_v(1) = first_car_v(2); 
    start_a(1) = first_car_a(2);
    for j = 1:(number-2)
        start_s(j+1) = obs(4*j-3,1);
        start_v(j+1) = (obs(4*j-1,1) + obs(4*j,1))/2;
    end
    start_s(j+2) = obs(4*j+1,1);
    start_v(j+2) = obs(4*j+3,1);
    % Optimization for the middle car (2,3,4th car in a 5-car model)
    %x = sym('a',[number,1]); 
    g = @(x) one_step_all(x, start_v, start_s, number, TotalS, TotalF, dt);       
    lb = zeros(number,1);
    ub = zeros(number,1);
    for j = 1:number
        lb(j) = max(a_min,((v_min-start_v(j))/dt));
        ub(j) = min(a_max,((v_max-start_v(j))/dt));
    end
    A = zeros(number-1, number);
    b = zeros(number-1, 1);
    for j = 1:(number-1)
        A(j, j) = -1;
        A(j, j+1) = 1;
        b(j) = 2*(start_s(j)-start_s(j+1)+(start_v(j)-start_v(j+1))*dt)/(dt^2);
    end
    Aeq = [];
    beq = [];
    x0 = zeros(number,1);       
    x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);  
    % retrieve result       
    for j = 2:(number-1)
        [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle([x(j);x(j-1);x(j+1)], start_v(j), start_v(j-1), start_v(j+1), start_s(j-1) - start_s(j), TotalS(j-1), TotalF(j-1), dt);
        output_fuel(i,j) = one_fuel;
        %output_cost(i,j) = all;
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;   
        output_cost(i,j-1,5) = p5;  
    end
    j = j+1;
    [p1,p2,p3,p4,one_fuel, all] = cost_value_last([x(j);x(j-1)], start_v(j), start_v(j-1), start_s(j-1) - start_s(j), TotalS(j-1), TotalF(j-1), dt);
    output_fuel(i,j) = one_fuel;
    %output_cost(i,j) = all;        
    output_cost(i,j-1,1) = p1;
    output_cost(i,j-1,2) = p2;        
    output_cost(i,j-1,3) = p3;
    output_cost(i,j-1,4) = p4;         
    % Update parameters for the next step
    for j=2:number
        start_a(j) = x(j);
    end        
    output_v(i+1,:) = start_v;
    output_s(i+1,:) = start_s;
    output_a(i+1,:) = start_a;    
    
    
    % Apply kalman filter from t = 3 --------------------------------------
    for i=2:step
        % Apply kalman filter to get estimated v and s at t = 3...
        for j = 1:number
            state_last(2*j-1, i) = start_s(j);
            state_last(2*j,i) = start_v(j);
            control_last(j) = start_a(j);
        end
        % % calculate state updates (add plant noise)
        state(:,i) = kf_A * state_last(:,i) + kf_B * control_last;
        % % get the observation (add sensor noise)
        obs(:,i) = kf_C * state(:,i);   

        for j = 1:(number-2)
            TotalS(j) = TotalS(j) + obs(4*j-3,i) - obs(4*j-3,i-1);
            v = (obs(4*j-1,i-1) + obs(4*j,i-1))/2;
            a = start_a(j+1);
            f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
            TotalF(j) = TotalF(j) + integral(f,0,1);                    
        end
        TotalS(j+1) = TotalS(j+1) + obs(4*j+1,i) - obs(4*j+1,i-1);
        v = obs(4*j+3,i-1);
        a = start_a(j+2);
        f = @(x) m1*((v+a*x).^2)+m2*(a.^2)+m3*((v+a*x).^2)*a + m4*(v+a*x)*(a.^2)+m5*(v+a*x)*a+m6*(v+a*x)+ m7*a+m8;
        TotalF(j+1) = TotalF(j+1) + integral(f,0,1); 

        % Get the estimated v and s at t = 4
        %start_s(1) = start_s(1) + start_v(1)*dt + (first_car_a(i+1)/2)*(dt^2);
        start_s(1) = first_car_s(i+1);
        start_v(1) = first_car_v(i+1); 
        start_a(1) = first_car_a(i+1);       
        for j = 1:(number-2)
            start_s(j+1) = obs(4*j-3,i);
            start_v(j+1) = (obs(4*j-1,i) + obs(4*j,i))/2;
        end
        start_s(j+2) = obs(4*j+1,i);
        start_v(j+2) = obs(4*j+3,i); 

        % Optimization for the middle car (2,3,4th car in a 5-car model)
        %x = sym('a',[number,1]); 
        g = @(x) one_step_all(x, start_v, start_s, number, TotalS, TotalF, dt);       
        lb = zeros(number,1);
        ub = zeros(number,1);
        for j = 1:number
            lb(j) = max(a_min,((v_min-start_v(j))/dt));
            ub(j) = min(a_max,((v_max-start_v(j))/dt));
        end
        A = zeros(number-1, number);
        b = zeros(number-1, 1);
        for j = 1:(number-1)
            A(j, j) = -1;
            A(j, j+1) = 1;
            b(j) = 2*(start_s(j)-start_s(j+1)+(start_v(j)-start_v(j+1))*dt)/(dt^2);
        end
        Aeq = [];
        beq = [];
        x0 = zeros(number,1);       
        x = fmincon(g, x0, A, b, Aeq, beq, lb, ub, [], options);  
        % retrieve result       
        for j = 2:(number-1)
            [p1,p2,p3,p4,p5,one_fuel, all] = cost_value_middle([x(j);x(j-1);x(j+1)], start_v(j), start_v(j-1), start_v(j+1), start_s(j-1) - start_s(j), TotalS(j-1), TotalF(j-1), dt);
            %disp ([num2str(p1,'%.5f'),'   ',num2str(p2,'%.5f'),'   ',num2str(p3,'%.5f'),'   ',num2str(p4,'%.5f'),'   ',num2str(p5,'%.5f') ,'   ',num2str(one_fuel,'%.5f'), '   ', num2str(all,'%.5f')]);
            output_fuel(i,j) = one_fuel;
            %output_cost(i,j) = all;
            output_cost(i,j-1,1) = p1;
            output_cost(i,j-1,2) = p2;        
            output_cost(i,j-1,3) = p3;
            output_cost(i,j-1,4) = p4;   
            output_cost(i,j-1,5) = p5;  
        end
        j = j+1;
        [p1,p2,p3,p4,one_fuel, all] = cost_value_last([x(j);x(j-1)], start_v(j), start_v(j-1), start_s(j-1) - start_s(j), TotalS(j-1), TotalF(j-1), dt);
        output_fuel(i,j) = one_fuel;
        %output_cost(i,j) = all;        
        output_cost(i,j-1,1) = p1;
        output_cost(i,j-1,2) = p2;        
        output_cost(i,j-1,3) = p3;
        output_cost(i,j-1,4) = p4;         
        % Update parameters for the next step
        for j=2:number
            start_a(j) = x(j);
        end        
        output_v(i+1,:) = start_v;
        output_s(i+1,:) = start_s;
        output_a(i+1,:) = start_a; 
    end

    [output_s, output_v, output_a, output_fuel] = change_unit(output_s, output_v, output_a, output_fuel, dt);
    save(fullfile(strcat(pwd,'/result'),strcat('output_v_', num2str(number), '.mat')), 'output_v', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_a_', num2str(number), '.mat')), 'output_a', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_s_', num2str(number), '.mat')), 'output_s', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_fuel_', num2str(number), '.mat')), 'output_fuel', '-mat');
    save(fullfile(strcat(pwd,'/result'),strcat('output_cost_', num2str(number), '.mat')), 'output_cost', '-mat');
    start(count_ind) = cputime - start(count_ind);
    count_ind = count_ind + 1;
end
% Save the CPU time for each optimization
save(fullfile(pwd,'elapsed_time.mat'),'start','-mat');

