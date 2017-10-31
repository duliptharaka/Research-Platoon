 % Compare the fuel Mean/Variance for Decentralized and Centralized control
clc;
clear all;

% parameters
TotalCar = 9;
TotalCount = (TotalCar-1)/2;
number = zeros(1,TotalCount);
for i = 1:TotalCount
    number(i) = 2*i + 1;
end
step = 700;
folder_d = 'd_platoon_no_noise';
folder_c = 'c_platoon_no_noise';

fuel_d = zeros(TotalCount,1);
fuel_c = zeros(TotalCount,1);
fuel_old = zeros(TotalCount,1);
load(fullfile(strcat(pwd,'/compare'),'first_car_v.mat'),'first_car_v','-mat');
load(fullfile(strcat(pwd,'/compare'),'first_car_a.mat'),'first_car_a','-mat');
delta_t = 1;

% calculate mpg mean
table_c = zeros(4,TotalCount);
table_d = zeros(4,TotalCount);
fuel_mean = zeros(2,TotalCount);
fuel_var = zeros(2,TotalCount);
for i = 1:TotalCount
    % For decentralized model
    data_ds = load(fullfile(strcat(pwd, '/',folder_d,'/result/output_s_', num2str(number(i)), '.mat')));
    d_s = data_ds.output_s;
    data_df = load(fullfile(strcat(pwd, '/',folder_d,'/result/output_fuel_', num2str(number(i)), '.mat')));
    d_f = data_df.output_fuel;
    [m,n] = size(d_f);
    gallon = zeros(m,n);
    % Convert instant mpg back to gallon
    for ii = 1:m
        for jj=1:n    
            %fuel(i,j) = (s(i+1,j)- s(i,j))/((fuel(i,j)*1000/0.75)*delta_t/1000*0.264172);
            gallon(ii,jj) = (d_s(ii+1,jj)- d_s(ii,jj))/d_f(ii,jj);
        end
    end
    % % calculate the mpg for each car
    mpg_d = zeros(1,number(i));
    totalg = zeros(1, number(i));
    totals = zeros(1, number(i));
    for j = 1: number(i)
        totalg(j) = sum(gallon(:,j));  % in gallon
        totals(j) = d_s(700,j);   % in mile
        mpg_d(j) = totals(j)/totalg(j);
    end
    fuel_mean(1,i) = sum(mpg_d)/number(i);
    fuel_var(1,i) = var(mpg_d);
    total_mpg = sum(totals)/sum(totalg);
    fuel_d(i) = (total_mpg-mpg_d(1))/mpg_d(1);
    
    % For centralized model
    data_cs = load(fullfile(strcat(pwd, '/',folder_c,'/result/output_s_', num2str(number(i)), '.mat')));
    c_s = data_cs.output_s;
    data_cf = load(fullfile(strcat(pwd, '/',folder_c,'/result/output_fuel_', num2str(number(i)), '.mat')));
    c_f = data_cf.output_fuel;
    [m,n] = size(c_f);
    gallon = zeros(m,n);
    % Convert instant mpg back to gallon
    for ii = 1:m
        for jj=1:n    
            %fuel(i,j) = (s(i+1,j)- s(i,j))/((fuel(i,j)*1000/0.75)*delta_t/1000*0.264172);
            gallon(ii,jj) = (c_s(ii+1,jj)- c_s(ii,jj))/c_f(ii,jj);
        end
    end
    % % calculate the mpg for each car
    mpg_c = zeros(1,number(i));
    totalg = zeros(1, number(i));
    totals = zeros(1, number(i));
    for j = 1: number(i)
        totalg(j) = sum(gallon(:,j));  % in gallon
        totals(j) = c_s(700,j);   % in mile
        mpg_c(j) = totals(j)/totalg(j);
    end
    fuel_mean(2,i) = sum(mpg_c)/number(i);
    fuel_var(2,i) = var(mpg_c);
    total_mpg = sum(totals)/sum(totalg);
    fuel_c(i) = (total_mpg-mpg_c(1))/mpg_c(1);    
end


for i=1:TotalCount
    table_c(1,i) = number(i);
    table_c(2,i) = fuel_mean(2,i);
    table_c(3,i) = fuel_var(2,i);
    table_c(4,i) = fuel_c(i);
    table_d(1,i) = number(i);
    table_d(2,i) = fuel_mean(1,i);
    table_d(3,i) = fuel_var(1,i);
    table_d(4,i) = fuel_d(i);
end






