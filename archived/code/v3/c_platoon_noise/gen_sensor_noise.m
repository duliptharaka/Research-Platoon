function [ output ] = gen_sensor_noise(num)
%GEN_PLANT_NOISE generates 6 white noise as the plant noise
% Note that 
const = 10^(-2);
SD1 = 10*const; % for position
SD2 = 15*const; % for direct velocity
SD3 = 5*const; % for distance
SD4 = 20*const; % for indirect velocity
output = zeros(num, 1);
for i = 1:((num+5)/4-2)
    output(4*i-3,1) = normrnd(0,SD1);
    output(4*i-2,1) = normrnd(0,SD3); 
    output(4*i-1,1) = normrnd(0,SD2);
    output(4*i,1) = normrnd(0,SD4);
end
output(4*i+1,1) = normrnd(0,SD1);
output(4*i+2,1) = normrnd(0,SD3);
output(4*i+3,1) = normrnd(0,SD2);
end

