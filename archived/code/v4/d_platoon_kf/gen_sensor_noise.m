function [ output ] = gen_sensor_noise(type)
%GEN_PLANT_NOISE generates 6 white noise as the plant noise
% Note that SD3 < SD1=SD4 < SD2 = SD6 = SD8 < SD5 = SD7
const = 10^(-2);
SD1 = 10*const;
SD2 = 15*const;
SD3 = 5*const;
SD4 = 10*const;
SD5 = 20*const;
SD6 = 15*const;
SD7 = 20*const;
SD8 = 15*const;
if strcmp(type, 'middle')
    output = [normrnd(0,SD1);normrnd(0,SD2);normrnd(0,SD3);normrnd(0,SD4);normrnd(0,SD5);normrnd(0,SD6);normrnd(0,SD7);normrnd(0,SD8)];
end
if strcmp(type, 'last')
    output = [normrnd(0,SD1);normrnd(0,SD2);normrnd(0,SD3);normrnd(0,SD4);normrnd(0,SD5);normrnd(0,SD6)];
end
if strcmp(type, 'get_mat_middle')
    output = [SD1^2 0 0 0 0 0 0 0; 0 SD2^2 0 0 0 0 0 0; 0 0 SD3^2 0 0 0 0 0; 0 0 0 SD4^2 0 0 0 0; 0 0 0 0 SD5^2 0 0 0; 0 0 0 0 0 SD6^2 0 0;0 0 0 0 0 0 SD7^2 0;0 0 0 0 0 0 0 SD8^2];
end
if strcmp(type, 'get_mat_last')
    output = [SD1^2 0 0 0 0 0; 0 SD2^2 0 0 0 0; 0 0 SD3^2 0 0 0; 0 0 0 SD4^2 0 0; 0 0 0 0 SD5^2 0; 0 0 0 0 0 SD6^2];
end
end

