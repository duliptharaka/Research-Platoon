function [ output ] = gen_plant_noise(num)
%GEN_PLANT_NOISE generates 6/5 white noise as the plant noise
% Note that SD1 < SD2
const = 10^(-4);
SD1 = 10*const;
SD2 = 15*const;
output = zeros(num, 1);
for i = 1:num/2
    output(2*i-1, 1) = normrnd(0,SD1);
    output(2*i, 1) = normrnd(0,SD2);    
end
end

