function [ output_phi ] = phi( s )
%PHI Summary of this function goes here
%   Detailed explanation goes here
%output_phi = 0.40813133 * (1 - exp(-0.07683938 * s)) + 0.38470053;
output_phi = 0.7 * (1 - exp(-0.03 * s)) + 0.3;
end

