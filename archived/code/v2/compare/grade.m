function [ output ] = grade( s )
%GRADE Summary of this function goes here
%   Detailed explanation goes here
% Elevation expression
%output = 2.62*(10^(-17))*(s^6) - 2.10 * (10^(-13))*(s^5) + 6.26*(10^(-10))*(s^4) -8.27*(10^(-7))*(s^3) + 4.61*(10^(-4))*(s^2) -6.46*(10^(-2))*s + 7.05857862;
output = atan(2.62*6*(10^(-17))*(s^5) - 2.10 * 5*(10^(-13))*(s^4) + 6.26*4*(10^(-10))*(s^3) -8.27*3*(10^(-7))*(s^2) + 4.61*2*(10^(-4))*(s) -6.46*(10^(-2)));
end

