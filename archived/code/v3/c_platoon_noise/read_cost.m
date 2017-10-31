function [ output ] = read_cost( cost, car_number, cost_ind)
%READ_COST Summary of this function goes here
%   Detailed explanation goes here
output = zeros(699, 1);
for i = 1:699
    for j = 1:5
        output(i) = cost(i,car_number,cost_ind);
    end
end

end

