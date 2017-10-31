function [ output ] = est_noise(Y, X)
%EST_NOISE Summary: Find the diagnal matrix from noise
[m,n] = size(Y);
diff = zeros(m,n);
output = zeros(m,m);

for j = 1:m
    for i = n        
    diff(m,n) = Y(m,n) - X(m,n);
    end
    output(j,j) = var(diff(j,:));
end

end

