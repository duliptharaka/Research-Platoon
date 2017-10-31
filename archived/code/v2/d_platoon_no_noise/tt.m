%w2 = r * exp(-alpha*d_front_next);
%R_e = R + h_d*v_next - d_front_next;
%w6 = r * (-alpha*(2*R-d_front_next));
%R_b = R + h_d*v_next - (2*R-d_front_next);
v = 50;
n = 100;
d = zeros(1,n);
result = zeros(1,n);
r = 3;
alpha = 0.2;
R = 30;
h_d = 3;
for i = 1:n
    d(i) = i;
    result(i) = r * exp(-alpha*d(i))*(R + h_d*v - d(i))^2 + r/(10^2) * (-alpha*(2*R-d(i)))*(R + h_d*v - (2*R-d(i)))^2;
end
%ylims([
plot(d,result);


