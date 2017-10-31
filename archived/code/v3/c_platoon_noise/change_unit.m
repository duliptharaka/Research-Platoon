function [ s,v,a,fuel ] = change_unit( s,v,a,fuel, delta_t )
% Change metric to American unit
% Convert unit
% 1 m = 0.000621371 mile
% 1 miles/h2 = 0.00012417777777778 m/s2
% 1 mph = 0.447 04 m/s
% 1 mL/s = 0.951019 gallon/hour
% 1L = 0.264172 gallon

% Convert s from meter to mile
% Convert v from meter/second to mile/hour (mph)
[m,n] = size(s);
for i = 1:m
    for j=1:n    
        s(i,j) = s(i,j) * 0.000621371;
        v(i,j) = v(i,j)/0.44704;
    end
end

% [m,n] = size(a);
% for i = 1:m
%     for j=1:n    
%         a(i,j) = a(i,j)/0.00012417777777778;
%     end
% end

% Convert fuel from mL/s to gallon/hour
% [m,n] = size(fuel);
% for i = 1:m
%     for j=1:n   
%         fuel(i,j) = fuel(i,j)*0.951019;
%     end
% end

% Convert fuel from kg/s to mile/gallon
% gasoline density is 0.75 Kg/L
[m,n] = size(fuel);
for i = 1:m
    for j=1:n    
        fuel(i,j) = (s(i+1,j)- s(i,j))/((fuel(i,j)*1000/0.75)*delta_t/1000*0.264172);
    end
end
