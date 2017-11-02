function fc = fuel_consumption_drag (a, v, d_i, delta_d)
  C_d = 0.32; % Experimental Drag coefficient
  rho_a = 1.184; % Density of air at 25 deg C
  A_v = 2.5; % Cross-sectional area of the vehicle
  E_g = 1.3e8; % Energy in a gallon of gasoline
  eta = .25; % Efficiency of the vehicle

    fc = (v^2*rho_a*C_d*A_v*drag_reduction_ratio(d_i)*delta_d)/(2*E_g*eta);
  if fc<0
    fc = 0;
  end
return