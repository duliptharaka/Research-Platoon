function fc = fuel_consumption (a, v) 
  w1 = 0.442e-6;
  w2 = -5.67e-6;
  w3 = 1.166e-6;
  w4 = 39.269e-6;
  w5 = 58.284e-6;
  w6 = 19.279e-6;
  w7 = 82.426e-6;
  w8 = 185.360e-6;
  
  if a > 0
    fc = w1*v^2 + w2*a^2 + w3*v^2*a + w4*v*a^2 + w5*v*a + w6*v + w7*a + w8;
  else
    fc = 0;
  end
return