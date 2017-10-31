function ratio = drag_reduction_ratio (dist)
  ratio = 0.7*(1 - exp(-0.03*dist)) + 0.3;
endfunction