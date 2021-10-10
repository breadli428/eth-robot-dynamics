function TI0 = getTransformI0(l0)
  % Input: void
  % Output: homogeneous transformation Matrix from the inertial frame I to frame 0. T_I0
  CI0 = [[0 1 0]' [0 0 1]' [1 0 0]'];
  r_I_I0 = [0 0 l0]';
  TI0 = [CI0 r_I_I0; zeros(1,3) 1];
end
