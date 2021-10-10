function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  r01 = [0 0 0.145]';
  S01 = [cos(q(1)) -sin(q(1)) 0; sin(q(1)) cos(q(1)) 0; 0 0 1];
  T01 = [S01 r01; zeros(1,3) 1];
end