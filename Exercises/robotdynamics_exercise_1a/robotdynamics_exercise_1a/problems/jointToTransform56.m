function T56 = jointToTransform56(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  r56 = [0.072 0 0]';
  S56 = [1 0 0; 0 cos(q(6)) -sin(q(6)); 0 sin(q(6)) cos(q(6))];
  T56 = [S56 r56; zeros(1,3) 1];
end
