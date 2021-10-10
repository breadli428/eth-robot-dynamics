function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  r23 = [0 0 0.270]';
  S23 = [cos(q(3)) 0 sin(q(3)); 0 1 0; -sin(q(3)) 0 cos(q(3))];
  T23 = [S23 r23; zeros(1,3) 1];
end
