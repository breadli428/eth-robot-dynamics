function T3E = jointToTransform3E(l31, l32)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 3. T_23

  T3E = [ 1, 0, 0,     l31 + l32;
           0, 1,      0,     0;
         0, 0, 1, 0;
               0, 0,      0,     1];
end
