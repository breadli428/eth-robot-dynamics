function T3C = jointToTransform3C(l31, l4, l5, theta)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 3. T_23

  T3C = [ cos(pi/2 - theta), -sin(pi/2 - theta), 0,     l31 + l4 * cos(theta) + l5 * sin(theta);
           sin(pi/2 - theta), cos(pi/2 - theta),      0,     l4*sin(theta) - l5*cos(theta);
         0, 0, 1, 0;
               0, 0,      0,     1];
end
