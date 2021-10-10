function T23 = jointToTransform23(q, l2)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 3. T_23
  if (length(q)>1)
  	q = q(3);
  end
  T23 = [ cos(q), -sin(q), 0,     l2;
           sin(q), cos(q),      0,     0;
         0, 0, 1, 0;
               0, 0,      0,     1];
end
