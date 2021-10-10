function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  r45 = [0.168 0 0]';
  S45 = [cos(q(5)) 0 sin(q(5)); 0 1 0; -sin(q(5)) 0 cos(q(5))];
  T45 = [S45 r45; zeros(1,3) 1];
end

