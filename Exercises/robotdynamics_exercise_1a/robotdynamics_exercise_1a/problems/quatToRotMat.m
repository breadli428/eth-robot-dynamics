function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  q_hat = q(2:4);
  q_hat_x = [0 -q(4) q(3); q(4) 0 -q(2); -q(3) q(2) 0];
  R = (2* q(1)^2 - 1)* eye(3)+ 2 * q(1) * q_hat_x + 2 * q_hat * q_hat';
end
