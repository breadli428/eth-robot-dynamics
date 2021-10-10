function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  q_AB_hat = q_AB(2:4);
  q_AB_hat_x = [0 -q_AB(4) q_AB(3); q_AB(4) 0 -q_AB(2); -q_AB(3) q_AB(2) 0];
  Ml_AB = [q_AB(1) -q_AB_hat'; q_AB_hat q_AB(1) * eye(3) + q_AB_hat_x];
  q_AC = Ml_AB * q_BC;
end

