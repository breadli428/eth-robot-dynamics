function p_IC_I = jointToCameraPosition_solution(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters
  
  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  theta = params.theta;
  l4 = params.l4;
  l5 = params.l5;
  
  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % compute T_I0
  p_I0_I =  [0; 0; l0];
  C_I0 = [0 0 1;
          1 0 0;
          0 1 0];
  T_I0 = [C_I0 p_I0_I;
          0 0 0 1];

  % compute T_01
  p_01_0 = [0; l1; 0];
  C_01 = [cos(q1) 0 sin(q1);
          0 1 0;
          -sin(q1) 0 cos(q1)];
  T_01 = [C_01 p_01_0;
          0 0 0 1];
  % compute T_12
  p_12_1 = [0; 0; 0];
  C_12 = [cos(q2) -sin(q2) 0;
          sin(q2) cos(q2) 0;
          0 0 1];
  T_12 = [C_12 p_12_1;
          0 0 0 1];
  % compute T_23
  p_23_2 = [l2; 0; 0];
  C_23 = [cos(q3) -sin(q3) 0;
          sin(q3) cos(q3) 0;
          0 0 1];
  T_23 = [C_23 p_23_2;
          0 0 0 1];
  % compute T_3C
  C_3C = [cos(theta-pi/2), -sin(theta-pi/2), 0;
          sin(theta-pi/2), cos(theta-pi/2), 0;
          0, 0, 1];
  p_3C_3 = [l31; 0; 0] + C_3C*[l5; l4; 0];
  
  T_3C = [C_3C p_3C_3;
          0 0 0 1];
      
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % concatenate transformations
  T_I1 = T_I0 * T_01;
  T_I2 = T_I1 * T_12;
  T_I3 = T_I2 * T_23;
  T_IC = T_I3 * T_3C;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % extract camera position
  p_IC_I = T_IC(1:3,4);
  
  
end