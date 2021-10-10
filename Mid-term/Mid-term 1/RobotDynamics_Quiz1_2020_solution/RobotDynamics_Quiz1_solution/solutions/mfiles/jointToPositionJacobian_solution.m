function I_Jp = jointToPositionJacobian_solution(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Implement your solution here.
  I_Jp = [];
  
  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  l4 = params.l4;
  l5 = params.l5;
  theta = params.theta;
  
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
  % extract relative positions
  p_I1_I = T_I1(1:3,4);
  p_I2_I = T_I2(1:3,4);
  p_I3_I = T_I3(1:3,4);
  p_IC_I = T_IC(1:3,4);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % write joint rotation axes
  n1_1 = [0; 1; 0];
  n2_2 = [0; 0; 1];
  n3_3 = [0; 0; 1];
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % extract rotation matrices
  R_I1 = T_I1(1:3, 1:3);
  R_I2 = T_I2(1:3, 1:3);
  R_I3 = T_I3(1:3, 1:3);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % rotate rotation axes
  n1_I = R_I1 * n1_1;
  n2_I = R_I2 * n2_2;
  n3_I = R_I3 * n3_3;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % build jacobian
  I_Jp = [cross(n1_I, p_IC_I-p_I1_I), cross(n2_I, p_IC_I-p_I2_I), cross(n3_I, p_IC_I-p_I3_I)];

end