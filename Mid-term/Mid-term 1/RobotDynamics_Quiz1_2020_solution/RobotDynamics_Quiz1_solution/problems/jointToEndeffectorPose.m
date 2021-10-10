function [ T_IE ] = jointToEndeffectorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  l32 = params.l32;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
      
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
  % compute T_3E
  C_3E = eye(3,3);
  p_3E_3 = [l31 + l32; 0; 0];
  
  T_3E = [C_3E p_3E_3;
          0 0 0 1];
  % concatenate transformations
  T_IE = T_I0 * T_01 * T_12 * T_23 * T_3E;
  
end