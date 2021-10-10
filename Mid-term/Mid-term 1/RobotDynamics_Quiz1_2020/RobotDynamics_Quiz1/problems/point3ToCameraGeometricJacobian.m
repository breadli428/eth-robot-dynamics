function [ J_3C_3 ] = point3ToCameraGeometricJacobian(q, params)
  % Inputs:
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  % Output:
  %     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
  %     frame {3}
  
  theta = params.theta;
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  l32 = params.l32;
  l4 = params.l4;
  l5 = params.l5;

  % Implement your solution here...
  TI0 = getTransformI0(l0);
  T01 = jointToTransform01(q, l1);
  T12 = jointToTransform12(q);
  T23 = jointToTransform23(q, l2);
  T3E = jointToTransform3E(l31, l32);
  T_IE = TI0 * T01 * T12 * T23 * T3E; 
  T3C = jointToTransform3C(l31, l4, l5, theta);
  r_3_3C = T3C(1:3,4);
  
  TI1 = TI0 * T01;
  TI2 = TI1 * T12;
  TI3 = TI2 * T23;
  TIE = TI3 * T3E;
  TIC = TI3 * T3C;
  
  T03 = T01 * T12 * T23;
  C03 = T03(1:3, 1:3);
  T13 = T12 * T23;
  C13 = T13(1:3, 1:3);
  C23 = T23(1:3, 1:3);
  
  C30 = C03';
  C31 = C13';
  C32 = C23';
  
  n1 = [0 1 0]';
  n2 = [0 0 1]';
  n3 = [0 0 1]';
  
    J_P = [   cross(C30*n1, r_3_3C) ...
            cross(C31*n2, r_3_3C) ...
            cross(C32*n3, r_3_3C) ...
             ];
   J_R = [   C30*n1 ...
            C31*n2 ...
            C32*n3 ...
        ];
  J_3C_3 = [J_P;J_R];

end