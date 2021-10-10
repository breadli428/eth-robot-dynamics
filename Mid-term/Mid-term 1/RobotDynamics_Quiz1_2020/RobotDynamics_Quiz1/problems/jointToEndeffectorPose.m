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
    
  % Implement your solution here ...
  TI0 = getTransformI0(l0);
  T01 = jointToTransform01(q, l1);
  T12 = jointToTransform12(q);
  T23 = jointToTransform23(q, l2);
  T3E = jointToTransform3E(l31, l32);
  T_IE = TI0 * T01 * T12 * T23 * T3E; 
  
end