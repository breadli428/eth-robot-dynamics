function J_3C_3 = point3ToCameraGeometricJacobian(q, params)
  % Inputs:
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  % Output:
  %     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
  %     frame {3}
  
  theta = params.theta;
  l31 = params.l31;
  l4 = params.l4;
  l5 = params.l5;

  % Implement your solution here...
  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
  
  % compute rotation matrix from camera frame to frame 3
  C_3C = [cos(theta-pi/2), -sin(theta-pi/2), 0;
          sin(theta-pi/2), cos(theta-pi/2), 0;
          0, 0, 1];
  % position vector from 3 to C in frame 3
  p_3C_3 = [l31; 0; 0] + C_3C*[l5; l4; 0];
  
  % compute rotation matrix from 3 to 1
  C_13 = [cos(q2+q3), -sin(q2+q3), 0;
          sin(q2+q3), cos(q2+q3), 0;
          0, 0, 1];
  
  % compute rotation axes
  n1_3 = C_13'*[0; 1; 0];
  n2_3 = [0; 0; 1];
  n3_3 = [0; 0; 1];
  
  % write geometric jacobian
  J_3C_3 = zeros(6,3);
  J_3C_3(1:3,:) = [cross(n1_3, p_3C_3), cross(n2_3, p_3C_3), cross(n3_3, p_3C_3)];
  J_3C_3(4:6,:) = zeros(3,3);

end