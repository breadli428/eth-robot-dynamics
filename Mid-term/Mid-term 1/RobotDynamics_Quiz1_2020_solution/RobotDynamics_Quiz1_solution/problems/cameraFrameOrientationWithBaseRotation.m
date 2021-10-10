function [ C_IC ] = cameraFrameOrientationWithBaseRotation(qW, qX, qY, qZ, q, params) 
% Input: 
%   qW, qX, qY, qZ: components of the quaternion q_I0 = [qW, qX, qY, qZ],
%                   which describes the orientation from the 0 frame to the
%                   inertial frame
%   q: a 3x1 vector of generalized coordinates
%   params: a struct of parameters
% Output:
%   C_IC: rotation matrix describing the camera frame orientation
%         with respect to the inertial frame when the base orientation is
%         not fixed. 
  
  q_I0 = [qW; qX; qY; qZ];
  
  % implement your solution here ...
  % extract the vector part of the quaternion
  quat_n = q_I0(2:4);
  skew_matrix = [0, -quat_n(3), quat_n(2);
                 quat_n(3), 0, -quat_n(1);
                 -quat_n(2), quat_n(1), 0];
  % convert the quaternion to a rotation matrix
  C_I0 = eye(3,3) + 2*qW*skew_matrix + 2*skew_matrix*skew_matrix;
  
  % get the matrix C_0C
  T_0C = T_0C_solution(q,params);
  C_0C = T_0C(1:3, 1:3);
  
  C_IC = C_I0 * C_0C;
  
end