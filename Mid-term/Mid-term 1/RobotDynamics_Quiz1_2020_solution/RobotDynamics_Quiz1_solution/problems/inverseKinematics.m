function [ q ] = inverseKinematics(I_r_IC_des, q_0, tol, params)
% Input: 
% I_r_IC_des: 3x1 desired position of the point C
% q_0: 3x1 initial guess for joint angles
% tol: 1x1 tolerance to use as termination criterion
%      The tolerance should be used as:
%      norm(I_r_IC_des - I_r_IC) < tol
% params: a struct of parameters
% Output:
% q: a vector of joint angles q (3x1) which achieves the desired
%    task-space position

% 0. Setup
max_it = params.max_it;       % Set the maximum number of iterations. 
lambda = params.lambda;       % Damping factor
alpha = params.alpha;         % Update rate

% 1. start configuration
q = q_0;

% implement your solution here ...
it = 0;
% 2. iterate until terminating condition
while (it==0 || (norm(dr)>tol && it < max_it))
    % 3. evaluate Jacobian for current q
    I_J = jointToPositionJacobian_solution(q, params);
    
    % 4. Update the psuedo-inverse
    I_J_pinv = pseudoInverseMat_solution(I_J, lambda);
    
    % 5. Find the camera configuration error vector
    % position error
    I_r_IC = jointToCameraPosition_solution(q, params);
    dr = I_r_IC_des - I_r_IC; 
    
    % 6. Update the generalized coordinates
    q =  q + alpha*I_J_pinv*dr;
    
    it = it+1;
end


end