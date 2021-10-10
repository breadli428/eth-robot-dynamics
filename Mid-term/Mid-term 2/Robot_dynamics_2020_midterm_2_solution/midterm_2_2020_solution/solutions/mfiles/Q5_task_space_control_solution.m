function [tau] = Q5_task_space_control_solution( params, gc, I_r_ICd, I_v_Cd, I_a_Cd)
% Task-space inverse dynamics controller tracking a desired end-effector motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - I_r_ICd   : the desired position (3x1) of the camera w.r.t. the inertial frame expressed in the inertial frame.
%   - I_v_Cd    : the desired linear velocity (3x1) of the camera in the inertial frame.
%   - I_a_Cd    : the desired linear acceleration (3x1) of the camera in the inertial frame.
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_Q5; % P gain matrix for camera position (3x3 diagonal matrix)
kd = params.kd_Q5; % D gain matrix for camera velocity  (3x3 diagonal matrix)

% Find jacobians, positions and orientation based on the current
I_Jp_C = I_Jp_C_fun(q); % Positional Jacobian of end effector
I_Jr_C = I_Jr_C_fun(q); % Rotational Jacobian of end effector
I_dJp_C = I_dJp_C_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_dJr_C = I_dJr_C_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)

% Geometrical Jacobian
I_J_C = [I_Jp_C; I_Jr_C];
I_dJ_C = [I_dJp_C; I_dJr_C];

T_IC = T_IC_fun(q); % Homogeneous transformation from frame C to frame I
I_r_IC = T_IC(1:3, 4);

%% Project the joint-space dynamics to the task space
% Only use positional Jacobian
J_task = I_J_C(1:3, :);
dJ_task = I_dJ_C(1:3, :);
JMinv = J_task / M;
lambda = inv(JMinv * J_task'); % <-- Can also use regularized inverse here.
mu = lambda * (JMinv * b - dJ_task * dq);
p =  lambda * JMinv * g;

dw_e = I_a_Cd + kp * (I_r_ICd - I_r_IC) + kd * (I_v_Cd - I_Jp_C * dq);

%% Compute torque
tau = J_task' * (lambda * dw_e + mu + p );

end
