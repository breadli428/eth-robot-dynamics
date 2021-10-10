% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.8;
params.l1 = 0.1;
params.l2 = 0.5;
params.l31 = 0.2;
params.l32 = 0.3;
params.l4 = 0.4;
params.l5 = 0.1;

params.theta = pi/3;

% parameters to be used inside inverseKinematics.m
params.max_it = 100;       % maximum number of iterations
params.lambda = 0.001;     % Damping factor
params.alpha = 0.5;        % Update rate


% Initialize a random vector of joint positions.
q = rand(3,1);