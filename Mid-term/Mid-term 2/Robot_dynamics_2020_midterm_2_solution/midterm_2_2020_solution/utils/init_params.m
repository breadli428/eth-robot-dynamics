function params = init_params()
% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.2;
params.l1 = 0.5;
params.l2 = 0.6;
params.l31 = 0.2;
params.l32 = 0.3;
params.l4 = 0.4;
params.l5 = 0.1;

params.theta = pi/3;

%% Gains
params.kp_j = diag([10.0 10.0 5.0]);
params.kd_j =  diag([2.0 2.0 1.0]);

params.kp_Q5 = diag([50 50 50]);
params.kd_Q5 = 2.0*sqrt(params.kp_Q5);

params.tau_max = 50.0;

%% Link properties
R = 0.01; % link radius
params.m = cell(3,1);
params.k_r_ks = cell(3,1); 
params.k_I_s = cell(3,1);

% link 1
params.m{1} = 4.5;
params.k_r_ks{1} = [0.0; 0.5 * params.l1; 0.0];
params.k_I_s{1} = diag([params.m{1}*params.l1^2/12.0, ...
                        params.m{1}*R^2/2.0, ...
                        params.m{1}*params.l1^2/12.0]);
% link2 
params.m{2} = 1.0;
params.k_r_ks{2} = [0.5 * params.l2; 0.0; 0.0];
params.k_I_s{2} = diag([params.m{2}*R^2/2.0, ...
                        params.m{2}*params.l2^2/12.0, ...
                        params.m{2}*params.l2^2/12.0]);
                          
% link3 
params.m{3} = 1.0;
params.k_r_ks{3} = [0.5 * (params.l31 + params.l32); 0.0; 0.0];
params.k_I_s{3} = diag([params.m{3}*R^2/2.0, ...
                        params.m{3}*(params.l31 + params.l32)^2/12.0, ...
                        params.m{3}*(params.l31 + params.l32)^2/12.0]);

% Object
params.m_o = 1.0;
params.E_I_o = [0.01 0 0;
                    0 0.02 0;
                    0 0 0.03];
                    
%% Gravity
params.I_g_acc = [0; 0; -9.81];

%% Simulation params
% Sampling time used for discrete integration steps, and for visualization
params.simulation_dt = 0.0001;
params.control_dt = 0.02;

end
