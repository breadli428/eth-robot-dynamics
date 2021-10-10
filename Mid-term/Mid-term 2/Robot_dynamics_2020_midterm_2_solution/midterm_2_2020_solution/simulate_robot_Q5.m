init_workspace

%% Setup
use_solution = 0; % Use solution (1) or user implementation (0)

% generalized coordinates
gc = generate_gc;

% Initialize the parameters for the mid-term exam.
params = init_params;

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

% Simulation
T_sim = 5.0;
N_sim = round(T_sim / params.control_dt);

%% Task space inverse dynamics
disp('Task space inverse dynamics...');
gc.q = [0.0; pi/4; -pi/2];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

% Reference trajectory
r = 0.25;
w = pi;
q_center = gc.q ; 
T_ICd = T_IC_fun(q_center);
I_center_d = T_ICd(1:3, 4);
t_traj = (0:(N_sim-1)) * params.control_dt;
target_traj = I_center_d + [r*sin(w*t_traj); zeros(1, N_sim); r*cos(w*t_traj)];
dtargetdt_traj = [w*r*cos(w*t_traj); zeros(1, N_sim); -w*r*sin(w*t_traj)];
ddtargetddt_traj = [-w*w*r*sin(w*t_traj); zeros(1, N_sim); -w*w*r*cos(w*t_traj)];

figure(1); clf; 
view([120, 30]);
for sim_step = 1:N_sim
   %% Current reference
   target = target_traj(:, sim_step);
   dtargetdt = dtargetdt_traj(:, sim_step);
   ddtargetddt = ddtargetddt_traj(:, sim_step);
    
   %% Visualize
   view_ = get(gca, 'View');
   plot3(target(1), target(2), target(3), 'go', 'MarkerSize',10);
   hold on; grid on;
   plot3(target_traj(1,:), target_traj(2,:), target_traj(3,:), 'g--', 'MarkerSize', 2);
   gc =  draw_robot(gc, kin, params);
   view(view_)
   drawnow()
   refresh
   
   %% Control Input
   if use_solution == 1
   	tau = Q5_task_space_control_solution(params, gc, target, dtargetdt, ddtargetddt);
   else
    tau = Q5_task_space_control(params, gc, target, dtargetdt, ddtargetddt);
   end

    %% Simulator Loop
    t_simloop = 0:params.simulation_dt:params.control_dt;
    N_simloop = length(t_simloop);
    for j = 1:N_simloop
        [gc, ~] = Q3_forward_simulator_solution(gc, tau, params); 
    end
   
   pause(params.control_dt);
end

