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
T_sim = 3.0;
N_sim = round(T_sim / params.control_dt);

%% Gravity Compensation
disp('Gravity Compensation...');
gc.q = [0.0; pi/4; -pi/4];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

% reference
q_des = gc.q + [pi/2; +pi/10; -pi/10]; 
dq_des = [0.0; 0.0; 0.0];
target =  subs(kin.I_r_IE, {'q1' 'q2' 'q3'}, {q_des(1) q_des(2) q_des(3)});

figure(1); clf; 
view([120, 30]);
for sim_step = 1:N_sim
   %% Visualize
   view_ = get(gca, 'View');
   plot3(target(1), target(2), target(3), 'go', 'MarkerSize',10);
   hold on; grid on;
   gc =  draw_robot(gc, kin, params);
   view(view_)
   drawnow()
   refresh
   
   %% control input
   if use_solution == 1
   	tau = Q4_gravity_compensation_solution(params, gc, q_des, dq_des);
   else
    tau = Q4_gravity_compensation(params, gc, q_des, dq_des); 
   end
    
   %% Simulator Loop
   t_simloop = 0:params.simulation_dt:params.control_dt;
   N_simloop = length(t_simloop);
   for j = 1:N_simloop
      [gc, ~] = Q3_forward_simulator_solution(gc, tau, params); 
   end
   
   pause(params.control_dt);
end

