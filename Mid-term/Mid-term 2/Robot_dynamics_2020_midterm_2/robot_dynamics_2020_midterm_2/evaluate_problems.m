% Initialize the workspace.
init_workspace

fprintf('Testing user implementations... \n')

%% Setup
% generalized coordinates
gc = generate_gc();
q_eval = [0.1, 0.2, 0.3]';
dq_eval = [0.4, 0.5, 0.6]';

% Initialize the parameters for the mid-term exam.
params = init_params();

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

%% Q1.
disp('==============');
disp('Running Q1 ...');

try
  eom = Q1_generate_eom(gc, kin, params, jac);
  disp('eom generated.');
  
  disp('Running eom.M ...');
  M = eval(subs(eom.M, gc.q, q_eval));
  disp('Running eom.g ...');
  g = eval(subs(eom.g, gc.q, q_eval));
  disp('Running eom.b ...');
  b = eval(subs(eom.b, [gc.q, gc.dq], [q_eval, dq_eval]));
  
  disp('Done.');
  
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Q2.
disp('==============');
disp('Running Q2 ...');

try
  eom = Q2_grasp_object(gc, kin, params, jac);
  disp('eom generated.');
  
  disp('Running eom.M ...');
  M = eval(subs(eom.M, gc.q, q_eval));
  disp('Running eom.g ...');
  g = eval(subs(eom.g, gc.q, q_eval));
  disp('Running eom.b ...');
  b = eval(subs(eom.b, [gc.q, gc.dq], [q_eval, dq_eval]));
  
  disp('Done.');
  
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Q3.
disp('==============');
disp('Running Q3 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  tau_eval = [0.7, 0.8, 0.9]';
  [~, user_qdd] = Q3_forward_simulator(gc_eval, tau_eval, params);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q4.
disp('==============');
disp('Running Q4 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  user_tau = Q4_gravity_compensation(params, gc_eval, q_des, dq_des);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q5.
disp('==============');
disp('Running Q5 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  T_ICd = T_IC_fun(q_des);
  I_Jp_C = I_Jp_C_fun(q_des);
  I_r_ICd = T_ICd(1:3, 4);
  I_v_Cd = I_Jp_C * dq_des;
  I_a_Cd = [0.01, 0.02, 0.03]';
  
  tau = Q5_task_space_control(params, gc_eval, I_r_ICd, I_v_Cd, I_a_Cd);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

