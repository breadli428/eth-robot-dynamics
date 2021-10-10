% Initialize the workspace.
init_workspace;
init_params;

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IE = jointToEndeffectorPose(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end



%% Exercise 2.
disp('Running exercise 2...');
try
  J_3C_3 = point3ToCameraGeometricJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 3.
disp('Running exercise 3...');
try
  q_0 = [0.0; 0.0; 0.0];
  I_r_IC_des = [0.5; 0.5; 1.2];
  tol = 1e-3;
  q_ik = inverseKinematics(I_r_IC_des, q_0, tol, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 4.
disp('Running exercise 4...');
try
  C_IE = cameraFrameOrientationWithBaseRotation(1, 0, 0, 0, q, params); 
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end