% generate kinematics
function kin = generate_kin(q, params)
%% Create kinematics container
kin = struct();

% homogeneous transformations from frame k to the inertial frame
kin.T_Ik = cell(3,1);

% rotation matrices from frame k to the inertial frame
kin.R_Ik = cell(3,1);

%% Homogeneous transformations

%kinematic params
theta = params.theta;
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l31 = params.l31;
l32 = params.l32;
l4 = params.l4;
l5 = params.l5;

% Joint positions
q1 = q(1);
q2 = q(2);
q3 = q(3);

% compute T_I0
p_I0_I = [0; 0; l0];
C_I0 = [0 0 1;
    1 0 0;
    0 1 0];
T_I0 = [C_I0 p_I0_I;
    0 0 0 1];

% compute T_01
p_01_0 = [0; l1; 0];
C_01 = [cos(q1) 0 sin(q1);
    0 1 0;
    -sin(q1) 0 cos(q1)];
T_01 = [C_01 p_01_0;
    0 0 0 1];

% compute T_12
p_12_1 = [0; 0; 0];
C_12 = [cos(q2) -sin(q2) 0;
    sin(q2) cos(q2) 0;
    0 0 1];
T_12 = [C_12 p_12_1;
    0 0 0 1];

% compute T_23
p_23_2 = [l2; 0; 0];
C_23 = [cos(q3) -sin(q3) 0;
    sin(q3) cos(q3) 0;
    0 0 1];
T_23 = [C_23 p_23_2;
    0 0 0 1];

% compute T 3E
C_3E = eye(3,3);
p_3E_3 = [l31 + l32; 0; 0];
T_3E = [C_3E p_3E_3;
    0 0 0 1];

% compute T_3C
C_3C = [cos(theta-pi/2), -sin(theta-pi/2), 0;
      sin(theta-pi/2), cos(theta-pi/2), 0;
      0, 0, 1];
p_3C_3 = [l31; 0; 0] + C_3C*[l5; l4; 0];

T_3C = [C_3C p_3C_3;
      0 0 0 1];
        
% homogeneous transformations from frame k to frame I
kin.T_Ik{1} = simplify(T_I0 * T_01);
kin.T_Ik{2} = simplify(kin.T_Ik{1}*T_12);
kin.T_Ik{3} = simplify(kin.T_Ik{2}*T_23);

% rotation matrices from frame k to frame I
kin.R_Ik{1} = kin.T_Ik{1}(1:3,1:3);
kin.R_Ik{2} = kin.T_Ik{2}(1:3,1:3);
kin.R_Ik{3} = kin.T_Ik{3}(1:3,1:3);

%% Endeffector
% end-effector homogeneous transformation and position
kin.T_IE = simplify(kin.T_Ik{3} * T_3E);
kin.R_IE =  kin.T_IE(1:3,1:3);
kin.I_r_IE = kin.T_IE(1:3,4);

%% Camera
% Camera homogeneous transformation and position
kin.T_IC = simplify(kin.T_Ik{3} * T_3C);
kin.R_IC =  kin.T_IC(1:3,1:3);
kin.I_r_IC = kin.T_IC(1:3,4);

%% Matlab functions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating camera transform file... ');
matlabFunction(kin.T_IC, 'vars', {q}, 'file', strcat(dpath,'/T_IC_fun'));
fprintf('done!\n')

end
