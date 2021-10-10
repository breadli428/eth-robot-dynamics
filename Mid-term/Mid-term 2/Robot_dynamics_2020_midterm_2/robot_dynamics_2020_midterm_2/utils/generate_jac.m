% generate jacobians
function jac = generate_jac(gc, kin, params)
% By calling:
%   jac = generate_jac(gen_cor, kin, dyn)
% a struct 'jac' is returned that contains the translation and rotation
% jacobians of the center of masses

%% Setup
q = gc.q;
dq = gc.dq;

T_Ik = kin.T_Ik;
R_Ik = kin.R_Ik;
I_r_IE = kin.I_r_IE;
I_r_IC = kin.I_r_IC;
T_IE = kin.T_IE;
T_IC = kin.T_IC;

%relative positions
k_r_ks = params.k_r_ks;

%% Compute link jacobians
I_Jp = cell(3,1); % gc dim
I_Jr = cell(3,1);

for k= 1:3
    % create containers
    I_Jp{k} = sym(zeros(3,3));
    I_Jr{k} = sym(zeros(3,3));
    
    % translational jacobian at the center of gravity s in frame I
    I_r_ks = [eye(3) zeros(3,1)]*T_Ik{k}*[k_r_ks{k};1]; % COM location in I
    I_Jp{k} = jacobian(I_r_ks, q);
    
    % rotational jacobian in frame I (_{I}n_{k})
    if k == 1
        I_Jr{k}(1:3,1) = R_Ik{1} * [0; 1; 0];
    else
        % copy columns of k-1 jacobian
        I_Jr{k} = I_Jr{k-1};
        
        % evaluate new column
        I_Jr{k}(1:3,k) = R_Ik{k} * [0; 0; 1];
    end
    
    % simplify expressions
    I_Jp{k} = simplify(I_Jp{k});
    I_Jr{k} = simplify(I_Jr{k});
end

% Compute the end effector jacobians in frame I
I_Jp_E = simplify(jacobian(I_r_IE, q));
I_Jr_E = I_Jr{3};

% Compute the time derivative of the end effector Jacobians
I_dJp_E = simplify(dAdt(I_Jp_E,q,dq));
I_dJr_E = simplify(dAdt(I_Jr_E,q,dq));

% Compute the camer jacobians in frame I
I_Jp_C = simplify(jacobian(I_r_IC, q));
I_Jr_C = I_Jr{3};

% Compute the time derivative of the end effector Jacobians
I_dJp_C = simplify(dAdt(I_Jp_C,q,dq));
I_dJr_C = simplify(dAdt(I_Jr_C,q,dq));

% % Generate function files from symbolic expressions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating Jacobian files... ');
matlabFunction(I_Jp_C, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jp_C_fun'));
matlabFunction(I_dJp_C, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJp_C_fun'));

matlabFunction(I_Jr_C, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jr_C_fun'));
matlabFunction(I_dJr_C, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJr_C_fun'));
% 
fprintf('done!\n')

% % Store jacobians in output struct
jac.I_Jp = I_Jp;
jac.I_Jr = I_Jr;
jac.I_Jp_E = I_Jp_E;
jac.I_Jr_E = I_Jr_E;
jac.I_dJp_E = I_dJp_E;
jac.I_dJr_E = I_dJr_E;
jac.I_Jp_C = I_Jp_C;
jac.I_Jr_C = I_Jr_C;
jac.I_dJp_C = I_dJp_C;
jac.I_dJr_C = I_dJr_C;

end
