function ocp_C2C = construct_C2C_NMPC_H2DF_LSTM_GRU(C2C_NMPC, Par)

ocp_model = construct_ocp_model(C2C_NMPC, Par);
ocp_C2C = construct_ocp(C2C_NMPC, ocp_model);

end

%% Auxiliary Functions

function ocp_model = construct_ocp_model(C2C_NMPC, Par)

ocp_model = acados_ocp_model();
ocp_model.set('name', C2C_NMPC.name);
ocp_model.set('T', 1 * C2C_NMPC.Opts.N);

% set dynamics:
last_LSTM_states = casadi.MX.sym('last_LSTM_states', C2C_NMPC.Dims.n_LSTM_states);
last_controls = casadi.MX.sym('last_controls', C2C_NMPC.Dims.n_controls);
last_outputs = casadi.MX.sym('last_outputs', C2C_NMPC.Dims.n_outputs);
last_states = [last_LSTM_states; last_controls; last_outputs];
delta_controls = casadi.MX.sym('delta_controls', C2C_NMPC.Dims.n_controls);
controls = last_controls + delta_controls;

[outputs, LSTM_states] = dyn_H2DF_GRU(last_LSTM_states, [controls ; last_outputs(1)], Par);
discrete_dynamics = [LSTM_states; controls; outputs];

ocp_model.set('sym_x', last_states);
ocp_model.set('sym_u', delta_controls);
ocp_model.set('dyn_type', 'discrete');
ocp_model.set('dyn_expr_phi', discrete_dynamics);

% set bounds on controls:

Jbu = eye(length(delta_controls));
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', C2C_NMPC.Bounds.delta_controls_norm(:, 1));
ocp_model.set('constr_ubu', C2C_NMPC.Bounds.delta_controls_norm(:, 2));
% 

% set bounds on states:
Jbx = zeros(length(last_controls), length(last_states));
Jbx(1, length(last_LSTM_states) + 1) = 1;
Jbx(2, length(last_LSTM_states) + 2) = 1;
Jbx(3, length(last_LSTM_states) + 3) = 1;
Jbx(4, length(last_LSTM_states) + 4) = 1;
Jbx(5, length(last_LSTM_states)+length(last_controls) + 1) = 1;
Jbx(6, length(last_LSTM_states)+length(last_controls) + 2) = 1;
Jbx(7, length(last_LSTM_states)+length(last_controls) + 3) = 1;
Jbx(8, length(last_LSTM_states)+length(last_controls) + 4) = 1;

ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_Jbx_e', Jbx);
ocp_model.set('constr_lbx', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm(:, 1)]);
ocp_model.set('constr_ubx', [C2C_NMPC.Bounds.controls_norm(:, 2);  C2C_NMPC.Bounds.outputs_norm(:, 2)]);
ocp_model.set('constr_lbx_e', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm(:, 1)]);
ocp_model.set('constr_ubx_e', [C2C_NMPC.Bounds.controls_norm(:, 2); C2C_NMPC.Bounds.outputs_norm(:, 2)]);


% set slack variables:
% {'doi_main', 'soi_pre', 'soi_main', 'doi_h2', 'imep', 'nox, 'soot', 'mprr', 'delta_doi_main  ', 'delta_soi_pre', 'delta_soi_main', 'delta_doi_h2'}
ocp_model.set('constr_expr_h', [last_controls; last_outputs; delta_controls]);
ocp_model.set('constr_expr_h_e', [last_controls; last_outputs]);

slack_variable_weights.controls = [1, 1, 1, 1] * 1; % already constrainted with X, no soft bounds neede / wanted
slack_variable_weights.outputs = [1, 1, 1, 1] * 1;
slack_variable_weights.delta_controls = [1, 1, 1, 1] * 1; % already constrainted with u, no soft bounds neede / wanted
Z = blkdiag(diag(slack_variable_weights.controls), diag(slack_variable_weights.outputs), diag(slack_variable_weights.delta_controls));
Z_e = blkdiag(diag(slack_variable_weights.controls), diag(slack_variable_weights.outputs));
z = [slack_variable_weights.controls.'; slack_variable_weights.outputs.'; slack_variable_weights.delta_controls.'];
z_e = [slack_variable_weights.controls.'; slack_variable_weights.outputs.'];

Jsh = eye(length(last_controls) + length(last_outputs) + length(delta_controls));
Jsh_e = eye(length(last_controls) + length(last_outputs));
ocp_model.set('constr_Jsh', Jsh);
ocp_model.set('constr_Jsh_e', Jsh_e);
% soft bounds ~= hard bounds
ocp_model.set('constr_lh', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm_soft(:, 1); C2C_NMPC.Bounds.delta_controls_norm(:, 1)]);
ocp_model.set('constr_uh', [C2C_NMPC.Bounds.controls_norm(:, 2); C2C_NMPC.Bounds.outputs_norm_soft(:, 2); C2C_NMPC.Bounds.delta_controls_norm(:, 2)]);
ocp_model.set('constr_lh_e', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm_soft(:, 1)]);
ocp_model.set('constr_uh_e', [C2C_NMPC.Bounds.controls_norm(:, 2); C2C_NMPC.Bounds.outputs_norm_soft(:, 2)]);

% map states and delta_actions on cost function:
% cost_function = {'doi_main', 'p2m', 'soi_main', 'doi_h2', 'imep', 'nox, 'soot', 'mprr', 'delta_doi_main', 'delta_soi_pre', 'delta_soi_main', 'delta_doi_h2'}
% lagrange term: y = Vx * x + Vu * u
% meyer term: y_e = Vx * x

% x = {'lstm_states', 'doi_main', 'soi_pre', 'soi_main', 'doi_h2', 'imep', 'nox, 'soot', 'mprr'}
Vx = zeros(C2C_NMPC.Dims.n_cost, length(last_states));
ii = C2C_NMPC.Dims.n_LSTM_states;
Vx(1, ii + 1) = 1; 
Vx(2, ii + 2) = 1;
Vx(3, ii + 3) = 1; 
Vx(4, ii + 4) = 1; 
Vx(5, ii + 5) = 1; 
Vx(6, ii + 6) = 1; 
Vx(7, ii + 7) = 1;
Vx(8, ii + 8) = 1; 
Vx_e = Vx(1:(end - C2C_NMPC.Dims.n_controls), :);

% u = {'delta_doi_main', 'delta_soi_pre', 'delta_soi_main', 'delta_doi_h2'}
Vu = zeros(C2C_NMPC.Dims.n_cost, length(delta_controls));
ii = C2C_NMPC.Dims.n_controls + C2C_NMPC.Dims.n_outputs + 1;
Vu (ii, 1) = 1;
Vu (ii + 1, 2) = 1; 
Vu (ii + 2, 3) = 1;
Vu (ii + 3, 4) = 1; 

% define weight matrix:
W_outputs = diag(C2C_NMPC.Weights.outputs);
W_controls = diag(C2C_NMPC.Weights.controls);
W_delta_controls = diag(C2C_NMPC.Weights.delta_controls);
W = blkdiag(W_controls, W_outputs, W_delta_controls);
W_e = blkdiag(W_controls, W_outputs);

% set costs:
ocp_model.set('cost_Vu', Vu);   
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);
ocp_model.set('cost_Z', Z);
ocp_model.set('cost_Z_e', Z_e);
ocp_model.set('cost_z', z);
ocp_model.set('cost_z_e', z_e);
ocp_model.set('cost_type', 'linear_ls');
ocp_model.set('cost_type_e', 'linear_ls');

% initialize reference and states:
ocp_model.set('constr_x0', zeros(length(last_states), 1));
ocp_model.set('cost_y_ref', zeros(size(W, 1), 1));
ocp_model.set('cost_y_ref_e', zeros(size(W_e, 1), 1));
end

function ocp_C2C = construct_ocp(C2C_NMPC, ocp_model)
disp('Creating acados ocp...');

% nlp_solver_ext_qp_res = 1;
% nlp_solver_warm_start_first_qp = 1;
qp_solver_cond_N = C2C_NMPC.Opts.N;
% qp_solver_cond_ric_alg = 0;
% qp_solver_ric_alg = 0;
% qp_solver_warm_start = 1;
% qp_solver_max_iter = max_iter; % maybe vary this, only 10 might be needed
    
% acados ocp opts
ocp_opts = acados_ocp_opts();
nlp_solver_exact_hessian = 'false'; % 'true':exact_hessian, 'false':gauss-newton
regularize_method = 'project_reduc_hess'; % 'project', 'project_reduc_hess', 'mirror', 'convexify'
qp_solver_iter_max = 50; % C2C_NMPC.Opts.max_sqp_iter; % default is 50; OSQP needs a lot sometimes
qp_solver_warm_start = 2; % 0: cold, 1: warm, 2: hot

ocp_opts.set('output_dir', fullfile('temp', ['build_acados_', char(datetime("today"))]));
ocp_opts.set('compile_interface', 'auto');
ocp_opts.set('codgen_model', 'true');
ocp_opts.set('param_scheme_N', C2C_NMPC.Opts.N);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian); 
% ToDo: Change regulatization
ocp_opts.set('regularize_method', regularize_method); 
% ocp_opts.set('nlp_solver_ext_qp_res', nlp_solver_ext_qp_res);
if (strcmp(C2C_NMPC.Opts.nlp_solver, 'sqp')) % not available for sqp_rti
    ocp_opts.set('nlp_solver_max_iter', C2C_NMPC.Opts.max_sqp_iter);
    ocp_opts.set('nlp_solver_tol_stat', C2C_NMPC.Opts.tol);
    ocp_opts.set('nlp_solver_tol_eq', C2C_NMPC.Opts.tol);
    ocp_opts.set('nlp_solver_tol_ineq', C2C_NMPC.Opts.tol);
    ocp_opts.set('nlp_solver_tol_comp', C2C_NMPC.Opts.tol);
end

qp_solver = 'partial_condensing_hpipm';
% qp_solver = 'full_condensing_hpipm';
% qp_solver = 'full_condensing_qpoases';
% qp_solver = 'partial_condensing_osqp';
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
    ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
end
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);

ocp_opts.set('sim_method', 'discrete');
ocp_opts.set('nlp_solver_step_length', C2C_NMPC.Opts.steplength);

simulink_opts = get_acados_simulink_opts();

% Input weighting matrices to simulink block
simulink_opts.inputs.cost_W_0 = 1;
simulink_opts.inputs.cost_W = 1;
simulink_opts.inputs.cost_W_e = 1;
simulink_opts.inputs.reset_solver = 1; % reset solver flag

% Output action and state trajectory from simulink block
simulink_opts.outputs.utraj = 1;
simulink_opts.outputs.xtraj = 1;
simulink_opts.outputs.cost_value = 1;
simulink_opts.outputs.KKT_residuals = 1;


simulink_opts.samplingtime = '-1';

ocp_C2C = acados_ocp(ocp_model, ocp_opts, simulink_opts);
root = pwd();

fprintf('done!\n\n');

cd '.\c_generated_code';

disp('generating s fun ...');
make_sfun;
disp('successfully generated s fun!\n\n');

cd(root);

end