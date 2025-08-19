%{ 
Authors:    Armin Norouzi(arminnorouzi2016@gmail.com),            
            David Gordon(dgordon@ualberta.ca),
            Eugen Nuss(e.nuss@irt.rwth-aachen.de)
            Alexander Winkler(winkler_a@mmp.rwth-aachen.de)
            Vasu Sharma(vasu3@ualberta.ca),


Copyright 2023 MECE,University of Alberta,
               Teaching and Research 
               Area Mechatronics in Mobile Propulsion,
               RWTH Aachen University

Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at: http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
%}

%-------------------------------------------------------------------------%
% 
%   Description: 
%   Single-Scale (SS) H2DF Control including Cycle-to-Cycle (C2C) NMPC
% 
%-------------------------------------------------------------------------%

%% Options for Script
Opts.make_solver = true;
if Opts.make_solver
    clear all;
    Opts.make_solver = true;
end
Opts.print_statistics = true;
Opts.generate_tikz = true;
Opts.plot_overview = true;

%% Construct C2C NMPC / Choose variant
C2C_NMPC.expdata = '2024_480_to_507_GRU_normalized_271'; 
load('Par_2024_0008_0008_0271.mat');
Opts.NetType = 'GRU'; % 
% C2C_NMPC.expdata = '2024_480_to_507_GRU_normalized_nofb';
% load('Par_2024_0008_0008_0282.mat'); % no imep feedback
% Opts.NetType = 'GRU_nofb'; 

% initialization:
C2C_NMPC = init_C2C_NMPC_H2DF(C2C_NMPC.expdata);
C2C_NMPC.Par = Par;
C2C_NMPC.name = C2C_NMPC.name;
C2C_NMPC.Opts.N = 3;
C2C_NMPC.Opts.nlp_solver = 'sqp'; % {'sqp', 'sqp_rti'}
C2C_NMPC.Opts.max_sqp_iter = 3;
C2C_NMPC.Opts.steplength = 1;
C2C_NMPC.Opts.gnsf_detect_struct = 'true';
C2C_NMPC.Opts.tol = 1e-4;

% set weights and bounds:
% outputs: {'imep', 'nox', 'soot', 'mprr'}
alpha = 1;
C2C_NMPC.Weights.outputs = [3, 0.001, 0.001, 0.1] * alpha;  %C2C_NMPC.Weights.outputs = [0.75, 0.01, 0.001, 0.1] * alpha; 
C2C_NMPC.Bounds.outputs = [-1e6, -1e6, -1e6, -1e6; ...
    10e5, 1800, 1.5, 15e5].'; % new data 2024 Alex
C2C_NMPC.Bounds.outputs_soft = [0, 0, 0, 0; ... % [0, 0, 0, 0; ...
    9.5e5, 1200, 0.8, 12e5].'; % new data 2024 Alex

% controls: {'doi_main', 'p2m', 'soi_main', 'doi_h2'}
beta = 0.001; %beta = 0.005
C2C_NMPC.Weights.controls = [0.6, 1, 1, 1] * beta; % 0.05
C2C_NMPC.Bounds.controls = [0.17e-3, 430, -10, 1.5e-3; ...
    0.5e-3, 1400, 10, 4e-3].'; % new data 2024 Alex, bounds according to RNG data, NEW Data more high load

% delta_controls: {'delta_doi_main', 'delta_p2m', 'delta_soi_main', 'delta_doi_h2'}
gamma = 0.1; % gamma =1 
C2C_NMPC.Weights.delta_controls = [1, 1, 1, 1] * gamma; 
% C2C_NMPC.Bounds.delta_controls = [-0.2, -0.2, -0.15, -0.2; ...
%     0.2, 0.2, 0.15, 0.2].'; % fast
C2C_NMPC.Bounds.delta_controls = [-0.015, -0.002, -0.015, -0.08; ...
    0.015, 0.002, 0.015, 0.08].'; % slightly damped
% C2C_NMPC.Bounds.delta_controls = [-0.008, -0.0015, -0.015, -0.05; ...
%     0.008, 0.0015, 0.015, 0.05].'; %  damped

% scaling / norm /std
% norm
C2C_NMPC.Bounds.outputs_norm = normalize_var(C2C_NMPC.Bounds.outputs, ...
    C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, ...
    'to-scaled');
C2C_NMPC.Bounds.outputs_norm_soft = normalize_var(C2C_NMPC.Bounds.outputs_soft, ...
    C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, ...
    'to-scaled');
C2C_NMPC.Bounds.controls_norm = normalize_var(C2C_NMPC.Bounds.controls, ...
    C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, ...
    'to-scaled');
C2C_NMPC.Bounds.delta_controls_norm = C2C_NMPC.Bounds.delta_controls;


% construct C2C NMPC:
if  Opts.make_solver
        if contains(Opts.NetType, 'GRU_nofb') == 1
            ocp_C2C = construct_C2C_NMPC_H2DF_LSTM_GRU_nofb(C2C_NMPC, C2C_NMPC.Par);
        else % 'GRU', 'LSTM'
            ocp_C2C = construct_C2C_NMPC_H2DF_LSTM_GRU(C2C_NMPC, C2C_NMPC.Par);
        end
else
    % overwrite weights again if we skip generating the solver
    W_outputs = diag(C2C_NMPC.Weights.outputs);
    W_controls = diag(C2C_NMPC.Weights.controls);
    W_delta_controls = diag(C2C_NMPC.Weights.delta_controls);
    W = blkdiag(W_controls, W_outputs, W_delta_controls);
    W_e = blkdiag(W_controls, W_outputs);
    for i = 0 : (C2C_NMPC.Opts.N -1)
        ocp_C2C.set('cost_W', W, i);
    end
    ocp_C2C.set('cost_W', W_e, C2C_NMPC.Opts.N);
    
    % bounds on u
    for i = 0 : (C2C_NMPC.Opts.N  -1)
        ocp_C2C.set('constr_lbu', [C2C_NMPC.Bounds.delta_controls_norm(:, 1)], i);
        ocp_C2C.set('constr_ubu', [C2C_NMPC.Bounds.delta_controls_norm(:, 2)], i);
    end
    
    % bounds on x
    for i = 1 : (C2C_NMPC.Opts.N)
        ocp_C2C.set('constr_lbx', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm(:, 1)], i);
        ocp_C2C.set('constr_ubx', [C2C_NMPC.Bounds.controls_norm(:, 2);  C2C_NMPC.Bounds.outputs_norm(:, 2)], i);
    end
    
    
    % bounds on h (slacks)
    for i = 1 : (C2C_NMPC.Opts.N) - 1 % lagrange term        
        ocp_C2C.set('constr_lh', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm_soft(:, 1); C2C_NMPC.Bounds.delta_controls_norm(:, 1)], i);
        ocp_C2C.set('constr_uh', [C2C_NMPC.Bounds.controls_norm(:, 2); C2C_NMPC.Bounds.outputs_norm_soft(:, 2); C2C_NMPC.Bounds.delta_controls_norm(:, 2)], i);
    end
    ocp_C2C.set('constr_lh', [C2C_NMPC.Bounds.controls_norm(:, 1); C2C_NMPC.Bounds.outputs_norm(:, 1)], C2C_NMPC.Opts.N); % mayer term
    ocp_C2C.set('constr_uh', [C2C_NMPC.Bounds.controls_norm(:, 2); C2C_NMPC.Bounds.outputs_norm(:, 2)], C2C_NMPC.Opts.N);
   
end

%% Run Closed Loop Simulation

% model dyn_H2DF inputs: {'doi_main', 'soi_pre', 'soi_main', 'doi_h2', 'imep'}
% model dyn_H2DF outputs: {'imep', 'nox', 'soot', 'mprr'}
H2DF.Par = Par; 
H2DF_Par = Par; % for simulink, dot indexing not accepted
H2DF.controls_labels(:, 1) = {'doi main', 'p2m', 'soi main', 'doi h2'};
H2DF.controls_units(:, 1) = {'s', 'us', '°CAbTDC', 's'};
H2DF.outputs_labels(:, 1) = {'imep', 'nox', 'soot', 'mprr'};
H2DF.outputs_units(:, 1) = {'pa', 'ppm', 'mg/m3', 'pa/°CA'};

% set C2C NMPC reference:
[C2C_NMPC.outputs_ref, C2C_NMPC.controls_ref] = generate_reference_h2dual(C2C_NMPC.Dims.n_controls);
% norm
C2C_NMPC.controls_ref_norm = normalize_var(C2C_NMPC.controls_ref, C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, 'to-scaled'); 
C2C_NMPC.outputs_ref_norm = normalize_var(C2C_NMPC.outputs_ref, C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, 'to-scaled');

% set C2C NMPC initializations:
% % 271 net at 3 bar
C2C_NMPC.x0 = [-0.00641768668196206; 0.212999464344029; -0.119895282775757; -0.229565845043318; ...
    -0.0898466333291564; 0.241875796377727; 0.193887842293794; -0.106956196816843; ...
    0.0292915015701450; 0.0334734339596497; 0.385663872807375; 0.150192476037635; ...
    0.296681363140564; 0.132696462712359; 0.00482822361070254; 0.0162949517757181];
C2C_NMPC.xu0 = [0.0292915015701450; 0.0334734339596497; 0.385663872807375; 0.150192476037635];

% declare arrays:
n_steps = size(C2C_NMPC.outputs_ref, 2);
Rec.physical.controls = zeros(C2C_NMPC.Dims.n_controls, n_steps);
Rec.physical.outputs = zeros(C2C_NMPC.Dims.n_outputs, n_steps);

% declare C2C NMPC arrays:
Rec.C2C_NMPC.state_traj = zeros(C2C_NMPC.Dims.n_states, C2C_NMPC.Opts.N + 1, n_steps);
Rec.C2C_NMPC.controls_traj = zeros(C2C_NMPC.Dims.n_controls, C2C_NMPC.Opts.N , n_steps);
Rec.H2DF.outputs_traj_denorm = zeros(C2C_NMPC.Dims.n_outputs, C2C_NMPC.Opts.N + 1, n_steps);
Rec.H2DF.controls_traj_denorm = zeros(C2C_NMPC.Dims.n_controls, C2C_NMPC.Opts.N , n_steps);
Rec.C2C_NMPC.states = zeros(C2C_NMPC.Dims.n_states, n_steps + 1);
Rec.C2C_NMPC.states(:, 1) = C2C_NMPC.x0;
Rec.C2C_NMPC.LSTM_states = zeros(C2C_NMPC.Dims.n_LSTM_states, n_steps + 1);
Rec.C2C_NMPC.LSTM_states(:, 1) = C2C_NMPC.x0(1:C2C_NMPC.Dims.n_LSTM_states, 1);
Rec.C2C_NMPC.controls = zeros(C2C_NMPC.Dims.n_controls, n_steps);
Rec.C2C_NMPC.controls_norm = zeros(C2C_NMPC.Dims.n_controls, n_steps);
Rec.C2C_NMPC.outputs = zeros(C2C_NMPC.Dims.n_outputs, n_steps);
Rec.C2C_NMPC.outputs_norm = zeros(C2C_NMPC.Dims.n_outputs, n_steps);
Rec.C2C_NMPC.t_nmpc = zeros(1, n_steps);
Rec.C2C_NMPC.t_sim = zeros(1, n_steps);
Rec.C2C_NMPC.solverstatus = zeros(1, n_steps);
Rec.C2C_NMPC.sqp_iter = zeros(1, n_steps);
Rec.C2C_NMPC.time_tot = zeros(1, n_steps);
Rec.C2C_NMPC.time_lin = zeros(1, n_steps);
Rec.C2C_NMPC.time_reg = zeros(1, n_steps);
Rec.C2C_NMPC.time_qp_sol = zeros(1, n_steps);
Rec.C2C_NMPC.cost = zeros(1, n_steps);

fprintf('Simulating ...\n')
waitbar_custom('init', n_steps)
tic
for ii = 1:n_steps
    tm = tic;

    % solve C2C NMPC ocp:
    if  ii > 1  
        [Rec.C2C_NMPC.controls_norm(:, ii), Rec.C2C_NMPC.controls_traj(:, :, ii), Rec.C2C_NMPC.state_traj(:, :, ii) ] = ...
            C2C_NMPC_fun(ocp_C2C, Rec.C2C_NMPC.states(1:C2C_NMPC.Dims.n_LSTM_states, ii), Rec.physical.controls_norm(:, ii - 1), Rec.physical.outputs_norm(:, ii - 1), C2C_NMPC.controls_ref_norm(:, ii), C2C_NMPC.outputs_ref_norm(:, ii), C2C_NMPC);
    else
        [Rec.C2C_NMPC.controls_norm(:, ii), Rec.C2C_NMPC.controls_traj(:, :, ii), Rec.C2C_NMPC.state_traj(:, :, ii) ] = ...
            C2C_NMPC_fun(ocp_C2C, Rec.C2C_NMPC.states(1:C2C_NMPC.Dims.n_LSTM_states, ii), Rec.C2C_NMPC.states((C2C_NMPC.Dims.n_LSTM_states + 1):(C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_controls), ii), Rec.C2C_NMPC.states((end - C2C_NMPC.Dims.n_outputs + 1):end, ii), C2C_NMPC.controls_ref_norm(:, ii), C2C_NMPC.outputs_ref_norm(:, ii), C2C_NMPC);
    end

    % store C2C NMPC info:
    Rec.C2C_NMPC.t_nmpc(1, ii) = toc(tm);
    Rec.C2C_NMPC.solverstatus(1, ii) = ocp_C2C.get('status');
    Rec.C2C_NMPC.sqp_iter(1, ii) = ocp_C2C.get('sqp_iter');
    Rec.C2C_NMPC.time_tot(1, ii) = ocp_C2C.get('time_tot');
    Rec.C2C_NMPC.time_lin(1, ii) = ocp_C2C.get('time_lin');
    Rec.C2C_NMPC.time_reg(1, ii) = ocp_C2C.get('time_reg');
    Rec.C2C_NMPC.time_qp_sol(1, ii) = ocp_C2C.get('time_qp_sol');
    Rec.C2C_NMPC.cost(1, ii) = ocp_C2C.get_cost();

    % print C2C NMPC statistics:
    if  Opts.print_statistics == true
        ocp_C2C.print('stat')
    end
    
    % plant H2DF:
    Rec.H2DF.controls_norm(:, ii) = Rec.C2C_NMPC.controls_norm(:, ii);
    if contains(Opts.NetType, 'DNN') == 1
       if ii > 1
            [Rec.H2DF.outputs_norm(:, ii), Rec.H2DF.LSTM_states(:, ii)] = ...
                dynout_h2dual_vsr1123_101(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.H2DF.controls_norm(:, ii); Rec.physical.outputs_norm(:, ii - 1)], H2DF.Par);
        else
            [Rec.H2DF.outputs_norm(:, ii), Rec.H2DF.LSTM_states(:, ii)] = ...
                dynout_h2dual_vsr1123_101(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.H2DF.controls_norm(:, ii); Rec.C2C_NMPC.states(end - 3:end, ii)], H2DF.Par);
       end      
    elseif contains(Opts.NetType, 'GRU_nofb') == 1 % LSTM GRU
       [Rec.H2DF.outputs_norm(:, ii), Rec.H2DF.LSTM_states(:, ii)] = ...
           dyn_H2DF_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), Rec.H2DF.controls_norm(:, ii), H2DF.Par);  
       % inject noise here
       Rec.H2DF.outputs_norm(:, ii) = Rec.H2DF.outputs_norm(:, ii);
    else % LSTM GRU
        if ii > 1
            [Rec.H2DF.outputs_norm(:, ii), Rec.H2DF.LSTM_states(:, ii)] = ...
                dyn_H2DF_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.H2DF.controls_norm(:, ii); Rec.physical.outputs_norm(1, ii - 1)], H2DF.Par);
            % inject noise here
            % Rec.H2DF.outputs_norm(:, ii) = Rec.H2DF.outputs_norm(:, ii);
        else
            [Rec.H2DF.outputs_norm(:, ii), Rec.H2DF.LSTM_states(:, ii)] = ...
                dyn_H2DF_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.H2DF.controls_norm(:, ii); Rec.C2C_NMPC.states(end - 3, ii)], H2DF.Par);
        end
    end

    Rec.H2DF.states(:, ii) = [Rec.H2DF.LSTM_states(:, ii); Rec.H2DF.controls_norm(:, ii); Rec.H2DF.outputs_norm(:, ii)];
    % norm
    Rec.H2DF.controls(:, ii) = normalize_var(Rec.H2DF.controls_norm(:, ii), C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, 'to-si');
    Rec.H2DF.outputs(:, ii) = normalize_var(Rec.H2DF.outputs_norm(:, ii), C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, 'to-si');
    
    % denorm trajectories
    for jj = 1 : C2C_NMPC.Opts.N
        Rec.H2DF.controls_traj_denorm(:, jj, ii) = normalize_var(Rec.C2C_NMPC.controls_traj(:, jj, ii), C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, 'to-si');    
    end
    for jj = 1 : C2C_NMPC.Opts.N + 1
        Rec.H2DF.outputs_traj_denorm(:, jj, ii) = normalize_var(Rec.C2C_NMPC.state_traj( (end-C2C_NMPC.Dims.n_outputs +1): end, jj, ii), C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, 'to-si');    
    end
    
    % physical controls and outputs:
    Rec.physical.controls_norm(:, ii) = Rec.H2DF.controls_norm(:, ii);
    Rec.physical.controls(:, ii) = Rec.H2DF.controls(:, ii);
    Rec.physical.controls_units(:, 1) = H2DF.controls_units(:, 1);
    Rec.physical.controls_labels(:, 1) = H2DF.controls_labels(:, 1);
    Rec.physical.outputs_norm(:, ii) = Rec.H2DF.outputs_norm(:, ii);
    Rec.physical.outputs(:, ii) = Rec.H2DF.outputs(:, ii);
    Rec.physical.outputs_units(:, 1) = H2DF.outputs_units(:, 1);
    Rec.physical.outputs_labels(:, 1) = H2DF.outputs_labels(:, 1);
    % Rec.physical.outputs_traj(:,:, ii) = Rec.H2DF.outputs_traj_denorm(:,:, ii);
    % Rec.physical.controls_traj(:,:, ii) = Rec.H2DF.controls_traj_denorm(:,:, ii);
   
    % state augmentation:
    if contains(Opts.NetType, 'DNN') == 1
        % no state augmentation for DNN FF net only implementation
    elseif contains(Opts.NetType, 'GRU_nofb') == 1
        Rec.C2C_NMPC.LSTM_states(:, ii + 1) = ...
            dyn_H2DF_state_augmentation_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), Rec.physical.controls_norm(:, ii), H2DF.Par);
        Rec.C2C_NMPC.states(:, ii + 1) = [Rec.C2C_NMPC.LSTM_states(:, ii + 1); Rec.physical.controls_norm(:, ii); Rec.physical.outputs_norm(:, ii)];
    else % LSTM, GRU
        if ii > 1
            Rec.C2C_NMPC.LSTM_states(:, ii + 1) = ...
                dyn_H2DF_state_augmentation_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.physical.controls_norm(:, ii); Rec.physical.outputs_norm(1, ii - 1)], H2DF.Par);
        else
            Rec.C2C_NMPC.LSTM_states(:, ii + 1) = ...
                dyn_H2DF_state_augmentation_GRU(Rec.C2C_NMPC.LSTM_states(:, ii), [Rec.physical.controls_norm(:, ii); Rec.C2C_NMPC.states(end - 3, ii)], H2DF.Par);
        end
        Rec.C2C_NMPC.states(:, ii + 1) = [Rec.C2C_NMPC.LSTM_states(:, ii + 1); Rec.physical.controls_norm(:, ii); Rec.physical.outputs_norm(:, ii)];
    end


    Rec.t_sim(1, ii) = toc(tm);
    waitbar_custom(ii)
end
toc
fprintf('\n')

costs_C2C = 0; % dummy

%% Plotting
% overview / all
if Opts.plot_overview
plot_results_C2C_NMPC(Rec.C2C_NMPC.state_traj, Rec.physical.controls, Rec.physical.controls_labels, Rec.physical.controls_units, Rec.physical.outputs, Rec.physical.outputs_labels, Rec.physical.outputs_units, C2C_NMPC.outputs_ref, ...
    Rec.C2C_NMPC.time_tot, Rec.C2C_NMPC.sqp_iter, Rec.C2C_NMPC.solverstatus, Rec.C2C_NMPC.cost, costs_C2C, C2C_NMPC, n_steps, Opts.generate_tikz)
end

%% Auxiliary Functions
function [u_opt, u_traj, x_traj] = C2C_NMPC_fun(ocp_C2C, last_lstm_states, last_controls_norm, last_outputs_norm, controls_ref_norm, outputs_ref_norm, C2C_NMPC)
    % set reference:
    ref_norm = [controls_ref_norm; outputs_ref_norm; zeros(C2C_NMPC.Dims.n_controls, 1)]; %delta control
    ref_norm_e = [controls_ref_norm; outputs_ref_norm];
    for ii = 0:(C2C_NMPC.Opts.N - 1)
        ocp_C2C.set('cost_y_ref', ref_norm, ii);
    end
    ocp_C2C.set('cost_y_ref_e', ref_norm_e);
    
    % set last states:
    ocp_C2C.set('constr_x0', [last_lstm_states; last_controls_norm; last_outputs_norm]); % inject noise here
    
    % solve ocp for next step:
    ocp_C2C.solve();
    u_traj = ocp_C2C.get('u');
    x_traj = ocp_C2C.get('x');
    u_opt = last_controls_norm + u_traj(:, 1);
end

function plot_results_C2C_NMPC(x_traj, controls, controls_labels, controls_units, outputs, outputs_labels, outputs_units, outputs_ref, t_nmpc, sqp_iter, solver_status, solver_cost, costs_C2C, C2C_NMPC, n_steps, generate_tikz)


controls_C2C_NMPC = zeros(C2C_NMPC.Dims.n_controls, n_steps);
for ii = 1:length(controls(1, :))
    controls_C2C_NMPC(:, ii) = normalize_var(x_traj((C2C_NMPC.Dims.n_LSTM_states + 1):(C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_controls), 2, ii), C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, 'to-si');
end

outputs_C2C_NMPC = zeros(C2C_NMPC.Dims.n_outputs, n_steps);
for ii = 1:length(controls(1, :))
    outputs_C2C_NMPC(:, ii) = normalize_var(x_traj((end - C2C_NMPC.Dims.n_outputs + 1):end, 2, ii), C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, 'to-si');
end


for ii = 1:length(controls(1, :))
    for jj = 1:(C2C_NMPC.Opts.N + 1)
        x_traj((C2C_NMPC.Dims.n_LSTM_states + 1):(C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_controls), jj, ii) = normalize_var(x_traj((C2C_NMPC.Dims.n_LSTM_states + 1):(C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_controls), jj, ii), C2C_NMPC.Normalization.controls.mean, C2C_NMPC.Normalization.controls.std, 'to-si');
    end
end
for ii = 1:length(controls(1, :))
    for jj = 1:(C2C_NMPC.Opts.N + 1)
        x_traj((end - C2C_NMPC.Dims.n_outputs + 1):end, jj, ii) = normalize_var(x_traj((end - C2C_NMPC.Dims.n_outputs + 1):end, jj, ii), C2C_NMPC.Normalization.outputs.mean, C2C_NMPC.Normalization.outputs.std, 'to-si');
    end
end

n_rows = 5; n_cols = 2;

figure_x = figure();
figure_x.Color = [1 1 1];

for ii = 1:length(controls(:, 1))
    subplot(n_rows, n_cols, 1 + (ii - 1) * n_cols)
    grid on; hold on; box on
    % for jj = 1:(length(controls(1, :)) - C2C_NMPC.Opts.N)
    %     plot_1 = plot((0:(C2C_NMPC.Opts.N)) + ones(1, C2C_NMPC.Opts.N + 1) * (jj - 1), x_traj(8 + ii, 1:end, jj), 'LineStyle', '-', 'Linewidth', 1, 'Color', [107/255, 142/255,35/255], 'LineStyle', '-');
    % end
    plot_2 = plot(1:n_steps, controls(ii, :), 'LineStyle', '-', 'Linewidth', 1, 'Color', [0 0 1]);
    plot_3 = plot(1:n_steps, controls_C2C_NMPC(ii, :), 'LineStyle', '-', 'Linewidth', 1, 'Color', [0 0 0]);
    ylabel([controls_labels(ii, 1), ' in ', controls_units(ii, 1)])
    xlabel(['Cycle', ' in ', '–'])
    % legend([plot_1, plot_2, plot_3], 'trajectory', 'physical plant', 'C2C NMPC')
    legend([plot_2, plot_3], 'physical plant', 'C2C NMPC')
    % legend('physical plant', 'C2C NMPC');
    axis_1(ii) = gca;
end

for ii = 1:length(outputs(:, 1))
    subplot(n_rows, n_cols, 2 + (ii - 1) * n_cols)
    grid on; hold on; box on
    % for jj = 1:(length(controls(1, :)) - C2C_NMPC.Opts.N)
    %     plot_1 = plot((0:(C2C_NMPC.Opts.N)) + ones(1, C2C_NMPC.Opts.N + 1) * (jj - 1), x_traj(11 + ii, 1:end, jj), 'LineStyle', '-', 'Linewidth', 1, 'Color', [107/255, 142/255,35/255], 'LineStyle', '-');
    % end
    plot_2 = plot(1:n_steps, outputs(ii, :), 'LineStyle', '-', 'Linewidth', 1, 'Color', [0 0 1]);
    plot_3 = plot(1:n_steps, outputs_C2C_NMPC(ii, :), 'LineStyle', '-', 'Linewidth', 1, 'Color', [0 0 0]);
    plot_4 = plot(1:n_steps, outputs_ref(ii, :), 'LineStyle', '--', 'Linewidth', 1, 'Color', [1 0 0]);
    ylabel([outputs_labels(ii, 1), ' in ', outputs_units(ii, 1)])
    xlabel(['Cycle', ' in ', '–'])
    % legend([plot_1, plot_2, plot_3, plot_4], 'trajectory', 'physical plant', 'C2C NMPC', 'reference');
    legend([plot_2, plot_3, plot_4], 'physical plant', 'C2C NMPC', 'reference');
    % legend('physical plant', 'C2C NMPC', 'reference');
    axis_2(ii) = gca;
end


subplot(n_rows, n_cols, 10);
grid on; hold on; box on
yyaxis left
plot(t_nmpc * 1000, 'marker', '.');
plot(t_nmpc ./ sqp_iter * 1000, 'marker', 'D');
ylabel('t in ms')
yyaxis right
plot(solver_status);
plot(sqp_iter);
ylabel('-')
xlabel(['Cycle', ' in ', '–'])
legend('total', 'per iter', 'status', 'iter')
axis_3 = gca;
% linkaxes([axis_1, axis_2, axis_3, axis_4], 'x');
linkaxes([axis_1, axis_2, axis_3], 'x');

if generate_tikz
    figFileName="../results/Plots"+ C2C_NMPC.expdata+"_OverviewResultsMiLMatlab";
    savefig(figFileName);
    saveas(gcf,figFileName,"jpg");
    % cleanfigure('targetResolution', 20)
    matlab2tikz(convertStringsToChars(figFileName+'.tex'),'showInfo', false);
end

figure
plot(solver_status)
ylabel('Solver Status','Interpreter', 'latex')
xlabel("\#Cycles",'Interpreter', 'latex')
ylim([0,4])
% solver status:
% {0 = success}
% {1 = failure}
% {2 = maximum number of iterations reached}
% {3 = minimum step size in QP solver reached}
% {4 = QP solver failed}
figFileName="../results/Plots"+ C2C_NMPC.expdata+"_SolverStability";
savefig(figFileName);
saveas(gcf,figFileName,"jpg");

figure
plot(1:n_steps, (outputs(1, :)- C2C_NMPC.outputs_ref(1,:)), 'LineStyle', '-', 'Linewidth', 1, 'Color', [0 1 0]);
xlabel("\#Cycles",'Interpreter', 'latex')
ylabel('IMEP Tracking Error in Pa','Interpreter', 'latex')
figFileName="../results/Plots"+ C2C_NMPC.expdata+"_IMEP_Tracking_Error";
savefig(figFileName);
saveas(gcf,figFileName,"jpg");

disp('Mean Value of Solver in ms: ')
disp(mean(t_nmpc * 1000))

end

