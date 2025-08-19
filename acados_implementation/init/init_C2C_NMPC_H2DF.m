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

function C2C_NMPC = init_C2C_NMPC_H2DF(version)

C2C_NMPC.name = 'C2C_NMPC';
C2C_NMPC.expdata = version;

switch version    
    case {'2024_480_to_507_GRU_normalized_271'}         
        C2C_NMPC.Dims.n_hidden_states = 8;
        C2C_NMPC.Dims.n_cell_states = 0;
        C2C_NMPC.Dims.n_LSTM_states = C2C_NMPC.Dims.n_hidden_states;
        C2C_NMPC.Dims.n_outputs = 4;
        C2C_NMPC.Labels.outputs = {'imep', 'nox', 'soot', 'mprr'}.';
        C2C_NMPC.Units.outputs = {'pa', 'ppm', 'mg/m3', 'pa/°CA'};
        C2C_NMPC.Dims.n_controls = 4;
        C2C_NMPC.Labels.controls = {'doi_main', 'p2m', 'soi_main', 'doi_h2'}.';
        C2C_NMPC.Units.controls = {'s', 'us', '°CAbTDC', 's'};
        C2C_NMPC.Dims.n_states = C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls;
        C2C_NMPC.Labels.states = [repmat({'hidden_state'}, C2C_NMPC.Dims.n_hidden_states, 1); ...
            C2C_NMPC.Labels.controls; C2C_NMPC.Labels.outputs];
        C2C_NMPC.Dims.n_cost = C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls * 2;        
        Data = struct2table(load(['2024_480_to_507_GRU_normalized_post.mat']));
    case {'2024_480_to_507_GRU_normalized_nofb'}         
        C2C_NMPC.Dims.n_hidden_states = 8;
        C2C_NMPC.Dims.n_cell_states = 0;
        C2C_NMPC.Dims.n_LSTM_states = C2C_NMPC.Dims.n_hidden_states;
        C2C_NMPC.Dims.n_outputs = 4;
        C2C_NMPC.Labels.outputs = {'imep', 'nox', 'soot', 'mprr'}.';
        C2C_NMPC.Units.outputs = {'pa', 'ppm', 'mg/m3', 'pa/°CA'};
        C2C_NMPC.Dims.n_controls = 4;
        C2C_NMPC.Labels.controls = {'doi_main', 'p2m', 'soi_main', 'doi_h2'}.';
        C2C_NMPC.Units.controls = {'s', 'us', '°CAbTDC', 's'};
        C2C_NMPC.Dims.n_states = C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls;
        C2C_NMPC.Labels.states = [repmat({'hidden_state'}, C2C_NMPC.Dims.n_hidden_states, 1); ...
            C2C_NMPC.Labels.controls; C2C_NMPC.Labels.outputs];
        C2C_NMPC.Dims.n_cost = C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls * 2;        
        Data = struct2table(load(['2024_480_to_507_GRU_normalized_post.mat']));
    case{'2024_545_to_550_IL_FF_normalized'}
        C2C_NMPC.Dims.n_hidden_states = 8;
        C2C_NMPC.Dims.n_cell_states = 0;
        C2C_NMPC.Dims.n_LSTM_states = C2C_NMPC.Dims.n_hidden_states;
        C2C_NMPC.Dims.n_outputs = 4;
        C2C_NMPC.Labels.outputs = {'imep', 'nox', 'soot', 'mprr'}.';
        C2C_NMPC.Units.outputs = {'pa', 'ppm', 'mg/m3', 'pa/°CA'};
        C2C_NMPC.Dims.n_controls = 4;
        C2C_NMPC.Labels.controls = {'doi_main', 'p2m', 'soi_main', 'doi_h2'}.';
        C2C_NMPC.Units.controls = {'s', 'us', '°CAbTDC', 's'};
        C2C_NMPC.Dims.n_states = C2C_NMPC.Dims.n_LSTM_states + C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls;
        C2C_NMPC.Labels.states = [repmat({'hidden_state'}, C2C_NMPC.Dims.n_hidden_states, 1); ...
            C2C_NMPC.Labels.controls; C2C_NMPC.Labels.outputs];
        C2C_NMPC.Dims.n_cost = C2C_NMPC.Dims.n_outputs + C2C_NMPC.Dims.n_controls * 2;
        Data = struct2table(load(['2024_480_to_481_GRU_normalized_post.mat']));    
    otherwise
        error('Define correct model version!');
end

Data.label = string(Data.label);
ind = boolean(sum(Data.label == C2C_NMPC.Labels.outputs.', 2));
C2C_NMPC.Normalization.outputs.mean = [Data.mean{boolean(ind)}].';
C2C_NMPC.Normalization.outputs.std = [Data.std{ind}].';
ind = boolean(sum(Data.label == C2C_NMPC.Labels.controls.', 2));
C2C_NMPC.Normalization.controls.mean = [Data.mean{ind}].';
C2C_NMPC.Normalization.controls.std = [Data.std{ind}].';
C2C_NMPC.Initialization = [Data.signal{ind}].';
C2C_NMPC.Initialization = C2C_NMPC.Initialization(:,1);

for ii = 1:C2C_NMPC.Dims.n_controls
    C2C_NMPC.Units.controls_norm{ii} = [C2C_NMPC.Units.controls{ii}, '/', num2str(C2C_NMPC.Normalization.controls.std(ii))];
end
for ii = 1:C2C_NMPC.Dims.n_outputs
    C2C_NMPC.Units.outputs_norm{ii} = [C2C_NMPC.Units.outputs{ii}, '/', num2str(C2C_NMPC.Normalization.outputs.std(ii))];
end
C2C_NMPC.Units.states = repmat({'1'}, C2C_NMPC.Dims.n_LSTM_states, 1);

fprintf(['Loading Parameters of NMPC "', C2C_NMPC.name, '" done!', '\n\n'])
end