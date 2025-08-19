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

function [output_ref, action_ref] = generate_reference_h2dual(n_actions)
% imep_ref = 1e5 * [ones(1, 100) * 8, ones(1, 200) * 7, ones(1, 200) ...
%     * 4, ones(1, 200) * 3, ones(1, 100) * 5, ones(1, 200) * 6];
%load('nmpc_imep_ref_sequence.mat');
load('nmpc_imep_ref_sequence_standard_load.mat');
imep_ref = IMEP_Standard_Load_Cycle;

% IMEP, Nox, soot
% output_ref = [imep_ref(1:1000); zeros(1, 1000); zeros(1, 1000); zeros(1, 1000)];
% action_ref = zeros(n_actions, 1000);

length_ref = 4900;

output_ref = [imep_ref(1:length_ref)*1e5; zeros(1, length_ref); zeros(1, length_ref); zeros(1, length_ref)];
action_ref = zeros(n_actions, length_ref);


end

