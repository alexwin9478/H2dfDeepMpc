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

function out = normalize_var(in, mean, std, direction)
% 
%   Signature   : out = normalize_var(int, factor, direction)
%
%   Description : Scales variable according to scaling factor. Interface
%                 better human readable than direct scaling.
%
%   Parameters  : in -> Variables to be scaled (rows: variables, columns:
%                       data points of variable)
%                 mean -> Mean
%                 std -> Standard deviation
%                 direction -> String containing scaling direction ('to-si'
%                              or 'to-scaled')
% 
%   Return      : out -> Scaled variable
% 
%-------------------------------------------------------------------------%

if ismatrix(in)
  signal_length = size(in, 2);
  std = repmat(std, 1, signal_length);
end

switch direction
  case 'to-si'
    out = in .* std + mean;
  
  case 'to-scaled'
    out = (in - mean) ./ std;
end