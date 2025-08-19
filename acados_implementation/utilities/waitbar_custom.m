%{ 
Authors:    Eugen Nuss(e.nuss@irt.rwth-aachen.de)

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

function waitbar_custom(iter, nIterMax)
%
%   Signature   : waitbar(iter, nIterMax)
% 
%   Description : Plot waitbar for for-loops
%
%   Parameters  : iter -> Current iterate
%                 nIterMax -> Maximum number of iterates
%
%   Return      : -
% 
%	Author(s)   : Eugen Nuss, Institute of Automatic Control, RWTH Aachen 
%                 University
%
%-------------------------------------------------------------------------%

persistent kk waitBar nIter

if strcmp(iter, 'init')
    % Initialize waitbar
    kk = 1;
    nIter = nIterMax;
    waitBar = '----------------------------------------';
    fprintf('%s', ['0 ' waitBar ' 100'])
    
elseif iter / nIter * length(waitBar) >= kk
    % Display progress
    nAdd = ceil(iter / nIter * length(waitBar) - kk);
    fprintf(repmat('\b', 1, 4 + length(waitBar)));
    waitBar(kk:kk + nAdd) = '%';
    fprintf('%s', [waitBar ' 100']);
    kk = kk + nAdd;
end

if iter == nIter
    fprintf('\n')
end