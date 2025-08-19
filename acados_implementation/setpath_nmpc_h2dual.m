function varargout = setpath_nmpc_h2dual(is_submodule)
% 
%   Signature   : varargout = setpath_nmpc_ci(is_submodule)
%
%   Description : Sets/ removes all necessary paths for project.
%                 1. If project is used as a submodule within another
%                    project, only pass forward the paths to include
%                 2. Checks if a file with include paths already exists in
%                    the temp folder. If not, the last session was ended 
%                    properly and the environment was closed. In this case,
%                    set the environment.
%                 3. Otherwise or Matlab was closed without closing the 
%                    environemnt (the process id will be different) or the
%                    environment is supposed to be closed. In the first 
%                    case, set environment normally. In the latter case 
%                    clear path.
%
%   Parameters  : is_submodule -> Boolean controlling whether to forward
%                                include paths to calling function
%
%   Return      : varargout -> If project is used as a submodule, output
%                              paths to include
%
%-------------------------------------------------------------------------%

% Get current folder dir
root_dir = fileparts(mfilename('fullpath'));

% Specify required paths
include_paths = {root_dir, ...
    fullfile(root_dir, 'data'), ...
    fullfile(root_dir, 'init'), ...
    fullfile(root_dir, 'model'), ...
    fullfile(root_dir, 'nmpc'), ...
    fullfile(root_dir, '..', '..', '..', 'matlab2tikz', 'src'), ...
    genpath(fullfile(root_dir, 'utilities')), ...
    genpath(fullfile(root_dir, 'test'))}.';
% Check which case is active
if nargin > 0 && is_submodule > 0
    do = 'forward';
else
    % Define path for standard temp dir
    temp_folder = fullfile(root_dir, 'temp');
    
    % Append temp folder to include path
    include_paths{end + 1} = temp_folder;
    
    % Check if there is a stored file from current or last session
    if ~isfile([temp_folder, filesep, 'include_paths.mat'])
        do = 'open';
    else    
        stored_info = load([temp_folder, filesep, 'include_paths.mat'], ...
            'include_paths', 'current_pid');
        if stored_info.current_pid == feature('getpid')
            do = 'close';
        else
            do = 'open';
        end
    end
end

switch do
    case 'forward'
        varargout{1} = include_paths;
        
    case 'open'
        % Issue message
        fprintf('Setting project path ...');

        % Create temp folder if non existent
        if ~isfolder(temp_folder)
            mkdir(temp_folder)
        end
        
        % Add specified folders to Matlab path
        for ii = 1:size(include_paths,1)
            addpath(include_paths{ii});
        end
        
%         % Set work directory for compiled and temporary data
%         Simulink.fileGenControl('set', 'CacheFolder', temp_folder, ...
%             'CodeGenFolder', temp_folder);
        
        % Store include path array and current process ID
        current_pid = feature('getpid');
        save([temp_folder, filesep, 'include_paths.mat'], ...
            'include_paths', 'current_pid');
        
        % env.sh auxiliary
        env_sh_call();
        
    case 'close'
        % Issue message
        fprintf('Closing project ...');
        
        for ii = 1:size(stored_info.include_paths, 1)
            rmpath(stored_info.include_paths{ii});
        end
        delete([temp_folder, filesep, 'include_paths.mat'])
end

fprintf('done!\n\n');