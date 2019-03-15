% Start file for the 'Flight Data Analysis Toolbox'.
%
%Notes:
% MATLAB path list gives priority to paths listed at the top.
%

%Version History: Version 1.0
% 09/20/2016  C. Regan     Initial Release (v1.0)
%

%% Set the base toolbox directory
[filePath, ~] = fileparts(mfilename('fullpath'));
disp('Flight Data Analysis Toolbox is starting...');


%% Add paths
% Turn off multiple path warning
warning('off', 'MATLAB:dispatcher:pathWarning');

% Add paths, excluding those containing '.git'
addpath(regexprep(genpath(filePath), '.git', ''));

% Turn on multiple path warning
warning('on', 'MATLAB:dispatcher:pathWarning');


%% Cleanup
clear filePath
