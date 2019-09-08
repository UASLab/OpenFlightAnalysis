% OpenFlightAnalysis - StartupOpenFlightAnalysis
%   Start file for the 'OpenFlightAnalysis Toolbox'.
%
% Notes:
%   MATLAB path list gives priority to paths listed at the top.
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
%
% Author: Chris Regan
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
