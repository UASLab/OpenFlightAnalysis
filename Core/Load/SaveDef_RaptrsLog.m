function [testPtDef] = RaptrsTestPtDef(fileData, fileConfig)
% Load flight data and segment based on Segment Definition
%
%Inputs:
% fileData - RAPTRS Log file
%
%Outputs:
% fltData - Flight Data, structure with named fields for each unique
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(2, 2);
nargoutchk(1, 1);

%% Check that Data file exists
if ~exist(fileData, 'file')
    % Try changing the extension to .h5
    fileH5 = strrep(fileData, '.mat', '.h5');
    if exist(fileH5, 'file')
        warning('Generating MAT file from H5...');
        Hdf2mat(fileH5)
    end
end

% Load the Log Data
temp = load(fileData);

fltData.logData = temp.data;
fltData.logDesc = temp.desc;

clear temp;

%% Read the Config
fltConfig = ReadConfig(fileConfig);

%% Find Test Segments



