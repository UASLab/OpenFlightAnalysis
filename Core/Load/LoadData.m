function [segData, fltData] = LoadData(segDef)
% Load flight data and segment based on Segment Definition
%
%Inputs:
% segDef   - Cell Array of Segment Definitions
%
%Outputs:
% segData - Open Data formated segments
% fltData - Flight Data, structure with named fields for each unique
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(1, 2);

%% Create a list of Files to load
numSeg = length(segDef);
fileDataList = {};
fileConfigList = {};
fileDefList = {};
iFlt = 1;
for iSeg = 1:numSeg
    fileData = segDef{iSeg}.fileData;
    fileConfig = segDef{iSeg}.fileConfig;
    fileDef = segDef{iSeg}.fileDef;
    if ~any(contains(fileDataList, fileData))
        fileDataList{iFlt} = fileData;
        fileConfigList{iFlt} = fileConfig;
        fileDefList{iFlt} = fileDef;
        iFlt = iFlt+1;
    end
end

%% Load Flight Data
fltData = struct();
numFlt = length(fileDataList);
for iFlt = 1:numFlt
    fileData = fileDataList{iFlt};
    [~, fltName, ~] = fileparts(fileData);

    % Check that Data file exists
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
    
    fltData.(fltName).logData = temp.data;
    fltData.(fltName).logDesc = temp.desc;

    % Read the Config
    fltData.(fltName).config = ReadConfig(fileConfigList{iFlt});

    % Read the Flight Definition
    fltData.(fltName).fltDef = ReadConfig(fileDefList{iFlt});
end


%% Segment Flight Data
segData = {};
for iSeg = 1:numSeg
    fltName = segDef{iSeg}.fltName;
    
    % Convert the Raw Log data to oData
    oData = OpenData_RaptrsLog(fltData.(fltName).logData, fltData.(fltName).config);
    
    % Slice the oData based on the segDef
    segData{iSeg} = OpenData_Slice(oData, oData.time_us, segDef{iSeg}.time_us);
end
