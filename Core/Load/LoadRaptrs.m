function [segData, fltData] = LoadRaptrs(segDef)
% Load flight data and segments based on Segment Definition
%
%Inputs:
% segDef   - Cell Array of Segment Definitions
%
%Outputs:
% segData - Open Data formated segments
% fltData - Raptrs Log Data, structure with named fields for each unique
% simData - Simulation Data, structure with named fields for each unique
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(1, 3);

%% Create a list of Files to load
numSeg = length(segDef);
fileDataList = {};
fileConfigList = {};
fileDefList = {};
iFlt = 1;
for iSeg = 1:numSeg
    fileData = segDef{iSeg}.fileData;

    if ~isfield(segDef{iSeg}, 'fileConfig')
        segDef{iSeg}.fileConfig = [];
        warning([mfilename ' - Flight Configuration File not defined:' segDef{iSeg}.fltName]);
    end
    fileConfig = segDef{iSeg}.fileConfig;

    if ~isfield(segDef{iSeg}, 'fileDef')
        segDef{iSeg}.fileDef = [];
        warning([mfilename ' - Flight Definition File not defined:' segDef{iSeg}.fltName]);
    end
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
            warning([mfilename ' - Generating MAT file from H5... ' fileH5]);
            Hdf2mat(fileH5)
        else
            warning([mfilename ' - File not found. ' fileData]);
        end
    end
    
    % Load the Log Data
    disp(['Loading Data Log: ' fileData]);
    temp = load(fileData);
    
    fltData.(fltName).logData = temp.data;
    fltData.(fltName).logDesc = temp.desc;

    % Read the Config
    disp(['Loading Flight Configuration: ' fileConfigList{iFlt}]);
    fltData.(fltName).config = ReadConfig(fileConfigList{iFlt});

    % Read the Flight Definition
    if ~exist(fileDefList{iFlt}, 'file')
        disp(['Creating Flight Definition: ' fileDefList{iFlt}]);
        [testPtDef] = RaptrsTestPtDef(fltData.(fltName));
        SaveRaptrsDefSeg(testPtDef, fileDefList{iFlt});
    end
        
    disp(['Loading Flight Definition: ' fileDefList{iFlt}]);
    fltData.(fltName).fltDef = ReadConfig(fileDefList{iFlt});
    
end


%% Segment Flight Data
segData = {};
for iSeg = 1:numSeg
    disp(iSeg)
    fltName = segDef{iSeg}.fltName;
    
    % Convert the Raw Log data to oData
    disp('OpenData_Raptrs')
    oData = OpenData_Raptrs(fltData.(fltName).logData, fltData.(fltName).config);
    
    % Slice the oData based on the segDef
    disp('OpenData_Slice')
    segData{iSeg} = OpenData_Slice(oData, oData.time_us, segDef{iSeg}.time_us);
    disp('done')
end
