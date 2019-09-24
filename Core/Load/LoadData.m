function [fltData, segData] = LoadData(segDef)


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

%%
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
    fltData.(fltName).fltDef = ReadConfig(fileConfigList{iFlt});
end


%% Segment Flight Data

oData = {}
for iSeg = 1:numSeg
    fltName = segDef{iSeg}.fltName;
    
    timeSeg = segDef{iSeg}.time_us;

    oData{iSeg} = OpenData_RaptrsLog(fltData.(fltName).logData, fltData.(fltName).config);
    
end
    
    
