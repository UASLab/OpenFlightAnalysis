function [data, desc] = HdfLoad(fileLoad, rootPath)
%% HdfLoad
% basePath = 'G:\Team Drives\UAVLab\Flight Data';
% ac = 'Thor';
% fltStr = 'FLT118';
% ext = '.h5';
% 
% fileLoad = fullfile(basePath, ac, [ac, fltStr], [ac, fltStr, ext]);
% 
% rootPath = '/';
% h5disp(fileLoad, rootPath, 'min');
% [data, desc] = HdfLoad(fileLoad, rootPath);

% fileSave = strrep(fileLoad, '.h5', '.mat');
% save(fileSave, 'data', 'desc');

%%

%% Check I/O Arguments
narginchk(1, 2);
if nargin < 2
    rootPath = [];
end

nargoutchk(0, 2);


%% Default Values and Constants
if isempty(rootPath), rootPath = '/'; end


%%
currInfo = h5info(fileLoad, rootPath); % returns information about the group, data set, or named datatype specified by location in the HDF5 file, filename.

% Recursively read Children Groups
numGroup = length(currInfo.Groups);
for iGroup = 1:numGroup
    groupPath = currInfo.Groups(iGroup).Name;
    [~, groupName] = fileparts(groupPath); % Use fileparts to pull the last portion of the string
    groupNameValid = matlab.lang.makeValidName(groupName); % Fix invalid Matlab names
    
    [data.(groupNameValid), desc.(groupNameValid)] = HdfLoad(fileLoad, groupPath);
end

% Recursively read the Current Level Datasets
numDataset = length(currInfo.Datasets);
for iDataset = 1:numDataset
    currDataset = currInfo.Datasets(iDataset);
    dataName = currDataset.Name;
    dataNameValid = matlab.lang.makeValidName(dataName); % Fix invalid Matlab names
    
    % Store the dataset info into a description structure
    desc.(dataNameValid) = currDataset;
    
    % Read the data from the file into the data structure
    pathDataset = [currInfo.Name, '/', dataName];
    data.(dataNameValid) = h5read(fileLoad, pathDataset);
    % data.(currNameValid) = h5read(filename, pathDataset, start, count, stride);
end

