function Hdf2mat(fileLoad, rootPath)
%% hdf2mat
% basePath = 'G:\Team Drives\UAVLab\Flight Data';
% ac = 'Thor';
% fltStr = 'FLT118';
% ext = '.h5';
% 
% fileLoad = fullfile(basePath, ac, [ac, fltStr], [ac, fltStr, ext]);
% 
% hdf2mat(fileLoad);


%%

%% Check I/O Arguments
narginchk(1, 2);
if nargin < 2
    rootPath = [];
end

nargoutchk(0, 0);


%% Default Values and Constants
if isempty(rootPath), rootPath = '/'; end


%%
[data, desc] = HdfLoad(fileLoad, rootPath);

fileSave = strrep(fileLoad, '.h5', '.mat');
save(fileSave, 'data', 'desc');


