function [oDataSlice] = OpenData_Slice(oData, sigCond, sigRange)
% Slice Open Data format
%
%Inputs:
% oData   - Open Data structured data
% sigCond - data used to slice
% sigRange - conditions for slicing, [min, max]
%
%Outputs:
% oDataSlice - Sliced Open Data structured data
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(3, 3);
nargoutchk(1, 1);

%%
indx = (sigCond >= min(sigRange)) & (sigCond <= max(sigRange));

fieldList = fieldnames(oData);
for iField = 1:length(fieldList)
    fieldName = fieldList{iField};
    if isstruct(oData.(fieldName))
        oDataSlice.(fieldName) = OpenData_Slice(oData.(fieldName), sigCond, sigRange)
    else
        oDataSlice.(fieldName) = oData.(fieldName)(indx);
    end
end

