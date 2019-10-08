function [xyT] = FreqRespEstCmplx(xxP, xyP)
% Compute the complex transfer function and coherence.
%
%Usage:  [xyT, xyC] = FreqRespEstCmplx(xxP, xyP);
%
%Inputs:
% xxP - auto-spectral of the input
% xyP - cross-spectral of the ouput/input
%
%Outputs:
% xyT - complex transfer function
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(2, 2);
nargoutchk(0, 1);


%% Check Inputs
[~, lenXX] = size(xxP);
[widthXY, lenXY] = size(xyP);

% Input lengths must be equal
if (lenXX ~= lenXY)
    error([mfilename ' - Inputs must be of equal dimension'])
end


%% Compute complex transfer function approximation
xyT = xyP ./ repmat(xxP, widthXY, 1);

