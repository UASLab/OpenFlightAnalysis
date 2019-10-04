function [xyT, xyC] = FreqRespEstCmplx(xxP, yyP, xyP)
% Compute the complex transfer function and coherence.
%
%Usage:  [xyT, xyC] = FreqRespEstCmplx(xxP, yyP, xyP);
%
%Inputs:
% xxP - auto-spectral of the input
% yyP - auto-spectral of the output
% xyP - cross-spectral of the ouput/input
%
%Outputs:
% xyT - complex transfer function
% xyC - magnitude square coherence
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(3, 3);
nargoutchk(0, 2);


%% Check Inputs
[~, lenXX] = size(xxP);
[widthYY, lenYY] = size(yyP);
[widthXY, lenXY] = size(xyP);

% Input lengths must be equal
if (lenXX ~= lenYY) || (lenXX ~= lenXY) || (widthXY ~= widthYY)
    error([mfilename ' - Inputs must be of equal dimension'])
end


%% Compute complex transfer function approximation
xyT = xyP ./ repmat(xxP, widthXY, 1);

% Compute magnitude squared coherence
xyC = (abs(xyP).^2)./(repmat(xxP, widthYY, 1) .* yyP);
%xyC = abs((xyT).*(xyP ./ yyP)); % equivalent to above


%% Check Outputs
