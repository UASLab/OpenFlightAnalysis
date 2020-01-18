function [xyC] = Coherence(xyP, xxP, yyP)
% Compute the complex transfer function and coherence.
%
%Usage:  [xyT, xyC] = FreqRespEstCmplx(xxP, xyP);
%
%Inputs:
% xyP - cross-spectral of the ouput/input
% xxP - auto-spectral of the input
% yyP - auto-spectral of the output
%
%Outputs:
% xyC - coherence
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(3, 3);
nargoutchk(0, 1);


%% Check Inputs
[widthXY, lenXY] = size(xyP);
[widthXX, lenXX] = size(xxP);
[widthYY, lenYY] = size(yyP);

% Input lengths must be equal
if (lenXX ~= lenXY)
    if (widthXX == lenXY)
        xxP = xxP';
    else
        error([mfilename ' - Inputs must be of equal dimension'])
    end
end

if (lenYY ~= lenXY)
    if (widthYY == lenXY)
        yyP = yyP';
    else
        error([mfilename ' - Inputs must be of equal dimension'])
    end
end

xyC = abs(xyP).^2 ./ (xxP .* yyP);

xyC(xyC < 0) = 0.0;
xyC(xyC > 1) = 1.0;