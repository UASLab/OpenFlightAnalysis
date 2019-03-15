function [xyT, xyC] = TransFuncCmplx(xxP, yyP, xyP)
% Compute the complex transfer function and coherence.
%
%Usage:  [xyT, xyC] = TransFuncCmplx(xxP, yyP, xyP);
%
%Inputs:
% xxP - auto-spectral density of the input
% yyP - auto-spectral density of the output
% xyP - cross-spectral density of the ouput/input
%
%Outputs:
% xyT - complex transfer function
% xyC - magnitude square coherence
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(3, 3);
nargoutchk(0, 2);


%% Check Inputs
[widthXX, lenXX] = size(xxP);
[widthYY, lenYY] = size(yyP);
[widthXY, lenXY] = size(xyP);

% Input lengths must be equal
if (lenXX ~= lenYY) || (lenXX ~= lenXY) || (widthXY ~= widthYY)
    error('Inputs must be of equal dimension')
end

% Transpose
if widthXX > lenXX
    transposeFlag = 1;
    xxP = xxP';
    yyP = yyP';
    xyP = xyP';
    [widthXX, lenXX] = size(xxP);
    [widthYY, lenYY] = size(yyP);
    [widthXY, lenXY] = size(xyP);
else
    transposeFlag = 0;
end


%% Compute complex transfer function approximation
xyT = xyP ./ repmat(xxP, widthXY, 1);

% Compute magnitude squared coherence
xyC = (abs(xyP).^2)./(repmat(xxP, widthYY, 1) .* yyP);
%xyC = abs((xyT).*(xyP ./ yyP)); % equivalent to above


%% Check Outputs
% Transpose
if transposeFlag == 1
    xyT = xyT';
    xyC = xyC';
end
