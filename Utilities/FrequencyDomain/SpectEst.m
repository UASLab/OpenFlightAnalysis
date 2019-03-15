function [xxP, indxSegRef, freq] = SpectEst(x, lengthSeg, shiftSeg, freqVec, freqRate, winType, smoothFactor)
% Compute the Spectrogram of time history data.
%
%Usage:  [xxPsd, xComplex, freq] = SpectEst(x, lengthSeg, shiftSeg, freqVec, freqRate, winType, smoothFactor);
%
%Inputs:
% x            - time history data
% lengthSeg    - number of indices to use in a segment of time history data
% shiftSeg     - number of indices to shift between segments
% freqVec      - vector of frequencies (see Note)
% freqRate     - sampling frequency of data (see Note)
% winType      - desired data window ['rectwin']
% smoothFactor - moving average width [1]
%
%Outputs:
% xxP        - magnitude of the PSD
% indxSegRef - 
% freq       - frequency vector (see Note)
%
%Notes:
% This implimentation uses only the min and max values of the freqVec, then
% computes the frequency seperation, and starting point.
% 'freqVec' and 'freqRate' must have the same units; 'freq' will have the
% units of the frequency inputs.
% The total power under the PSD is not the correct, relative power distribution accross frequencies is accurate.
%
%Dependency:
% ChirpZEst
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release
%


%% Check I/O Arguments
narginchk(6, 7)
if nargin < 7, smoothFactor = [];
    if nargin < 6, winType = []; end
end

nargoutchk(0, 3)


%% Default Values and Constants
if isempty(winType), winType = 'cosi'; end
if isempty(smoothFactor), smoothFactor = 5; end


%% Check Inputs
[widthX, lenX] = size(x);

% Transpose
if widthX > lenX
    transposeFlag = 1;
    x = x';
    [widthX, lenX] = size(x);
else
    transposeFlag = 0;
end

%% Pack time history data into segments
numSeg = floor((lenX-lengthSeg+1)/shiftSeg);

[indxSegList1, indxSegList2] = ndgrid(shiftSeg.*[0:numSeg-1], 1:lengthSeg);
indxSegList = round(indxSegList1 + indxSegList2);

% Loop through segments
for indxSeg = 1:numSeg
    xSeg(indxSeg, :) = x(indxSegList(indxSeg, :));
    
    % Compute the Fourier transform for x
    [xxP(indxSeg, :), ~, freq] = ChirpZEst(xSeg(indxSeg, :), freqVec, freqRate, winType, smoothFactor);
end

indxSegRef = round(median(indxSegList, 2));


%% Check Outputs
% Transpose
if transposeFlag == 1
    freq = freq';
    xxP = xxP';
    indxSegRef = indxSegRef';
end


