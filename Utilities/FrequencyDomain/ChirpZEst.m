function [xxPsd, xChirp, freq] = ChirpZEst(x, freqVec, freqRate, winType, smoothFactor)
% Compute the ChirpZ of time history data.
%
%Inputs:
% x            - time history data
% freqVec      - vector of frequencies (see Note)
% freqRate     - sampling frequency of data (see Note)
% winType      - desired data window ['rectwin']
% smoothFactor - moving average width [1]
%
%Outputs:
% xxPsd    - magnitude of the PSD
% xChirp   - result of the ChirpZ
% freq     - frequency vector (see Note)
%
%Notes:
% This implimentation uses only the min and max values of the freqVec, then
% computes the frequency seperation, and starting point.
% 'freqVec' and 'freqRate' must have the same units; 'freq' will have the
% units of the frequency inputs.
% The total power under the PSD is not the correct, relative power distribution accross frequencies is accurate.
%
%Dependency:
% SmoothSignal
% WindowSignal
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release
%


%% Check I/O Arguments
narginchk(2, 5);
if nargin < 5, smoothFactor = [];
    if nargin < 4, winType = []; end
    if nargin < 3, freqRate = []; end
end

nargoutchk(0, 3);


%% Default Values and Constants
if isempty(freqRate), freqRate = 1; end
if isempty(winType), winType = 'rectwin'; end
if isempty(smoothFactor), smoothFactor = 1; end


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


%% Remove Trend and Window Data
% The output will be half the length of the input, input should be even length
lenChirp = 2*floor(lenX/2);

% Remove linear trend from the data
xDetrend = detrend(x(:, 1:lenChirp)')';

% Window the data
xWin = WindowSignal(xDetrend, winType);


%% Compute ChirpZ of x
% Min and Max Frequencies
freqMin = min(freqVec);
freqMax = max(freqVec);

% Number of frequency points
mChirp = length(freqVec);
freqStep = (freqMax - freqMin) / (mChirp - 1);

% Ratio between points
wChirp = exp(-1i * (2*pi / (mChirp - 1)) * (freqMax - freqMin) / freqRate);

% Starting point
aChirp = exp(1i * 2*pi * freqMin / freqRate);

% ChirpZ
xChirp = czt(xWin', mChirp, wChirp, aChirp)';
%xChirp = ChirpZ(xWin, mChirp, wChirp, aChirp); %much slower


%% Compute power spectral density (PSD)
xxPsd = (xChirp .* conj(xChirp)) / (mChirp * freqRate);


%% Smooth the PSD
if smoothFactor > 1
    xxPsd = SmoothSignal(xxPsd, [], smoothFactor);
end


%% Form Frequency Vector
freq = (0 : mChirp-1) * freqStep + freqMin;


%% Check Outputs
% Transpose
if transposeFlag == 1
    xxPsd = xxPsd';
    xChirp = xChirp';
    freq = freq';
end


%% [function] ChirpZ
function [z] = ChirpZ(x, mChirp, wChirp, aChirp)
% Compute ChirpZ.
%
%Usage:  ;
%
%Inputs:
%
%Outputs:
%
%Notes:
% Created from SIDPAC v2, function 'chirpz.m'


%% Check I/O Arguments
narginchk(4, 4)
nargoutchk(0, 1)


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


%% Compute the ChirpZ
nVec = -(0 : lenX - 1)';
kVec = -(0 : mChirp - 1);
nk = nVec * kVec;

z = zeros(mChirp, 1);
z = (wChirp .^ nk).'*((aChirp .^ nVec) .* x');


%% Check Outputs
% Transpose
if transposeFlag == 1
    z = z';
end
