function [xxP, xDft, freq] = SpectEst(x, freqRate, freqVec, winType, smoothFactor, dftType)
% Compute the Spectrum of time history data using FFT.
%
%Usage:  [xDft, freq] = SpectEst(x, freqRate);
%
%Inputs:
% x            - time history data
% freqRate     - sampling frequency of data (see Note)
% freqVec      - vector of frequencies (see Note)
% winType      - Window Type
% smoothFactor - Smoothing [1]
% dftType      - DFT Type ['FFT']
% scaleType    - Power Scale Type ['density']
%
%Outputs:
% xxP   - Spectrum Power (scaled) (mag)
% xDft  - result of the DFT
% freq  - frequency vector (see Note)
%
%Notes:
% The 'freq' output will have the units of the 'freqRate' input.

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 6);
if nargin < 2
    freqRate = [];
end

nargoutchk(0, 3);

%% Default Values and Constants
if isempty(freqRate), freqRate = 1; end
scaleType = 'density';

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

% Detrend and Window
x = detrend(x')';
win = WindowSignal(ones(size(x)), winType);
x = x .* win;

%% Compute Power scaling
scale = PowerScale(scaleType, freqRate, win')';

switch lower(dftType)
    case 'fft'
        %% Compute FFT of x
        [xDft, freq] = FFT(x, freqRate);
        
        %% Power
        xxP = scale * (xDft .* conj(xDft));
        
        % The Power should be doubled for a one-sided DFT, however if the Last point is an unpaired Nyquist freq point, don't double
        xxP = 2 * xxP;
        
        if mod(length(xxP), 2) == 1
            xxP(:, end) = 0.5 * xxP(:, end);
        end
        
        % If the signal was detrended the zero frequency component should be removed
        if ~isempty(detrendType)
            freq(:, end) = [];
            xDft(:, end) = [];
            xxP(:, end) = [];
        end
        
    case {'czt', 'chirpz'}
        [xDft, freq] = ChirpZ(x, freqRate, freqVec);
        
        % Compute Power, factor of 2 because CZT is one-sided
        xxP = 2*scale .* (xDft .* conj(xDft));
end

%% Check Outputs
% Transpose
if transposeFlag == 1
    xxP = xxP';
    xDft = xDft';
    freq = freq';
end

end %SpectEst


%% [Function] ChirpZ
function [xDft, freq] = ChirpZ(x, freqRate, freqVec)
% Compute the DFT of time history data with Chirp-Z.

% Min and Max Frequencies
freqMin = min(freqVec);
freqMax = max(freqVec);

% Number of frequency points
mChirp = length(freqVec);
freqStep = (freqMax - freqMin) / (mChirp - 1);
freq = (0 : mChirp-1) * freqStep + freqMin;

% Ratio between points
wChirp = exp(-1i * (2*pi / (mChirp - 1)) * (freqMax - freqMin) / freqRate);

% Starting point
aChirp = exp(1i * 2*pi * freqMin / freqRate);

% ChirpZ
xDft = czt(x', mChirp, wChirp, aChirp)';


end %ChirpZ


%% [Function] FFT
function [xDft, freq] = FFT(x, freqRate)
% Compute the DFT of time history data using FFT.

% Check Inputs
[~, lenX] = size(x);

% The output will be half the length of the input, input should be even length
lenDft = 2*floor(lenX/2);

% Compute FFT of x
xDft  = fft(xWin, lenDft, 2);

% Form Frequency Vector
freq = (freqRate/lenDft) * (1:length(xDft));

end % FFT
