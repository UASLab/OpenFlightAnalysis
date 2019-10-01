function [xxP, xDft, freq, scale] = SpectEst(x, optSpect)
% Compute the Spectrum of time history data using FFT.
%
%Usage:  [xxP, xDft, freq, scale] = SpectEst(x, optSpect);
%
%Inputs:
% x            - time history data
% optSpect
%   freqRate     - sampling frequency of data (see Note)
%   freq         - vector of frequencies (see Note)
%   winType      - Window Type
%   smoothFactor - Smoothing [1]
%   dftType      - DFT Type ['FFT']
%   scaleType    - Power Scale Type ['density']
%
%Outputs:
% xxP   - Spectrum Power (scaled) (mag)
% xDft  - result of the DFT
% freq  - frequency vector (see Note)
% scale - scaling value
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
narginchk(1, 2);
if nargin < 2
    optSpect = struct();
end

nargoutchk(0, 4);

%% Default Values and Constants
if ~isfield(optSpect, 'dftType'), optSpect.dftType = []; end
if ~isfield(optSpect, 'freqRate'), optSpect.freqRate = []; end
if ~isfield(optSpect, 'scaleType'), optSpect.scaleType = []; end
if ~isfield(optSpect, 'optWin')
    optSpect.optWin = struct();
end
if ~isfield(optSpect.optWin, 'len'), optSpect.optWin.len = []; end
if ~isfield(optSpect, 'smoothType'), optSpect.smoothType = []; end

if isempty(optSpect.freqRate), optSpect.freqRate = 1; end
if isempty(optSpect.scaleType), optSpect.scaleType = 'density'; end
if isempty(optSpect.optWin.len), optSpect.optWin.len = length(x); end


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
x = detrend(x);

win = WindowFunc(optSpect.optWin);
xWin = x .* win;

%% Compute Power scaling
scale = PowerScale(optSpect.scaleType, optSpect.freqRate, win);

switch lower(optSpect.dftType)
    case 'fft'
        %% Compute FFT of x
        [xDft, freq] = FFT(xWin, optSpect.freqRate);
        
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
        [xDft, freq] = ChirpZ(xWin, optSpect.freqRate, optSpect.freq);
        
        % Compute Power, factor of 2 because CZT is one-sided
        xxP = 2*scale .* (xDft .* conj(xDft));
end



end %SpectEst


%% [Function] PowerScale
function [scale] = PowerScale(scaleType, freqRate, win)
    % Compute the scaling for power
    switch lower(scaleType)
        case 'density'
            scale = 1.0 / (freqRate * sum(win.^2));
        case 'spectrum'
            scale = 1.0 / sum(win).^2;
        otherwise
            scale = 1;
            warning([mfilename ' - Unknown Power Scaling.']);
    end
end

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
