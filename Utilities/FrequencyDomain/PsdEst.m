function [xxPsd, xFft, freq] = PsdEst(x, freqRate, winType, smoothFactor)
% Compute the Pseudo-Spectral-Density of time history data.
%
%Usage:  [xxPsd, xFft, freq] = PsdEst(x, freqRate, winType, smoothFactor);
%
%Inputs:
% x            - time history data
% freqRate     - sampling frequency of data (see Note)
% winType      - desired data window ['rectwin']
% smoothFactor - moving average width [5]
%
%Outputs:
% xxPsd - magnitude of the PSD (auto-specral density)
% xFft  - result of the FFT
% freq  - frequency vector (see Note)
%
%Notes:
% The 'freq' output will have the units of the 'freqRate' input.
% The total power under the PSD is not the correct, relative power distribution accross frequencies is accurate.
%
%Dependency:
% SmoothSignal
% WindowSignal
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 4);
if nargin < 4, smoothFactor = [];
    if nargin < 3, winType = []; end
    if nargin < 2, freqRate = []; end
end

nargoutchk(0, 3);


%% Default Values and Constants
if isempty(freqRate), freqRate = 1; end
if isempty(winType), winType = 'rectwin'; end
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


%% Remove Trend and Window Data
% The output will be half the length of the input, input should be even length
lenFft = 2*floor(lenX/2);

% Remove linear trend from the data
xDetrend = detrend(x(:, 1:lenFft)')';

% Window the data
xWin = WindowSignal(xDetrend, winType);


%% Compute FFT of x
xFft  = fft(xWin, lenFft, 2);


%% Compute power spectral density (PSD), and scale power
xxPsd = (xFft .* conj(xFft)) * (2 / (lenFft * freqRate));

% Use only the first half of the PSD
xxPsd = xxPsd(:, 2:lenFft/2);


%% Smooth the PSD
if smoothFactor > 1
    xxPsd = SmoothSignal(xxPsd, [], smoothFactor);
end


%% Form Frequency Vector
lenPsd = length(xxPsd);
freq = (freqRate/lenFft) * (1:lenPsd);


%% Check Outputs
% Transpose
if transposeFlag == 1
    xxPsd = xxPsd';
    xFft = xFft';
    freq = freq';
end
