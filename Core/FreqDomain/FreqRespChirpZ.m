function [freq, gain_dB, phase_deg, xyC, xyT, xxP, yyP, xyP] = FreqRespEstChirpZ(x, y, freqVec, freqRate, winType, smoothFactor, coherLimit)
% Estimate the frequency response function (x -> y) between two time histories using the
% Chirp-z transform.
%
% Inputs:
%  x            - assumed input time history
%  y            - assumed output time history
%  freqVec      - frequencies of interest (see Note)
%  freqRate     - sample rate (see Note)
%  winType      - desired data window ['rectwin']
%  smoothFactor - moving average width [1]
%  coherLimit   - coherence limit theshold []
%
% Outputs:
%  freq      - frequency of transfer function (see Note)
%  gain_dB   - magnitude of transfer function (db)
%  phase_deg - phase angle of transfer function (deg)
%  xyC       - magnitude squared coherence
%  xyT       - complex transfer function
%  xxP       - psd of the input
%  yyP       - psd of the output
%  xyP       - psd of the ouput/input
%
% Notes:
%  'freqVec' and 'freqRate' must have the same units; 'freq' will have the
%  units of the frequency inputs.
%  The response can be optionally limited to a specified coherence
%  threshold.
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(4, 7)
if nargin < 7, coherLimit = [];
    if nargin < 6, smoothFactor = []; end
    if nargin < 5, winType = []; end
end

nargoutchk(0, 8)


%% Default Values and Constants
if isempty(winType), winType = 'rectwin'; end
if isempty(smoothFactor), smoothFactor = 1; end
if isempty(freqRate), freqRate = 1; end


%% Check Inputs
[widthX, lenX] = size(x);
[widthY, lenY] = size(y);

% Transpose
if widthX > lenX
    transposeFlag = 1;
    x = x';
    y = y';
    %[widthX, lenX] = size(x);
    %[widthY, lenY] = size(y);
else
    transposeFlag = 0;
end


%% Approximate the transfer function response
% Compute the ChirpZ and PSD for x and y
[xxP, xDft, freq] = SpectEstCZT(x, freqVec, freqRate, winType, smoothFactor);
[yyP, yChirp] = SpectEstCZT(y, freqVec, freqRate, winType, smoothFactor);

lenChirp = length(xDft);

% Compute cross-term
xyP = (yChirp .* conj(xDft)) ./ (lenChirp * freqRate); %(v1.1)

% Smooth the PSDs, see SpectEstCZT.m
if smoothFactor > 1
    xyP = SmoothSignal(xyP, [], smoothFactor);
end


%% Compute complex transfer function approximation
[xyT, xyC] = FreqRespEstCmplx(xxP, yyP, xyP);

% Compute gain and phase of the complex transfer function
[gain_dB, phase_deg] = GainPhase(xyT);


%% Coherence limiting
if ~isempty(coherLimit)
    % Find the index values for for coherence less than threshold (eliminate spikes)
    indxCoh = find(xyC >= coherLimit);

    % Limit data to specified indices
    freq = freq(:, indxCoh);
    gain_dB = gain_dB(:, indxCoh);
    phase_deg = phase_deg(:, indxCoh);
    xyC = xyC(:, indxCoh);
    xyT = xyT(:, indxCoh);
    xxP = xxP(:, indxCoh);
    yyP = yyP(:, indxCoh);
    xyP = xyP(:, indxCoh);
end


%% Check Outputs
% Transpose
if transposeFlag == 1
    freq = freq';
    gain_dB = gain_dB';
    phase_deg = phase_deg';
    xyC = xyC';
    xyT = xyT';
    xxP = xxP';
    yyP = yyP';
    xyP = xyP';
end