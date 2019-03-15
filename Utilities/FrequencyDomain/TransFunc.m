function [freq, gain_dB, phase_deg, xyC, xyT, xxP, yyP, xyP] = TransFunc(x, y, freqVec, freqRate, winType, smoothFactor, coherLimit)
% Estimate the transfer function (y/x) between two time histories.
%
%Usage:  [freq, gain_dB, phase_deg, xyC, xyT, xxP, yyP, xyP] = TransFunc(x, y, freqVec, freqRate, winType, smoothFactor, coherLimit);
%
%Inputs:
% x            - assumed input time history
% y            - assumed output time history
% freqVec      - frequency range of interest [0, inf] (see Note)
% freqRate     - sample rate [1] (see Note)
% winType      - desired data window ['cosi']
% smoothFactor - moving average width [5]
% coherLimit   - coherence limit theshold []
%
%Outputs:
% freq      - frequency of transfer function (see Note)
% gain_dB   - magnitude of transfer function (db)
% phase_deg - phase angle of transfer function (deg)
% xyC       - magnitude squared coherence
% xyT       - complex transfer function
% xxP       - auto-spectral density of the input
% yyP       - auto-spectral density of the output
% xyP       - cross-spectral density of the ouput/input
%
%Notes:
% 'freqVec' and 'freqRate' must have the same units; 'freq' will have the
% units of the frequency inputs.
% The response can be optionally limited to a specified frequency range
% and/or coherence threshold.
%
%Dependency:
% GainPhase
% PsdEst
% TransFuncCmplx
% SmoothData
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(2, 7);
if nargin < 7, coherLimit = [];
    if nargin < 6, smoothFactor = []; end
    if nargin < 5, winType = []; end
    if nargin < 4, freqRate = []; end
    if nargin < 3, freqVec = []; end
end

nargoutchk(0, 8);


%% Default Values and Constants
if isempty(freqVec), freqVec = [0, Inf]; end
if isempty(freqRate), freqRate = 1; end
if isempty(winType), winType = 'cosi'; end
if isempty(smoothFactor), smoothFactor = 5; end


%% Check Inputs
[widthX, lenX] = size(x);
[widthY, lenY] = size(y);

% Transpose
if widthX > lenX
    transposeFlag = 1;
    x = x';
    y = y';
    [widthX, lenX] = size(x); % (v2.0)
    [widthY, lenY] = size(y); % (v2.0)
else
    transposeFlag = 0;
end

% Input lengths must be equal
if (lenX ~= lenY) % (v2.0)
    error('Input 1 and input 2 must be of equal length')
end



%% Approximate the transfer function response
% Compute the FFT and PSD for x and y
[xxP, xFft, freq] = PsdEst(x, freqRate, winType, smoothFactor);
[yyP, yFft] = PsdEst(y, freqRate, winType, smoothFactor);

lenFft = size(xFft, 2);

% Compute cross spectral density
xyP = (yFft .* repmat(conj(xFft),size(yFft, 1), 1)) * (2 / (lenFft * freqRate)); % (v1.8) (v2.0)

% Use only the first half of the PSD, see PsdEst.m
xyP = xyP(:, 2:lenFft/2);

% Smooth the PSDs, see PsdEst.m
if smoothFactor > 1
    xyP = SmoothSignal(xyP, [], smoothFactor);
end


%% Compute complex transfer function approximation
[xyT, xyC] = TransFuncCmplx(xxP, yyP, xyP);

% Compute gain and phase of the complex transfer function
[gain_dB, phase_deg] = GainPhase(xyT);


%% Frequency range limiting
% Find the index values for frequencies between [freqMin freqMax]
freqMin = min(freqVec);
freqMax = max(freqVec);
indxRange = find((freq >= freqMin) & (freq <= freqMax));

% Limit data to specified indices
if ~isempty(indxRange)
    freq = freq(:, indxRange);
    gain_dB = gain_dB(:, indxRange);
    phase_deg = phase_deg(:, indxRange);
    xyC = xyC(:, indxRange);
    xyT = xyT(:, indxRange);
    xxP = xxP(:, indxRange);
    yyP = yyP(:, indxRange);
    xyP = xyP(:, indxRange);
end


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
