function [freq, xyT, xyC, xxP, yyP, xyP] = FreqRespEst(x, y, optSpect)
% Estimate the transfer function (y/x) between two time histories.
%
%Usage:  [freq, xyT, xyC, xxP, yyP, xyP] = FreqRespEst(x, y, optSpect);
%
%Inputs:
% x            - assumed input time history
% y            - assumed output time history
% optSpect
%   freqRate     - sample rate (see Note)
%   freq         - frequencies of interest (see Note)
%   winType      - desired data window ['rectwin']
%   smoothFactor - moving average width [1]
%   coherLimit   - coherence limit theshold []
%
%Outputs:
% freq      - frequency of transfer function (see Note)
% xyT       - complex transfer function
% xyC       - magnitude squared coherence
% xxP       - auto-spectral density of the input
% yyP       - auto-spectral density of the output
% xyP       - cross-spectral density of the ouput/input
%
%Notes:
% 'freq' and 'freqRate' must have the same units; 'freq' will have the
% units of the frequency inputs.
% The response can be optionally limited to a specified frequency range
% and/or coherence threshold.
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(2, 3);
if nargin < 3
    optSpect = struct();
end

nargoutchk(0, 6);

%% Default Values and Constants
if ~isfield(optSpect, 'dftType'), optSpect.dftType = []; end
if ~isfield(optSpect, 'freqRate'), optSpect.freqRate = []; end
if ~isfield(optSpect, 'scaleType'), optSpect.scaleType = []; end

if ~isfield(optSpect, 'optWin')
    optSpect.optWin = struct();
end
if ~isfield(optSpect.optWin, 'len'), optSpect.optWin.len = []; end

if ~isfield(optSpect, 'optSmooth')
    optSpect.optSmooth = struct();
end
if ~isfield(optSpect.optSmooth, 'type'), optSpect.optSmooth.type = []; end
if ~isfield(optSpect.optSmooth, 'len'), optSpect.optSmooth.len = []; end


if isempty(optSpect.freqRate), optSpect.freqRate = 1; end
if isempty(optSpect.scaleType), optSpect.scaleType = 'density'; end
if isempty(optSpect.optWin.len), optSpect.optWin.len = length(x); end
if isempty(optSpect.optSmooth.type), optSpect.optSmooth.type = 'rect'; end
if isempty(optSpect.optSmooth.len), optSpect.optSmooth.len = 5; end


%% Check Inputs
% Input lengths must be equal
if (length(x) ~= length(y)) % (v2.0)
    error([mfilename ' - Input 1 and input 2 must be of equal length.'])
end


%% Approximate the transfer function response
% Compute the FFT and PSD for x and y
[xxP, xDft, freq, powerScale] = SpectEst(x, optSpect);
[yyP, yDft] = SpectEst(y, optSpect);

% Smooth the PSDs, see SpectEstFFT.m
xxP = SmoothFunc(xxP, optSpect.optSmooth);
yyP = SmoothFunc(yyP, optSpect.optSmooth);

% Compute cross spectral density, Scale is doubled because one-sided DFTs
xyP = 2*powerScale * (yDft .* repmat(conj(xDft), size(yDft, 1), 1));

% Smooth the PSDs, see SpectEstFFT.m
xyP_smooth = SmoothFunc(xyP, optSpect.optSmooth);


%% Compute complex transfer function approximation
xyT = xyP ./ xxP;

% Compute magnitude squared coherence
xyC = abs(xyP_smooth).^2 ./ (xxP .* yyP);
%xyC = abs((xyT).*(xyP_smooth ./ yyP)); % equivalent to above


%% Check Outputs

