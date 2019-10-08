function [frf] = FreqRespEst(x, y, optFrf)
% Estimate the transfer function (y/x) between two time histories.
%
%Usage:  [frf] = FreqRespEst(x, y, optFrf);
%
%Inputs:
% x            - assumed input time history
% y            - assumed output time history
% optFrf
%   optSpect
%     freqRate     - sample rate (see Note)
%     freq         - frequencies of interest (see Note)
%     optWin       - Window Options
%     optSmooth    - Smoothing Options
%
%Outputs:
% frf
%   freq      - frequency of transfer function (see Note)
%   T       - complex transfer function
%   cohor       - magnitude squared coherence
%   crossP       - cross-spectral Power of the ouput/input
%   inP       - input Power
%   outP       - output Power
%   inSpect
%   outSpect
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
    optFrf = struct();
end

nargoutchk(0, 1);

%% Default Values and Constants
if ~isfield(optFrf, 'dftType'), optFrf.dftType = []; end
if ~isfield(optFrf, 'freqRate'), optFrf.freqRate = []; end
if ~isfield(optFrf, 'scaleType'), optFrf.scaleType = []; end

if ~isfield(optFrf, 'optWin'), optFrf.optWin = struct(); end

if isempty(optFrf.freqRate), optFrf.freqRate = 1; end
if isempty(optFrf.scaleType), optFrf.scaleType = 'density'; end


%% Check Inputs


%% Approximate the transfer function response
frf.inSig = x;
frf.outSig = y;

% Compute the FFT and PSD for x and y
optSpectIn = optFrf; optSpectIn.freqIn = []; optSpectIn.freqOut = [];
optSpectOut = optFrf; optSpectOut.freqIn = []; optSpectOut.freqOut = [];


optSpectIn.freq = optFrf.freqIn;
[frf.inSpect] = SpectEst(frf.inSig, optSpectIn);

optSpectOut.freq = optFrf.freqOut;
[frf.outSpect] = SpectEst(frf.outSig, optSpectOut);


if all(frf.inSpect.freq == frf.inSpect.freq)
    frf.freq = frf.inSpect.freq;
end

frf.inP = frf.inSpect.P;
frf.outP = frf.outSpect.P;

% Compute cross spectral density, Scale is doubled because one-sided DFTs
frf.crossP = 2*frf.inSpect.scale * (frf.outSpect.dft .* repmat(conj(frf.inSpect.dft), size(frf.outSpect.dft, 1), 1));


%% Compute complex transfer function approximation
frf.T = FreqRespEstCmplx(frf.inP, frf.crossP);

% Smooth the PSDs
if isfield(optFrf, 'optSmooth')
    frf.crossPRaw = frf.crossP;
    frf.crossP = SmoothFunc(frf.crossPRaw, optFrf.optSmooth);
end

% Compute magnitude squared coherence
frf.coher = Coherence(frf.crossP, frf.inP, frf.outP);

