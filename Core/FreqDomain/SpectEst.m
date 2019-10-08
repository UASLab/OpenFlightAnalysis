function [spect] = SpectEst(x, optSpect)
% Compute the Spectrum of time history data using FFT.
%
%Usage:  [spect] = SpectEst(x, optSpect);
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
% spect
%   signal - original time history signal
%   win    - signal after detrend and window
%   P      - Spectrum Power (scaled) (mag)
%   dft    - result of the DFT
%   freq   - frequency vector (see Note)
%   scale  - scaling value
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

nargoutchk(0, 1);

%% Default Values and Constants
if ~isfield(optSpect, 'dftType'), optSpect.dftType = []; end
if ~isfield(optSpect, 'freqRate'), optSpect.freqRate = []; end
if ~isfield(optSpect, 'scaleType'), optSpect.scaleType = []; end

if ~isfield(optSpect, 'optWin'), optSpect.optWin = struct(); end

if isempty(optSpect.freqRate), optSpect.freqRate = 1; end
if isempty(optSpect.scaleType), optSpect.scaleType = 'density'; end

%% Check Inputs
spect.signal = x;

% Detrend and Window
optSpect.optWin.len = length(spect.signal);
win = WindowFunc(optSpect.optWin);
spect.win = detrend(spect.signal) .* win;

%% Compute Power scaling
spect.scale = PowerScale(optSpect.scaleType, optSpect.freqRate, win);

switch lower(optSpect.dftType)
    case 'fft'
        %% Compute FFT of x
        [spect.dft, spect.freq] = FFT(spect.win, optSpect.freqRate);
        
        %% Power
        spect.P = spect.scale * (spect.dft .* conj(spect.dft));
        
        % The Power should be doubled for a one-sided DFT, however if the Last point is an unpaired Nyquist freq point, don't double
        spect.P = 2 * spect.P;
        
        if mod(length(spect.P), 2) == 1
            spect.P(:, end) = 0.5 * spect.P(:, end);
        end
        
        % If the signal was detrended the zero frequency component should be removed
        if ~isempty(detrendType)
            spect.freq(:, end) = [];
            spect.dft(:, end) = [];
            spect.P(:, end) = [];
        end
        
    case {'czt', 'chirpz'}
        [spect.dft, spect.freq] = ChirpZ(spect.win, optSpect.freqRate, optSpect.freq);

        % Compute Power, factor of 2 because CZT is one-sided
        spect.P = 2*spect.scale .* (spect.dft .* conj(spect.dft));
end

        
% Smooth the Power Output
if isfield(optSpect, 'optSmooth')
    spect.PRaw = spect.P;
    spect.P = SmoothFunc(spect.PRaw, optSpect.optSmooth);
end



