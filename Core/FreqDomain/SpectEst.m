function [Spect] = SpectEst(x, Spect)
% Compute the Spectrum of time history data using FFT.
%
%Usage:  [Spect] = SpectEst(x, Spect);
%
%Inputs:
% x            - time history data
% Spect.Opt
%   DftType      - DFT Type ['FFT']
%   ScaleType    - Power Scale Type ['Spectrum']
%   Ts           - sampling time of data (see Note)
%   Frequency    - vector of frequencies (see Note)
%   winType      - Window Type
%
%Outputs:
% Spect
%   Signal - original time history signal
%   Window    - signal after detrend and window
%   Power      - Spectrum Power (scaled) (mag)
%   DFT    - result of the DFT
%   Frequency   - frequency vector (see Note)
%   Scale  - scaling value
%
%Notes:
% The 'freq' output will have the units of the 2*pi/'Ts' input.
% So, if Ts is in seconds then 1/sec = Hz, 2*pi/Ts is rad/s

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 2);
if nargin < 2
    Spect.Opt = struct();
end

nargoutchk(0, 1);

%% Default Values and Constants
if ~isfield(Spect.Opt, 'DftType'), Spect.Opt.DftType = []; end
if ~isfield(Spect.Opt, 'Ts'), Spect.Opt.Ts = []; end
if ~isfield(Spect.Opt, 'ScaleType'), Spect.Opt.ScaleType = []; end

if ~isfield(Spect.Opt, 'Window'), Spect.Opt.Window = struct(); end

if isempty(Spect.Opt.Ts), Spect.Opt.Ts = 1; end
if isempty(Spect.Opt.ScaleType), Spect.Opt.ScaleType = 'Spectrum'; end

%% Check Inputs
Spect.Signal = x;

% Detrend and Window
Spect.Opt.Window.Length = length(Spect.Signal);
Spect.Window = WindowFunc(Spect.Opt.Window);
Spect.SignalWin = detrend(Spect.Signal')' .* Spect.Window;

%% Compute Power scaling
Spect.Scale = PowerScale(Spect.Opt.ScaleType, 2*pi/Spect.Opt.Ts, Spect.Window);

switch lower(Spect.Opt.DftType)
    case 'fft'
        %% Compute FFT of x
        [Spect.DFT, Spect.Frequency] = FFT(Spect.SignalWin, 2*pi/Spect.Opt.Ts);
        
        %% Power
        Spect.Power = Spect.Scale * (Spect.DFT .* conj(Spect.DFT));
        
        % The Power should be doubled for a one-sided DFT, however if the Last point is an unpaired Nyquist freq point, don't double
        Spect.Power = 2 * Spect.Power;
        
        if mod(length(Spect.Power), 2) == 1
            Spect.Power(:, end) = 0.5 * Spect.Power(:, end);
        end
        
        % If the signal was detrended the zero frequency component should be removed
        if 1
            Spect.Frequency(:, end) = [];
            Spect.DFT(:, end) = [];
            Spect.Power(:, end) = [];
        end
        
    case {'czt', 'chirpz'}
        [Spect.DFT, Spect.Frequency] = ChirpZ(Spect.SignalWin, 2*pi/Spect.Opt.Ts, Spect.Opt.Frequency);

        % Compute Power, factor of 2 because CZT is one-sided
        Spect.Power = 2*Spect.Scale .* (Spect.DFT .* conj(Spect.DFT));
end

% Smooth the Power Output
if isfield(Spect.Opt, 'Smooth')
    Spect.PowerRaw = Spect.Power;
    Spect.Power = SmoothFunc(Spect.PowerRaw, Spect.Opt.Smooth);
end
