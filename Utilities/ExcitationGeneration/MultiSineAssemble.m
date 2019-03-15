function [signals, signalComp] = MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW)
% Assemble multi-sine composite signals.
%
%Usage:  [signals, signalComp] = MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW);
%
%Inputs:
% freqComp_rps   - frequency components of the signals (rad/s)
% phaseComp_rad  - phase components of the signals (rad)
% signalPowerRel - relative power components of the signals
% time_s         - time vector for time history (s)
% signalDist     - component distribution of the signals [ones]
% normalSW       - switch to normalize the signals to [-1 1]
%
%Outputs:
% signals    - time history of the composite signals
% signalComp - time history of all signal components
%
%Notes:
% Signal power is a function of frequency and length of time: FIXME:
%   signalPower = 0.5 .* amplComp.^2 .* (freqComp_rps*timeEnd_s + sin(freqComp_rps*timeEnd_s).*cos(freqComp_rps*timeEnd_s));
%   signalPower = 2 .* amplComp.^2
%

%Version History: Version 1.3
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
% 02/05/2006  C. Regan     Fixed warning message, replace input amplComp with signalPowerRel (v1.2)
% 02/07/2006  C. Regan     Fixed it so that normalization doesn't alter the relative power of each channel (v1.3)
%


%% Check I/O Arguments
error(nargchk(4, 6, nargin, 'struct'))
if nargin < 6, normalSW = [];
    if nargin < 5, signalDist = []; end
end

error(nargoutchk(0, 2, nargout, 'struct'))


%% Default Values and Constants
if isempty(signalDist), signalDist = ones(size(freqComp_rps)); end


%% Check Inputs
if (length(freqComp_rps) ~= length(phaseComp_rad)) || (length(freqComp_rps) ~= length(signalPowerRel))
    warning(['AVFTT:' mfilename ':Inputs'], 'Inputs for signal power, frequency, and phase must have the same length');
end


%% Generate the composite signals
[numComp, numSignals] = size(signalDist);

% Compute component amplitudes
amplComp = sqrt(0.5 .* signalPowerRel);

% Generate each signal component
signalComp = zeros(numComp, length(time_s));
for i = 1:numComp
    signalComp(i, :) = amplComp(i) .* cos(freqComp_rps(i).*time_s + phaseComp_rad(i));
end

% Combine signal components into signals and normalize
signals = zeros(numSignals, length(time_s));
for indxSignal = 1:numSignals
    listSignal = find(signalDist(:, indxSignal));
    signals(indxSignal, :) = sum(signalComp(listSignal, :), 1);
end

% Normalize signals
if normalSW
    signals = signals ./ max(max(abs(signals)));
end

%% Check Outputs
