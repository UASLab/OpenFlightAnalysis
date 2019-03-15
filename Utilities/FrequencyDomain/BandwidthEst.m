function [freqBW_rps, gainBW_dB, phaseBW_deg] = BandwidthEst(freq_rps, gain_dB, phase_deg)
% This function estimates the bandwidth of a transfer function.
%
%Usage:  [freqBW_rps, gainBW_dB, phaseBW_deg] = BandwidthEst(freq_rps, gain_dB, phase_deg);
%
%Inputs:
% freq_rps  - frequency of the transfer function (rad/sec)
% gain_dB   - magnitude of the transfer function (dB)
% phase_deg - phase angle of the transfer function (deg)
%
%Outputs:
% freqBW_rps  - estimated bandwidth frequency (rad/sec)
% gainBW_dB   - gain at the bandwidth frequency (dB)
% phaseBW_deg - phase at the bandwidth frequency (deg)
%
%Notes:
% The bandwidth is defined as the lesser of the frequency where the gain is
% 6DB higher than the gain where the phase angle = -180, and the frequency
% at which the phase angle is -135.
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(3, 3, nargin, 'struct'))
error(nargoutchk(0, 3, nargout, 'struct'))


%% Set default values and optional arguments
% Constants
r2d = 180/pi;
d2r = pi/180;


%% Check Inputs
% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Estimate the Bandwidth
% Calculate the frequency with gain 6dB higher than the gain at the -180 phase angle crossing
gainCross180 = interp1(phase_deg, gain_dB, -180);
freqGainCross6dB = interp1(gain_dB, freq_rps, gainCross180 + 6);

% Calculate the frequency at the -135 phase angle crossing
freqCross135 = interp1(phase_deg, freq_rps, -135);


if freqGainCross6dB > freqCross135
  freqBW_rps = freqCross135;
else
  freqBW_rps = freqGainCross6dB;
end

% Calculate the gain and phase at the bandwidth frequency
gainBW_dB = interp1(freq_rps, gain_dB, freqBW_rps);
phaseBW_deg = interp1(freq_rps, phase_deg, freqBW_rps);

%% Check Outputs
