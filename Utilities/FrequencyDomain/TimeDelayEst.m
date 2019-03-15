function [tDelay_s] = TimeDelayEst(freq_rps, phase_deg)
% This function estimates the equivalent time delay based on phase.
%
%Usage:  [tDelay_s] = TimeDelayEst(freq_rps, phase_deg)
%
%Inputs:
% freq_rps  - frequency of the transfer function (rad/sec)
% phase_deg - phase angle of the transfer function (deg)
%
%Output:
% tDelay_s - estimated equivalent time delay (sec)
%
%Notes:
% 
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release
%


%% Check I/O Arguments
narginchk(2, 2)
nargoutchk(0, 1)


%% Default Values and Constants
r2d = 180/pi;
d2r = pi/180;


%% Check Inputs
% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Compute the Estimated Equivalent Time Delay
% Calculate the phase angle at twice the frequency of the -180 degree phase
% crossing
freqCross180 = interp1(phase_deg, freq_rps, -180);
phaseTwiceFreqCross180 = interp1(freq_rps, phase_deg, 2 * freqCross180);

% Caclulate the estimated equivalent time delay
tDelay_s = -(phaseTwiceFreqCross180 + 180) / (2 * (freqCross180 * r2d));


%% Check Outputs
