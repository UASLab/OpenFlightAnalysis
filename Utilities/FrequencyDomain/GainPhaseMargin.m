function [gainMargin_dB, phaseMargin_deg, freqGainCross_rps, freqPhaseCross_rps] = GainPhaseMargin(freq_rps, gain_dB, phase_deg, minFlag)
% Compute the Gain and Phase Margins from response data.
%
%Usage:  [gainMargin_dB, phaseMargin_deg, freqGainCross_rps, freqPhaseCross_rps] = GainPhaseMargin(freq_rps, gain_dB, phase_deg, minFlag);
%
%Inputs:
% freq_rps  - frequency (rad/sec)
% gain_dB   - transfer function gain (dB)
% phase_deg - transfer function phase (deg)
% minFlag   - flag to return only min margins ['min']
%
%Outputs:
% gainMargin_dB      - gain margin (dB)
% phaseMargin_deg    - phase margin (deg)
% freqGainCross_rps  - frequency at the gain margin (rad/sec)
% freqPhaseCross_rps - frequency at the phase margin (rad/sec)
%
%Notes:
% Linear interpolation is used despite the mismatch in scales between gain,
% phase, and the frequency vector.  This mismatch should only have a minor
% effect.
%

%Version History: Version 1.0
% 10/31/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 4, nargin, 'struct'))
if nargin < 4
    minFlag = [];
end

error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants
if isempty(minFlag), minFlag = 'min'; end

% Constants
r2d = 180/pi;
d2r = pi/180;


%% Check Inputs
% FIXIT

%% Unwrap the Phase Angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Gain Margins (-180 phase crossings)
% Find index prior to each -180 modulo 360 crossing [-180, 180)
phaseCrit_deg = 360 * floor((phase_deg(1:end-1) + 180)/360) - 180;
indxPhaseCross = find(phase_deg(2:end) < phaseCrit_deg | phase_deg(2:end) >= phaseCrit_deg + 360);

if isempty(indxPhaseCross) % Zero critical phase crossings
    freqGainCross_rps = NaN;
    gainMargin_dB = Inf;
else
    % Linearly interpolate the ratio to the critital phase crossing
    phaseCrossCrit_deg = phaseCrit_deg(indxPhaseCross) + 360*(phase_deg(indxPhaseCross+1) > phase_deg(indxPhaseCross));
    indxPhaseCrossFraction = (phaseCrossCrit_deg - phase_deg(indxPhaseCross)) ./ (phase_deg(indxPhaseCross+1) - phase_deg(indxPhaseCross));

    % Linearly interpolate frequency and gain at the phase crossings
    freqGainCross_rps = freq_rps(indxPhaseCross) + indxPhaseCrossFraction .* (freq_rps(indxPhaseCross+1)-freq_rps(indxPhaseCross));
    gainCross_dB = gain_dB(indxPhaseCross) + indxPhaseCrossFraction .* (gain_dB(indxPhaseCross+1) - gain_dB(indxPhaseCross));

    % Gain margins
    gainMargin_dB = 0 - gainCross_dB;
end


%% Phase Margins (0dB gain crossings)
% Find index prior to each 0 db crossing
indxGainCross = find(gain_dB(1:end-1) .* gain_dB(2:end) <= 0 & gain_dB(1:end-1) ~= gain_dB(2:end));

if isempty(indxGainCross) % Zero critical gain crossings
    freqPhaseCross_rps = NaN;
    phaseMargin_deg = Inf;
else
    % Linearly interpolate the ratio to 0dB
    indxGainCrossFraction = 0 - gain_dB(indxGainCross) ./ (gain_dB(indxGainCross+1) - gain_dB(indxGainCross));

    % Linearly interpolate frequency and phase at gain phase crossings
    freqPhaseCross_rps = freq_rps(indxGainCross) + indxGainCrossFraction .* (freq_rps(indxGainCross+1) - freq_rps(indxGainCross));
    phaseCross_deg = phase_deg(indxGainCross) + indxGainCrossFraction .* (phase_deg(indxGainCross+1) - phase_deg(indxGainCross));
    phaseCross_deg = mod(phaseCross_deg, 360);

    % Phase margins
    phaseMargin_deg = phaseCross_deg - 180;
end


%% Return all the margins or only the minimum
switch minFlag
    case 'min' % Return the min Margins
        [junk, indxGainMin] = min(abs(gainMargin_dB));
        freqGainCross_rps = freqGainCross_rps(indxGainMin);
        gainMargin_dB = gainMargin_dB(indxGainMin);

        [junk, indxPhaseMin] = min(abs(phaseMargin_deg));
        freqPhaseCross_rps = freqPhaseCross_rps(indxPhaseMin);
        phaseMargin_deg = phaseMargin_deg(indxPhaseMin);
    otherwise % Return all the Margins, in order of crossing frequency
        [freqGainCross_rps, indxSort] = sort(freqGainCross_rps);
        gainMargin_dB = gainMargin_dB(indxSort);

        [freqPhaseCross_rps, indxSort] = sort(freqPhaseCross_rps);
        phaseMargin_deg = phaseMargin_deg(indxSort);
end


%% Check Outputs
% FIXIT
