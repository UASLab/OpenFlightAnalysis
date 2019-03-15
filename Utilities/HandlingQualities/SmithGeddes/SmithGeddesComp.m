function [freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, phase2BW_deg, gainFitPoints] = SmithGeddesComp(freq_rps, gain_dB, phase_deg, phase2_deg, typeFR)
% Computes the Ralph Smith criteria of PIO potential. 
%
%Usage:  [freqPilotBW_rps, phasePilotBW_deg, gainSlope_db_oct, phase2BW_deg, gainFitPoints] = SmithGeddesComp(freq_rps, gain_dB, phase_deg, phase2_deg, typeFR);
%
%Inputs:
% freq_rps   - frequency of the frequency response (rad/sec)
% gain_dB    - magnitude of the frequency response (dB)
% phase_deg  - phase of frequency responce (deg)
% phase2_deg - phase of nz/dep (ny/dap) frequency responce (deg) []
% typeFR     - axis of interest: 'long' or 'latdir' ['long']
%
%Outputs:
% freqPilotBW_rps  - pilot bandwidth frequency (rad/sec)
% phasePilotBW_deg - phase at pilot bandwidth frequency (deg)
% gainSlope_db_oct - slope of the gain between 1 and 6 rps (dB/octave)
% phase2BW_deg     - bandwidth phase of nz/dep (ny/dap) FR (deg) []
% gainFitPoints    - start and end point of the gain fit (rps,dB)
%
%Notes: FIXME: Improve readability, references
% H.Q. from:
%   pitch attitude to pitch stick force
%     and
%   normal acceleration at the pilot station to pitch stick force 
%   transfer functions,
%     or from:
%   roll attitude to roll stick transfer function.
% Input frequency response is for theta/pitchStick or phi/rollStick.
% The optional phase frequency response input is normalLoad/pitchStick
%   the normal load is at the pilot station (g).
%
%Reference:
% AFWAL-TR-81-3090
% 1993 SAE presentation
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 5, nargin, 'struct'))
if nargin < 5, typeFR = 'long';
    if nargin < 4, phase2_deg = []; end
end

error(nargoutchk(0, 5, nargout, 'struct'))


%% Default Values and Constants
if isempty(typeFR), typeFR = 'long'; end

r2d = 180/pi;
d2r = pi/180;
imaj = sqrt(-1);


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);
phase2_deg = phase2_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Calculate the pilot bandwidth phase criterion
% first-order fit of gain
gainFitFreqLow = 1; % lower frequency for fit [1]
gainFitFreqUp = 6;  % upper frequency for fit [1]
indxFreqRange = find((freq_rps > gainFitFreqLow) & (freq_rps < gainFitFreqUp));
gainSlopeFit = polyfit(log10(freq_rps(indxFreqRange)), gain_dB(indxFreqRange), 1);

% Gain slope in dB/octave (convert from dB/decade)
gainSlope_db_oct = gainSlopeFit(1) * log10(2);

% Define the line for the fit
gainFitPoints(:,1) = [gainFitFreqLow gainFitFreqUp];
gainFitPoints(:,2) = polyval(gainSlopeFit, log10(gainFitPoints(:,1)));

% Compute pilot bandwidth frequency at phase
freqPilotBW_rps = 6 + 0.24*gainSlope_db_oct;
phasePilotBW_deg = interp1(freq_rps, phase_deg, freqPilotBW_rps);

% Convert to principal angle (-2*pi < phasePilotBW_deg < 0)
phasePilotBW_deg = angle(exp(imaj * phasePilotBW_deg * d2r)) * r2d;
if phasePilotBW_deg > 0
    phasePilotBW_deg = phasePilotBW_deg - 360;
end

%% Calculate the pilot load bandwidth phase criterion
if ~isempty(phase2_deg)
    phase2BW_deg = interp1(freq_rps, phase2_deg, freqPilotBW_rps) - 14.3*freqPilotBW_rps;

    % Correct phase angle
    if phase2BW_deg < -450
        phase2BW_deg = angle(exp(imaj * phase2BW_deg * d2r)) * r2d;
    end
    if phase2BW_deg > 0
        phase2BW_deg = phase2BW_deg - 360;
    end
else
    phase2BW_deg = [];
end


%% Check Outputs
