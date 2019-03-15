function [pilotLead_deg, resPeak_dB, freqIn_rps, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithComp(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLag_sec, droop_dB, respType)
% Compute the Neal/Smith handling qualities criterion parameters.
%
%Usage:  [pilotLead_deg, resPeak_dB, freqIn_rps, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithComp(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLag_sec, droop_dB, respType);
%
%Inputs:
% freq_rps     - frequencies corresponding to gain and phase (rad/sec)
% gain_dB      - magnitude of frequency response (dB) (Q/DEP optional)
% phase_deg    - phase of frequency response (deg) (Q/DEP optional)
% freqBW_rps   - bandwidth frequency (rad/sec) [3.0]
% pilotLag_sec - pilot Lag (sec) [0.3]
% droop_dB     - droop value (dB) [-3.0]
% respType     - 'q' if using Q/STICK freq response ['theta']
%
%Outputs:
% pilotLead_deg - required pilot lead (deg)
% resPeak_dB    - resonance peak resulting (dB)
% freqIn_rps    - frequency vector close-loop frequency response (rad/sec)
% gainCL_dB     - magnitude of close-loop frequency response (dB)
% phaseCL_deg   - phase of close-loop frequency response (deg)
% gainOL_dB     - magnitude of open-loop frequency response (dB)
% phaseOL_deg   - phase of open-loop frequency response (deg)
%
%Notes:
% Based on input frequency response of pitch attitude to stick (THA/STICK)
%   or, optionally, pitch rate to stick (Q/STICK).
% Output vectors for freq, gain, phase contain one more point than the input vectors.
% A bisection method is used to prime the secant method for root finding.
%   It does this by varying a pseudo pilot compensation in terms of pilot
%   lead and gain to arive at closed outer-loop phase = -90 degrees at the
%   pilot bandwidth frequency, and closed loop gain = -3dB up to the
%   bandwidth frequency.  The resulting pilot lead and the peak value of
%   the closed loop magnitude are used to determine the handling qualities
%   level.
%
%Dependency:
% NealSmithDroopPeak
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release, based on Keith Wichman's routines (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 7, nargin, 'struct'))
if nargin < 7, respType = [];
    if nargin < 6, droop_dB = []; end
    if nargin < 5, pilotLag_sec = []; end
    if nargin < 4, freqBW_rps = []; end
end

error(nargoutchk(0, 7, nargout, 'struct'))


%% Default Values and Constants
if isempty(freqBW_rps), freqBW_rps = 3.0; end
if isempty(pilotLag_sec), pilotLag_sec = 0.3; end
if isempty(droop_dB), droop_dB = -3.0; end
if isempty(respType), respType = 'theta'; end

% Constants
d2r = pi/180;
r2d = 180/pi;

%Set maximum number of iterations.
iterMax = 40;


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Add pilot lag to input phase
phaseLaged_deg = phase_deg - pilotLag_sec * (freq_rps * r2d);
gainLaged_dB = gain_dB;


%% Check Response type
if strcmp(respType, 'q')
    phaseLaged_deg = phaseLaged_deg - 90;
    gainLaged_dB = gainLaged_dB - mag2dB(freq_rps);
end

%% Add a point at the bandwith frequency
% Calculate the gain and phase at the bandwidth frequency
indxBW = min(find(freq_rps >= freqBW_rps));
gainBW_dB = interp1(freq_rps, gainLaged_dB, freqBW_rps);
phaseBW_deg = interp1(freq_rps, phaseLaged_deg, freqBW_rps);

% Insert the bandwidth into the freq, gain, phase vectors
freqIn_rps = [freq_rps(1:indxBW-1) ; freqBW_rps ; freq_rps(indxBW:end)];
gainIn_dB = [gainLaged_dB(1:indxBW-1) ; gainBW_dB ; gainLaged_dB(indxBW:end)];
phaseIn_deg = [phaseLaged_deg(1:indxBW-1) ; phaseBW_deg ; phaseLaged_deg(indxBW:end)];


%% Search
% Prime the secant method with 2 solutions computed from the maximum lead
% possible (that lead which would put the c.l. at -3dB/-90deg point which
% is -125.3 degrees open loop phase) and half of that amount of lead
leadin(1) = -125.3 - phaseBW_deg;
[droopRoot(1), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(1), droop_dB, 'droop');
leadin(2) = leadin(1) / 2;
[droopRoot(2), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(2), droop_dB, 'droop');

% Begin Loop which iterates on lead input to meet droop requirements using
% secant method.  It computes the necessary gain to move 'freqBW_rps' point to the
% closed loop -90 contour and then compute the closed loop based on this
% 'pilotLead_deg'
i = 2;   %Since 2 solutions have been computed.
while (abs(droopRoot(i)) > 0.1) % Tolerance of 0.1 of the droop level to end
    % Stop if max iterations exceeded
    if i > iterMax
        break
    end
    i = i + 1;

    %Secant Method algorithm.
    leadin(i) = leadin(i-1) - droopRoot(i-1)*(leadin(i-1)-leadin(i-2)) / (droopRoot(i-1)-droopRoot(i-2));
    [droopRoot(i), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(i), droop_dB, 'droop');
end
pilotLead_deg = leadin(i);

% Compute Resonance peak
gainCLMin_dB = min(gainCL_dB(1:indxBW));
indxDroop = max(find(gainCL_dB(1:indxBW) == gainCLMin_dB));
resPeak_dB = max(gainCL_dB(indxDroop:end));

% If resonance peak is less than 2dB, relax -3dB droop requirement to < 2dB
% (See NASA CR#163097....-3dB here forces unrealistic pilot compensation)
if resPeak_dB<2.0
    %Compute solutions of peak for the last 2 iterations
    [peakRoot(i-1), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(i-1), [], 'peak');
    [peakRoot(i), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(i), [], 'peak');

    while (abs(peakRoot(i)) > 0.1)
        % Stop if max iterations exceeded
        if i > iterMax
            break
        end

        i = i + 1;

        leadin(i) = leadin(i-1) - peakRoot(i-1)*(leadin(i-1) - leadin(i-2))/(peakRoot(i-1) - peakRoot(i-2));
        [peakRoot(i), gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freqIn_rps, gainIn_dB, phaseIn_deg, freqBW_rps, leadin(i), [], 'peak');
    end
    pilotLead_deg = leadin(i);

    % Compute Resonance peak
    resPeak_dB = max(gainCL_dB);
end


%% Check Outputs
