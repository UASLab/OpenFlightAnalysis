function [root, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLead_deg, droop_dB, rootType)
% Calculate the peak or droop root
%
%Usage:  [root, gainCL_dB, phaseCL_deg, gainOL_dB, phaseOL_deg] = NealSmithDroopPeak(freq_rps, gain_dB, phase_deg, freqBW_rps, pilotLead_deg, droop_dB, rootType);
%
%Inputs:
% freq_rps      - frequencies corresponding to gain and phase (rad/sec)
% gain_dB       - magnitude of frequency response (dB) (Q/DEP optional)
% phase_deg     - phase of frequency response (deg) (Q/DEP optional)
% freqBW_rps    - bandwidth frequency (rad/sec) [3.0]
% pilotLead_deg - pilot phase Lead (deg)
% droop_dB      - droop (or peak) value (dB)
% rootType      - root type to find ('droop' or 'peak') ['droop']
%
%Outputs:
% root        - peak or droop root (dB)
% gainCL_dB   - magnitude of close-loop frequency response (dB)
% phaseCL_deg - phase of close-loop frequency response (deg)
% gainOL_dB   - magnitude of open-loop frequency response (dB)
% phaseOL_deg - phase of open-loop frequency response (deg)
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(6, 7, nargin, 'struct'))
if nargin < 7, rootType = []; end

error(nargoutchk(0, 5, nargout, 'struct'))


%% Default Values and Constants
if isempty(freqBW_rps), freqBW_rps = 3.0; end
if isempty(rootType), rootType = 'droop'; end

d2r = pi/180;
r2d = 180/pi;
imaj = sqrt(-1);


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;


%% Calculate the gain and phase at bandwidth frequency
gainBW_dB = interp1(freq_rps, gain_dB, freqBW_rps);
phaseBW_deg = interp1(freq_rps, phase_deg, freqBW_rps);


%% Calculate the pilot compensator lead-lag time constants
if pilotLead_deg >= 0
    % If positive lead is required, the lag time constant can be set to zero,
    % leaving a pure lead (not lead-lag)
    tau1 = sin(pilotLead_deg * d2r) / cos(pilotLead_deg * d2r) / freqBW_rps;
    tau2 = 0;
elseif pilotLead_deg < 0
    % If lag is required, then we use a lead-lag filter for pilot compensation
    switch rootType
        case 'peak'
            a = freqBW_rps;
            b = -(1 + freqBW_rps^3) * tan(pilotLead_deg * d2r);
            c = -freqBW_rps^2;
            tau1 = max(roots([a b c]));
            tau2 = freqBW_rps / tau1;
        case 'droop'
            a = 1;
            b = -2 * tan(pilotLead_deg * d2r);
            c = -1;
            tau1 = max(roots([a b c])) / freqBW_rps;
            tau2 = 1 / (tau1 * freqBW_rps^2);
        otherwise
            warning('') % FIXME: blank warning
    end
end

%% Calculate the complex frequency response of the pilot lead-lag compensator
% (1 + j*w*tau1) / (1 + j*w*tau2)
H = (1 + imaj*tau1*freq_rps) ./ (1 + imaj*tau2*freq_rps);

%% Convert the plant frequency response to complex frequency response
% Calculate the gain necessary to shift the magnitude at the bandwidth
% frequency to hit the -90 deg CLFR phase curve.
gainLeadLag = sqrt((1 + tau1^2 * freqBW_rps^2) / (1 + tau2^2 * freqBW_rps^2));
gainCompBW_dB = Mag2DB(-cos((phaseBW_deg + pilotLead_deg) * d2r)) - (Mag2DB(gainLeadLag) + gainBW_dB);

gainG = DB2Mag(gain_dB + gainCompBW_dB);
phaseG = phase_deg * d2r;
G = (gainG.*cos(phaseG)) + imaj*(gainG.*sin(phaseG));

%% Calculate the compensated open-loop frequency response (GH)
OLFR = G.*H;
gainOL_dB = Mag2DB(abs(OLFR));
phaseOL_deg = unwrap(angle(OLFR)) * r2d;

%% Calculate the close-loop frequency response (GH/(1+GH))
CLFR = (OLFR)./(1 + OLFR);
gainCL_dB = Mag2DB(abs(CLFR));
phaseCL_deg = unwrap(angle(CLFR)) * r2d;

%% Calculate the Root
% Find the min close-loop gain up to bandwidth frequency, the droop point
gainMinCL_dB = min(gainCL_dB(freq_rps < freqBW_rps));

switch rootType
    case 'peak'
        indxDroop = max(find(gainCL_dB(freq_rps < freqBW_rps) == gainMinCL_dB));
        gainPeakCL_dB = max(gainCL_dB(indxDroop:end));

        % Peak root
        root = gainPeakCL_dB - 2.0;
    case 'droop'
        % Droop root
        root = gainMinCL_dB - droop_dB;
    otherwise
        warning('') % FIXME: blank warning
end


%% Check Outputs
