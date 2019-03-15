function [gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg] = LoesBound(freq_rps)
% Frequency response for the "Maximum Unnoticed Added Dynamics" bounds.
%
%Usage:  [gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg] = LoesBound(freq_rps);
%
%Inputs:
% freq_rps - frequency of frequency response (rad/sec)
%
%Outputs:
% gainLowBound_dB - lower gain bound frequency response (dB)
% phaseLowBound_deg  - lower phase bound frequency response (deg)
% gainUpBound_dB   - upper gain bound frequency response (dB)
% phaseUpBound_deg - upper phase bound frequency response (deg)
%
%Notes:
% 
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(1, 1, nargin, 'struct'))
error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


%% Transfer function definitions of boundaries
% TODO: Check bounary values
% Lower bound numerators and denominators
gainNumLow    = [0.095, 9.92, 2.15];
gainDenomLow  = [1, 11.6, 4.95];
phaseNumLow   = [475.32, 184100, 29460];
phaseDenomLow = [1, 11.66, 0.039];

% Upper bound numerators and denominators
phaseNumUp   = [68.89, 1100.12, -275.22];
phaseDenomUp = [1, 39.94, 9.99];
gainNumUp    = [3.16, 31.61, 22.79];
gainDenomUp  = [1, 27.14, 1.84];


%% Frequency response (gain, phase) for boundaries
% Lower bound frequency response
[gainLowBound, temp] = bode(gainNumLow, gainDenomLow, freq_rps);
[temp, phaseLowBound_deg] = bode(phaseNumLow, phaseDenomLow, freq_rps);
gainLowBound_dB = 20*log10(gainLowBound);

% Upper bound frequency response
[gainUpBound, temp] = bode(gainNumUp, gainDenomUp, freq_rps);
[temp, phaseUpBound_deg] = bode(phaseNumUp, phaseDenomUp, freq_rps);
gainUpBound_dB = 20*log10(gainUpBound);


%% Check Outputs
