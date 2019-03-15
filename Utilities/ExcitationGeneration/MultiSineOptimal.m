function [signals, phaseComp_rad, signalComp] = MultiSineOptimal(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, normalSW, forceZeroSW, costType)
% Compute multisine signals based on Minimizing the peak factor criterion.
%
%Usage:  [signals, phaseComp_rad, signalComp] = MultiSineOptimal(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, normalSW, forceZeroSW, costType);
%
%Inputs:
% freqComp_rps   - component frequencies in the signals (rad/s)
% signalPowerRel - relative power components of the signals
% time_s         - time vector for time history (s)
% signalDist     - component distribution of the signals
% phaseComp1_rad - phase of the first component (rad) []
% normalSW       - switch to normalize summed signals []
% forceZeroSW    - switch to force initial zero []
% costType       - Cost function type []
%
%Outputs:
% signals       - time history of signals
% phaseComp_rad - component phases in the signals (rad)
% signalComp    - time history of all signal components
%
%Notes:
% 
%
%Dependency:
% MultiSineOptimalPhase
% MultiSineAssemble
% MultiSinePhaseShift
%

%Reference:
% "Multiple Input Design for Real-Time Parameter Estimation"
% Eugene A. Morelli, 2003
%

%Version History: Version 1.3
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 08/01/2006  C. Regan     Added 'phaseComp_rad' as an output (v1.1)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.2)
% 02/05/2007  C. Regan     Removed MultiSineComponents, new I/O (v1.3)
%


%% Check I/O Arguments
error(nargchk(4, 8, nargin, 'struct'))
if nargin < 8, costType = [];
    if nargin < 7, forceZeroSW = []; end
    if nargin < 6, normalSW = []; end
    if nargin < 5, phaseComp1_rad = []; end
end

error(nargoutchk(0, 6, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


%% Optimal based component phase distribution
[phaseComp_rad] = MultiSineOptimalPhase(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, costType);


%% Generate the signals
[signals, signalComp] = MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW);

% Offset the phase components to yield near zero initial and final values
% for each of the signals, based on Morrelli.  This is optional.
if forceZeroSW
    % Phase shift required for each of the frequency components
    phaseShift_rad = MultiSinePhaseShift(signals, time_s, freqComp_rps, signalDist);

    % Adjust the phasing by the shift value
    phaseComp_rad = phaseComp_rad + phaseShift_rad;
    
    % Recompute the sweep signals based on the new phasings
    [signals, signalComp] = MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW);
end


%% Check Outputs
