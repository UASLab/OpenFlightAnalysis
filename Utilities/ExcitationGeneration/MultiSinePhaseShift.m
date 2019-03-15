function [phaseShift_rad] = MultiSinePhaseShift(signals, time_s, freqComp_rps, signalDist)
% Compute the phase shift for a multi-sine signal to start/end near zero.
%
%Usage:  [phaseShift_rad] = MultiSinePhaseShift(signals, time_s, freqComp_rps, signalDist);
%
%Inputs:
% signals      - time history of the composite signals
% time_s       - time vector for time history (s)
% freqComp_rps - frequency components of the signals (rad/s)
% signalDist   - component distribution of the the signals
%
%Outputs:
% phaseShift_rad - component phase shift (rad)
%
%Notes:
% 
%

%Reference:
% "Multiple Input Design for Real-Time Parameter Estimation"
% Eugene A. Morelli, 2003
%

%Version History: Version 1.1
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(4, 4, nargin, 'struct'))
error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Number of signals and components
[numComp, numSignals] = size(signalDist);


%% Time shift per signal
% Find the time that each signal first crosses zero
timeShift_s = zeros(numSignals, 1);
for indxSignal = 1:numSignals

    indxShift = 1;
    initSign = sign(signals(indxSignal, indxShift));
    while sign(signals(indxSignal, indxShift)) == initSign
        indxShift = indxShift + 1;
    end

    % Refine the solution via interpolation around the sign switch, and return the time shift
    timeShift_s(indxSignal) = interp1(signals(indxSignal, (indxShift-1:indxShift)), time_s((indxShift-1:indxShift)), 0);
end


%% Phase shift per component
% Find the phase shift associated with the time shift for each frequency component in the combined signals
phaseShift_rad = zeros(numComp, 1);
for indxSignal = 1:numSignals
    listSigComp = find(signalDist(:,indxSignal));
    phaseShift_rad(listSigComp, 1) = timeShift_s(indxSignal) * freqComp_rps(listSigComp);
end


%% Check Outputs
