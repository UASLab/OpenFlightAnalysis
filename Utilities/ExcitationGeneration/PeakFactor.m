function [peakFactor] = PeakFactor(signal)
% Calculate the peak factor of a multisine signal.
%
%Usage:  [peakFactor] = PeakFactor(signal);
%
%Inputs:
% signal - time history of the signals
%
%Outputs:
% peakFactor - peak factor of each signal
%
%Notes:
% relative peak factor:  peakFactorRel = peakFactor/sqrt(2);
%

%Version History: Version 1.0
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(1, 1, nargin, 'struct'))
error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Determine if transposition is required
if size(signal, 1) > size(signal, 2)
    signal = signal';
    transpose = 1;
else
    transpose = 0;
end

% Number of signals in the input
numSignals = size(signal, 1);


%% Calculate the peak factor
for indxSignal = 1:numSignals
    signalElem = (signal(indxSignal, :));
    peakFactor(indxSignal,:) = (max(signalElem) - min(signalElem)) / (2*sqrt(dot(signalElem, signalElem) / length(signalElem)));
end


%% Check Outputs
% Fix the transpose if necessary
if transpose == 1
    peakFactor = peakFactor';
end
    