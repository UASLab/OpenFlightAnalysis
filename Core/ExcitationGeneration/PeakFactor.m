function [peakFactor] = PeakFactor(signal)
% Calculate the peak factor of a multisine signal.
%
% Inputs:
%  signal - time history of the signals
%
% Outputs:
%  peakFactor - peak factor of each signal
%
% Notes:
%  relative peak factor:  peakFactorRel = peakFactor/sqrt(2);
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 1)
nargoutchk(0, 1)


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
