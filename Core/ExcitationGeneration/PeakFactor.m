function [peakFactor] = PeakFactor(signals)
% Calculate the peak factor of a multisine signal.
%
% Inputs:
%  signals - time history of the signals
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


%% Check Inputs
% Number of signals in the input
numChan = size(signals, 1);


%% Calculate the peak factor
peakFactor = zeros(numChan, 1);
for iChan = 1:numChan
    signalElem = signals(iChan, :);
    peakFactor(iChan) = (max(signalElem) - min(signalElem)) / (2*sqrt(dot(signalElem, signalElem) / length(signalElem)));
end

