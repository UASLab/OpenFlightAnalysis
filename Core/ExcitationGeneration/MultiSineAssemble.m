function [structMultiSine] = MultiSineAssemble(structMultiSine, normalSW)
% Assemble multi-sine composite signals.
%
% Inputs:
%  structMultiSine [structure]
%   freqChan_rps   - frequency components of the signals (rad/s)
%   phaseChan_rad  - phase components of the signals (rad)
%   ampChan_nd     - amplitude components of the signals
%   time_s         - time vector for time history (s)
%   indxChan       - component distribution of the signals [ones]
%  normalSW       - switch to normalize the signals to [-1 1]
%
% Outputs:
%  structMultiSine [structure]
%   signals    - time history of the composite signals
%   signalChan - time history of all signal components
%
% Notes:
%  Signal power is a function of frequency and length of time: FIXME:
%   signalPower = 0.5 .* amplComp.^2 .* (freqChan_rps*timeEnd_s + sin(freqChan_rps*timeEnd_s).*cos(freqChan_rps*timeEnd_s));
%   signalPower = 2 .* amplComp.^2
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%


%% Check I/O Arguments
narginchk(1, 2)
if nargin < 2
    normalSW = [];
end

nargoutchk(0, 2)


%% Default Values and Constants


%% Check Inputs


%% Generate the composite signals
numChan = structMultiSine.numChan;
lenTime = length(structMultiSine.time_s);

freqChan_rps = structMultiSine.freqChan_rps;
ampChan_nd = structMultiSine.ampChan_nd;
phaseChan_rad = structMultiSine.phaseChan_rad;

% Generate each signal component
signalChan = cell(1, numChan);
for iChan = 1:numChan
    numComp = length(freqChan_rps{iChan});
    signalChan{iChan} = zeros(numComp, lenTime);
    for iComp = 1:numComp
        signalChan{iChan}(iComp, :) = ampChan_nd{iChan}(iComp) .* cos(freqChan_rps{iChan}(iComp).*structMultiSine.time_s + phaseChan_rad{iChan}(iComp));
    end
end

% Combine signal components into signals and normalize
signals = zeros(numChan, lenTime);
for iChan = 1:numChan
    signals(iChan, :) = sum(signalChan{iChan});
    
    % Normalize signals
    if normalSW
        scale = 1 ./ max(abs(signals(iChan,:)));
        signals(iChan, :) = scale * signals(iChan, :);
        structMultiSine.ampChan_nd{iChan} = scale * structMultiSine.ampChan_nd{iChan};
    end
end


%% Check Outputs
structMultiSine.signalChan = signalChan;
structMultiSine.signals = signals;
