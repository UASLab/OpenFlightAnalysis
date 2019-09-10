function [structMultiSine] = ModifyExcitation(structMultiSine, numRepeat, timeLead_s, timeTrail_s)
% Modify an excitation signal by appending, repeating, padding with zeros,
% or seperating into individual signals.
%
% Inputs:
%  structMultiSine [structure]
%   signals    - time history of original signals
%   time_s     - time vector (sec)
%   timeRate_s - time rate (sec)
%  numRepeat   - number of time to repeat the signal, and switch [0]
%  timeLead_s  - dead time prior to excitation (sec) [0]
%  timeTrail_s - dead time after excition (sec) [0]
%
% Outputs:
%  structMultiSine [structure]
%   signals - modified time history of original signals
%   time_s  - modified time vector (sec)
%
% Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 4)
if nargin < 4, timeTrail_s = [];
    if nargin < 3, timeLead_s = []; end
    if nargin < 2, numRepeat = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(numRepeat), numRepeat = 0; end
if isempty(timeLead_s), timeLead_s = 0; end
if isempty(timeTrail_s), timeTrail_s = 0; end


%% Check Inputs
numChan = structMultiSine.numChan;


%% Sample rate
timeRate_s = structMultiSine.timeRate_s;
signals = structMultiSine.signals;
time_s = structMultiSine.time_s;


%% Append Signals
% Repeat the signal
if numRepeat >= 1
    signals = repmat(signals, [1, numRepeat]);
    time_s = timeRate_s * (0:(length(signals)-1));
end


%% Pad with zeros
% Add zero time before signal begins
if timeLead_s > 0
    lenLead = floor(timeLead_s/timeRate_s);
    signals = [zeros(numChan, lenLead), signals];
    time_s = timeRate_s * (0:(length(signals)-1));
end

% Add zero time after signal ends
if timeTrail_s > 0
    lenTrail = timeTrail_s/timeRate_s;
    signals = [signals, zeros(numChan, lenTrail)];
    time_s = timeRate_s * (0:(length(signals)-1));
end

%% Check Outputs
structMultiSine.signals = signals;
structMultiSine.time_s = time_s;
