function [signals, time_s] = ModifyExcitation(signals, time_s, gains, numRepeat, seperateSW, timeLead_s, timeTrail_s)
% Modify an excitation signal by appending, repeating, padding with zeros,
% or seperating into individual signals.
%
% Inputs:
%  signals     - time history of original signals
%  time_s      - time vector (sec)
%  gains       - gains to modify excitation, scalar, vector or matrix [1]
%  numRepeat   - number of time to repeat the signal, and switch [0]
%  seperateSW  - switch to seperate the signals to run idividually in sequence [0]
%  timeLead_s  - dead time prior to excitation (sec) [0]
%  timeTrail_s - dead time after excition (sec) [0]
%
% Outputs:
%  signals - modified time history of original signals
%  time_s  - modified time vector (sec)
%
% Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(2, 7)
if nargin < 7, timeTrail_s = [];
    if nargin < 6, timeLead_s = []; end
    if nargin < 5, seperateSW = []; end
    if nargin < 4, numRepeat = []; end
    if nargin < 3, gains = []; end
end

nargoutchk(0, 2)


%% Default Values and Constants
if isempty(gains), gains = 1; end
if isempty(numRepeat), numRepeat = 0; end
if isempty(seperateSW), seperateSW = 0; end
if isempty(timeLead_s), timeLead_s = 0; end
if isempty(timeTrail_s), timeTrail_s = 0; end


%% Check Inputs
% Individual signals and row vectors, transpose if necessary
[numSignals, lenSignals] = size(signals);
if numSignals > lenSignals
    % Transpose the signals
    transposeFlag = 1;
    signals = signals';
    [numSignals, lenSignals] = size(signals);
else
    transposeFlag = 0;
end


%% Sample rate
timeRate_s = time_s(2)-time_s(1);


%% Gain
% Apply the gain to the signal
if any(gains ~= 1)
    for indxSignal = 1:numSignals
        signals(indxSignal, :) = gains(indxSignal, :) .* signals(indxSignal, :);
    end
    time_s = timeRate_s * (0:(length(signals)-1));
end


%% Append Signals
% Repeat the signal
if numRepeat >= 1
    for indxRepeat = 1:numRepeat
        signals = [signals signals];
    end
    time_s = timeRate_s * (0:(length(signals)-1));
end


%% Seperate
% Seperate the signals so that only one runs at a time
if seperateSW
    temp = zeros(numSignals, numSignals*length(signals));
    for indxSignals = 1:numSignals
        indx = (((indxSignals-1) * length(signals) + 1) : ((indxSignals) * length(signals)));
        temp(indxSignals, indx) = signals(indxSignals, :);
    end
    signals = temp;
    time_s = timeRate_s * (0:(length(signals)-1));
end


%% Pad with zeros
% Add zero time before signal begins
if timeLead_s > 0
    lenLead = find(time_s >= timeLead_s, 1 ) - 1;
    signals = [zeros(numSignals, lenLead), signals];
    time_s = timeRate_s * (0:(length(signals)-1));
end

% Add zero time after signal ends
if timeTrail_s > 0
    lenTrail = timeTrail_s/timeRate_s;
    signals = [signals, zeros(numSignals, lenTrail)];
    time_s = timeRate_s * (0:(length(signals)-1));
end


%% Check Outputs
if transposeFlag == 1
    % Un-Transpose the signals
    signals = signals';
end
