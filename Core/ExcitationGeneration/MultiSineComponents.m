function [structMultiSine] = MultiSineComponents(structMultiSine, methodSW)
% Define a multisine frequency component distribution.
%
% Inputs:
%  structMultiSine [structure]
%   freqRange_rps  - [minimum, max] desired frequency for each signal, matrix (rad/s)
%   timeRate_s      - sample time rate (sec)
%   numCycles       - number of lowest frequency cycles [3]
%   freqStepDes_rps - maximum frequency step size (rad/s) [0]
%  methodSW        - component distribution method ['zipper']
%
% Outputs:
%  structMultiSine [structure]
%   freqComp_rps - frequency components of the signals (rad/s)
%   time_s       - time vector (s)
%   indxComp     - component distribution of the signals
%
% Notes:
%  For a Schroeder phase distribution to work well the frequency
%  distribution should be of the fully populated "Zippered" type.
%  The length of freqMinDes_rps will determine the number of signals to
%  generate.
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 2)
if ~isfield(structMultiSine, 'freqRange_rps')
    error(['oFA:' mfilename ':Inputs'], 'The freqRange_rps field must be provided');
end
if ~isfield(structMultiSine, 'timeRate_s')
    error(['oFA:' mfilename ':Inputs'], 'The timeRate_s field must be provided');
end
if ~isfield(structMultiSine, 'numCycles'), structMultiSine.numCycles = []; end
if ~isfield(structMultiSine, 'freqStepDes_rps'), structMultiSine.freqStepDes_rps = []; end
if nargin < 2
    methodSW = [];
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(structMultiSine.numCycles), structMultiSine.numCycles = 3; end
if isempty(structMultiSine.freqStepDes_rps), structMultiSine.freqStepDes_rps = 0; end
if isempty(methodSW), methodSW = 'zipper'; end

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Check Inputs
freqMinDes_rps = structMultiSine.freqRange_rps(:,1);
freqMaxDes_rps = structMultiSine.freqRange_rps(:,2);
timeRate_s = structMultiSine.timeRate_s;
numCycles = structMultiSine.numCycles;
freqStepDes_rps = structMultiSine.freqStepDes_rps;

if length(freqMinDes_rps) ~= length(freqMaxDes_rps)
    warning(['oFA:' mfilename ':Inputs'], 'The min and max frequency inputs must be the same length');
end

freqMaxLimit_rps = (1/(2*timeRate_s) * hz2rps);
if freqMaxDes_rps > freqMaxLimit_rps
    warning(['oFA:' mfilename ':freqMax'], 'The maximum desired frequency is too high for the frame rate');
    freqMaxDes_rps = freqMaxLimit_rps;
end


%% Refine the frequency selection to avoid leakage, based on Bosworth
% Convert frequencies from rad/s to Hz
freqMinDes_hz = freqMinDes_rps * rps2hz;
freqMaxDes_hz = freqMaxDes_rps * rps2hz;

% Time vector is based on completing the desired number of cycles for the
% min frequency component, must be divisible by the frame rate
timeLength_s = (round((numCycles/min(freqMinDes_hz))/timeRate_s)) * timeRate_s;
time_s = (0 : timeRate_s : (timeLength_s - timeRate_s));

% Frequency sequence step size
freqStepMin_hz = 1 / timeLength_s;

freqStepDes_hz = freqStepDes_rps * rps2hz;
freqStep_hz = round(freqStepDes_hz / freqStepMin_hz) * freqStepMin_hz;

if freqStep_hz < freqStepMin_hz, freqStep_hz = freqStepMin_hz; end

% Adjust the min frequency based on the max time
freqMin_hz = numCycles / timeLength_s;
freqMax_hz = ceil(max(freqMaxDes_hz) / freqStep_hz) * freqStep_hz;

% Frequencies of all the components
freqComp_hz = (freqMin_hz : freqStep_hz : freqMax_hz);
freqComp_rps = freqComp_hz * hz2rps;

% Number of components available to make-up the signals
numComp = length(freqComp_rps);


%% Distribute the frequency components into the signals
% Number of signals
numChan = length(freqMinDes_rps);

% Distribution methods
indxChan = cell(1, numChan);
freqChan_rps = cell(1, numChan);
switch methodSW
    case {'zipper', 'zip'}
        % Zippered distribution
        for iChan = 1:numChan
            indxChan{iChan} = iChan:numChan:numComp;
            freqChan_rps{iChan} = freqComp_rps(indxChan{iChan});
        end

    otherwise
        % Non-Zippered distibution - FIXIT
        % place components to bound the min and max as closely as possible and
        % distribute signals with as even a distribution as possible
end


%% Check Outputs
structMultiSine.freqChan_rps = freqChan_rps;
structMultiSine.time_s = time_s;
structMultiSine.indxChan = indxChan;
