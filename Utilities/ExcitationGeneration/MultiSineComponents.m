function [freqComp_rps, time_s, signalDist] = MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, timeRate_s, numCycles, freqStepDes_rps, methodSW)
% Define a multisine frequency component distribution.
%
%Usage:  [freqComp_rps, time_s, signalDist] = MultiSineComponents(freqMinDes_rps, freqMaxDes_rps, timeRate_s, numCycles, freqStepDes_rps, methodSW);
%
%Inputs:
% freqMinDes_rps  - minimum desired frequency for each signal, vector (rad/s)
% freqMaxDes_rps  - maximum desired frequency for each signal, vector (rad/s)
% timeRate_s      - sample time rate (sec)
% numCycles       - number of lowest frequency cycles [3]
% freqStepDes_rps - maximum frequency step size (rad/s) [0]
% methodSW        - component distribution method ['zipper']
%
%Outputs:
% freqComp_rps - frequency components of the signals (rad/s)
% time_s       - time vector (s)
% signalDist   - component distribution of the signals
%
%Notes:
% For a Schroeder phase distribution to work well the frequency
% distribution should be of the fully populated "Zippered" type.
% The length of freqMinDes_rps will determine the number of signals to
% generate.
%

%Version History: Version 1.3
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 06/28/2006  C. Regan     Minor changes for R13 compatibility (v1.1)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.2)
% 02/05/2006  C. Regan     Added freqStepDes_rps input and use (v1.3)
%


%% Check I/O Arguments
error(nargchk(3, 6, nargin, 'struct'))
if nargin < 6, methodSW = [];
    if nargin < 5, freqStepDes_rps = []; end
    if nargin < 4, numCycles = []; end
end

error(nargoutchk(0, 3, nargout, 'struct'))


%% Default Values and Constants
if isempty(numCycles), numCycles = 3; end
if isempty(freqStepDes_rps), freqStepDes_rps = 0; end
if isempty(methodSW), methodSW = 'zipper'; end

% Constants
hz2rps = 2*pi;
rps2hz = 1/hz2rps;


%% Check Inputs
if length(freqMinDes_rps) ~= length(freqMaxDes_rps)
    warning(['AVFTT:' mfilename ':Inputs'], 'The min and max frequency inputs must be the same length');
end

freqMaxLimit_rps = (1/(2*timeRate_s) * hz2rps);
if freqMaxDes_rps > freqMaxLimit_rps
    warning(['AVFTT:' mfilename ':freqMax'], 'The maximum desired frequency is too high for the frame rate');
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
freqComp_hz = (freqMin_hz : freqStep_hz : freqMax_hz)';
freqComp_rps = freqComp_hz * hz2rps;

% Number of components available to make-up the signals
numComp = length(freqComp_rps);


%% Distribute the frequency components into the signals
% Number of signals
numSignals = length(freqMinDes_rps);

% Initialize the signal distribution to zeros
signalDist = zeros(numComp, numSignals);

% Distribution methods
switch methodSW
    case {'zipper', 'zip'}
        % Zippered distribution
        for indxComp = 1:numSignals:numComp
            for indxSignal = 1:numSignals
                if indxComp <= numComp
                    signalDist(indxComp, indxSignal) = 1;
                end
                indxComp = indxComp + 1;
            end
        end

    otherwise
        % Non-Zippered distibution - FIXIT
        % place components to bound the min and max as closely as possible and
        % distribute signals with as even a distribution as possible
end
