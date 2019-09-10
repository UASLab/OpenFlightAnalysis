function [structMultiSine] = MultiSinePhaseShift(structMultiSine)
% Compute the phase shift for a multi-sine signal to start/end near zero.
%
% Inputs:
%  structMultiSine [structure]
%   signals      - time history of the composite signals
%   time_s       - time vector for time history (s)
%   freqChan_rps - frequency components of the signals (rad/s)
%   indxChan     - component distribution of the the signals
%
% Outputs:
%  structMultiSine [structure]
%   phaseChan_rad - component phase shifted (rad)
%
% Notes:
%

% Reference:
%  "Multiple Input Design for Real-Time Parameter Estimation"
%  Eugene A. Morelli, 2003
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
numChan = structMultiSine.numChan;


%% Time shift per signal
% Find the time that each signal first crosses zero
tShift_s = zeros(numChan, 1);
for iChan = 1:numChan

    iShift = 1;
    initSign = sign(structMultiSine.signals(iChan, iShift));
    while sign(structMultiSine.signals(iChan, iShift)) == initSign
        iShift = iShift + 1;
    end

    % Refine the solution via interpolation around the sign switch, and return the time shift
    tShift_s(iChan, :) = interp1(structMultiSine.signals(iChan, iShift-1:iShift), structMultiSine.time_s(iShift-1:iShift), 0);
end


%% Phase shift per component
% Find the phase shift associated with the time shift for each frequency component in the combined signals
structMultiSine.phaseShift_rad = cell(1, numChan);
for iChan = 1:numChan
    structMultiSine.phaseShift_rad{iChan} = tShift_s(iChan, :) * structMultiSine.freqChan_rps{iChan};
end

% Adjust the phasing by the shift value
for iChan = 1:numChan
    structMultiSine.phaseChan_rad{iChan} = structMultiSine.phaseChan_rad{iChan} + structMultiSine.phaseShift_rad{iChan};
end
    
%% Check Outputs

