function [structMultiSine] = MultiSineOptimal(structMultiSine, phaseComp1_rad, normalSW, forceZeroSW, costType)
% Compute multisine signals based on Minimizing the peak factor criterion.
%
% Inputs:
%  structMultiSine [structure]
%   freqChan_rps   - component frequencies in the signals (rad/s)
%   ampChan_nd     - relative power components of the signals
%   time_s         - time vector for time history (s)
%   indxChan       - component distribution of the signals
%  phaseComp1_rad - phase of the first component (rad) []
%  normalSW       - switch to normalize summed signals []
%  forceZeroSW    - switch to force initial zero []
%  costType       - Cost function type []
%
% Outputs:
%  structMultiSine [structure]
%   signals       - time history of signals
%   phaseChan_rad - component phases in the signals (rad)
%   signalChan    - time history of all signal components
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
narginchk(1, 5)
if ~isfield(structMultiSine, 'freqChan_rps')
    error(['oFA:' mfilename ':Inputs'], 'The freqChan_rps field must be provided');
end
if ~isfield(structMultiSine, 'ampChan_nd')
    error(['oFA:' mfilename ':Inputs'], 'The ampChan_nd field must be provided');
end
if ~isfield(structMultiSine, 'time_s')
    error(['oFA:' mfilename ':Inputs'], 'The time_s field must be provided');
end
if ~isfield(structMultiSine, 'indxChan')
    error(['oFA:' mfilename ':Inputs'], 'The indxChan field must be provided');
end

if nargin < 5, costType = [];
    if nargin < 4, forceZeroSW = []; end
    if nargin < 3, normalSW = []; end
    if nargin < 2, phaseComp1_rad = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end
if isempty(forceZeroSW), forceZeroSW = 1; end


%% Optimal based component phase distribution
[structMultiSine] = MultiSineOptimalPhase(structMultiSine, phaseComp1_rad, costType);


%% Generate the signals
[structMultiSine] = MultiSineAssemble(structMultiSine, normalSW);

% Offset the phase components to yield near zero initial and final values
% for each of the signals, based on Morrelli.  This is optional.
if forceZeroSW
    % Phase shift required for each of the frequency components
    structMultiSine = MultiSinePhaseShift(structMultiSine);

    % Recompute the sweep signals based on the new phasings
    [structMultiSine] = MultiSineAssemble(structMultiSine, normalSW);
end


%% Check Outputs
