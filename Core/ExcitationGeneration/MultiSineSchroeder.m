function [structMultiSine] = MultiSineSchroeder(structMultiSine, phaseComp1_rad, boundSW, normalSW, forceZeroSW)
% Compute multisine signals based on Schoeders minimum peak criterion.
%
% Inputs:
%  structMultiSine [structure]
%   freqChan_rps   - component frequencies in the signals (rad/s)
%   ampChan_nd     - relative power components of the signals
%   time_s         - time vector for time history (s)
%   indxChan       - component distribution of the signals
%  phaseComp1_rad - phase of the first component (rad) [0]
%  boundSW        - switch to force the phase in the range [0, 2*pi) []
%  normalSW       - switch to normalize summed signals []
%  forceZeroSW    - switch to force initial zero [1]
%
% Outputs:
%  structMultiSine [structure]
%   signals       - time history of signals
%   phaseComp_rad - component phases in the signals (rad)
%   signalComp    - time history of all signal components
%
% Notes:
%

% Reference:
%  "Synthesis of Low-Peak-Factor Signals and Binary Sequences with Low
%  Autocorrelation", Schroeder, IEEE Transactions on Information Theory
%  Jan 1970.
%
%  "Tailored Excitation for Multivariable Stability Margin Measurement
%  Applied to the X-31A Nonlinear Simulation"  NASA TM-113085
%  John Bosworth and John Burken
%
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

if nargin < 5, forceZeroSW = [];
    if nargin < 4, normalSW = []; end
    if nargin < 3, boundSW = []; end
    if nargin < 2, phaseComp1_rad = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end
if isempty(forceZeroSW), forceZeroSW = 1; end


%% Schoeder based component phase distribution
[structMultiSine] = MultiSineSchroederPhase(structMultiSine, phaseComp1_rad, boundSW);


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
