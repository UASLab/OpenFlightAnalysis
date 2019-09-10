function [structMultiSine] = MultiSineSchroederPhase(structMultiSine, phaseComp1_rad, boundSW)
% Multisine phase shift based on Schroeder's minimum peak criterion.
%
% Inputs:
%  structMultiSine [structure]
%   ampChan_nd     - relative signal power for each component
%   indxChan       - signal components
%  phaseComp1_rad - phase of the first component (rad) [0]
%  boundSW        - switch to force the phase in the range [0, 2*pi) []
%
% Outputs:
%  structMultiSine [structure]
%   phaseChan_rad - component phases in the signals (rad)
%
% Notes:
%

%Reference:
% "Synthesis of Low-Peak-Factor Signals and Binary Sequences with Low
% Autocorrelation", Schroeder, IEEE Transactions on Information Theory
% Jan 1970.
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 3)
if ~isfield(structMultiSine, 'ampChan_nd')
    error(['oFA:' mfilename ':Inputs'], 'The ampChan_nd field must be provided');
end
if ~isfield(structMultiSine, 'indxChan')
    error(['oFA:' mfilename ':Inputs'], 'The indxChan field must be provided');
end

if nargin < 3, boundSW = [];
    if nargin < 2, phaseComp1_rad = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end

numChan = structMultiSine.numChan;

%% Compute the Schroeder phase shifts
% Compute phases  (Reference 1, Equation 11)

ampComp_nd = [];
for iChan = 1:numChan
    ampComp_nd(structMultiSine.indxChan{iChan}) = structMultiSine.ampChan_nd{iChan};
end
sigPowerRel = (ampComp_nd / max(ampComp_nd)).^2 / length(ampComp_nd);

numComp = length(ampComp_nd);
phaseComp_rad = NaN(size(ampComp_nd));
phaseComp_rad(1) = phaseComp1_rad;
for iComp = 2:numComp
    sumVal = 0;
    for indxL = 1:iComp-1
        sumVal = sumVal + (iComp - indxL)*sigPowerRel(indxL);
    end

    phaseComp_rad(iComp) = phaseComp_rad(1) - 2*pi*sumVal;
end


%% Bound Phases
% Correct phases to be in the range [0, 2*pi), optional
if boundSW
    phaseComp_rad = mod(phaseComp_rad, 2*pi);
end


%% Deal Phase components into channels
phaseChan_rad = {};
for iChan = 1:numChan
    phaseChan_rad{iChan} = phaseComp_rad(structMultiSine.indxChan{iChan});
end


%% Check Outputs
structMultiSine.ampComp_nd = ampComp_nd;
structMultiSine.phaseComp_rad = phaseComp_rad;
structMultiSine.phaseChan_rad = phaseChan_rad;

