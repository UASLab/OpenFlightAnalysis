function [phaseComp_rad] = MultiSineSchroederPhase(signalPowerRel, phaseComp1_rad, boundSW)
% Multisine phase shift based on Schroeder's minimum peak criterion.
%
% Inputs:
%  signalPowerRel - relative signal power for each component
%  phaseComp1_rad - phase of the first component (rad) [0]
%  boundSW        - switch to force the phase in the range [0, 2*pi) []
%
% Outputs:
%  phaseComp_rad - component phases in the signals (rad)
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
if nargin < 3, boundSW = [];
    if nargin < 2, phaseComp1_rad = []; end
end

nargoutchk(0, 1)


%% Default Values and Constants
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end


%% Check Inputs
% Ensure the summation of the relative signal power vector is unity
if sum(signalPowerRel) ~= 1
    warning(['oFA:' mfilename ':SignalPower'], ['Sum of relative signal power should be unity: ' num2str(sum(signalPowerRel))]);
end


%% Compute the Schroeder phase shifts
% Compute phases  (Reference 1, Equation 11)
numComp = length(signalPowerRel);
phaseComp_rad(1) = phaseComp1_rad;
for indxComp = 2:numComp
    sumVal = 0;
    for indxL = 1:indxComp-1
        sumVal = sumVal + (indxComp - indxL)*signalPowerRel(indxL);
    end

    phaseComp_rad(indxComp, 1) = phaseComp_rad(1) - 2*pi*sumVal;
end


%% Bound Phases
% Correct phases to be in the range [0, 2*pi), optional
if boundSW
    phaseComp_rad = mod(phaseComp_rad, 2*pi);
end


%% Check Outputs
