function [phaseComp_rad] = MultiSineSchroederPhase(signalPowerRel, phaseComp1_rad, boundSW)
% Multisine phase shift based on Schroeder's minimum peak criterion.
%
%Usage:  [phaseComp_rad] = MultiSineSchroederPhase(signalPowerRel, phaseComp1_rad, boundSW);
%
%Inputs:
% signalPowerRel - relative signal power for each component
% phaseComp1_rad - phase of the first component (rad) [0]
% boundSW        - switch to force the phase in the range [0, 2*pi) []
%
%Outputs:
% phaseComp_rad - component phases in the signals (rad)
%
%Notes:
% 
%

%Reference:
% "Synthesis of Low-Peak-Factor Signals and Binary Sequences with Low
% Autocorrelation", Schroeder, IEEE Transactions on Information Theory
% Jan 1970.
%

%Version History: Version 1.2
% 06/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
% 02/05/2007  C. Regan     I/O order change, default values (v1.2)
%


%% Check I/O Arguments
error(nargchk(1, 3, nargin, 'struct'))
if nargin < 3, boundSW = [];
    if nargin < 2, phaseComp1_rad = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end


%% Check Inputs
% Ensure the summation of the relative signal power vector is unity
if sum(signalPowerRel) ~= 1
    warning(['AVFTT:' mfilename ':SignalPower'], ['Sum of relative signal power should be unity: ' num2str(sum(signalPowerRel))]);
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

% For flat-power spectrum
%
% Compute phases (Reference 1, Equation 12)
%phaseComp_rad(1) = phaseComp1_rad;
%for indxComp = 2:numComp
%    phaseComp_rad(indxComp, 1) = phaseComp_rad(1) - (pi/numComp)*indxComp^2;
%end

% There are several alternate definitions in the literature, all yield
% similiar peak factors:

% Flat-power spectrum (1/numComp)
%phaseComp_rad(:, 1) = (pi/numComp)*([1:numComp].^2 + [1:numComp]);

% Slightly simplified, this version is the one typically used at Dryden
%phaseComp_rad(:, 1) = (pi/numComp)*([1:numComp].^2);


%% Bound Phases
% Correct phases to be in the range [0, 2*pi), optional
if boundSW
    phaseComp_rad = mod(phaseComp_rad, 2*pi);
end


%% Check Outputs
