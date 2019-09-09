function [phaseComp_rad] = MultiSineOptimalPhase(freqComp_rps, signalPowerRel, time_s, signalDist, phaseComp1_rad, costType)
% Determine the phase shift for each multisine frequency component.
%
% Inputs:
%  freqComp_rps   - component frequencies in the signals (rad/s)
%  signalPowerRel - relative signal power for each component
%  time_s         - time vector for time history (s)
%  signalDist     - component distribution of the signals [ones]
%  phaseComp1_rad - phase of the first component (rad) [0]
%  costType       - Cost function type ['norm2']
%
% Outputs:
%  phaseComp_rad - component phases in the signals (rad)
%
% Notes:
%  This function is setup for generating a flat-power spectrum.
%  The anonymous function "@() f()" is not suported prior to R14.
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
narginchk(3, 6)
if nargin < 6, phaseComp1_rad = [];
    if nargin < 5, costType = []; end
    if nargin < 4, signalDist = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(costType), costType = 'norm2'; end
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end
if isempty(signalDist), signalDist = ones(size(freqComp_rps)); end


%% Check Inputs


%% Compute the optimal phases to yield minimal peak factor
% Initial guess is based on Schroeder distribution
boundSW = 1;
[phaseComp_rad] = MultiSineSchroederPhase(signalPowerRel, phaseComp1_rad, boundSW);

% Optimize the component phases to minimize the peak factor
% Cost function minimizes the average peakfactor for the signals, see Notes
% Setup the Optimizer
normalSW = [];

optProb.x0 = phaseComp_rad;

optProb.solver = 'fminunc';
optProb.options = optimoptions(optProb.solver);

% optProb.options.Algorithm = 'interior-point';
optProb.options.Display = 'iter-detailed';
optProb.options.PlotFcns = { @optimplotfval };
optProb.options.UseParallel = true;

% Cost Type
switch lower(costType)
    case 'norm2'
        optProb.objective = @(phaseComp_rad) ...
            norm(PeakFactor(MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW)), 2);

        optProb.options.MaxIterations = 800;
        optProb.options.MaxFunctionEvaluations = 1e6;

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);

    case 'squaresum'
        optProb.objective = @(phaseComp_rad) ...
            sum(PeakFactor(MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW)).^2);

        optProb.options.MaxIterations = 800;
        optProb.options.MaxFunctionEvaluations = 1e6;

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);

    case 'max'
        optProb.objective = @(phaseComp_rad) ...
            max(PeakFactor(MultiSineAssemble(freqComp_rps, phaseComp_rad, signalPowerRel, time_s, signalDist, normalSW)));

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);

    case 'individual' % Recall the function for each individual signal
        for indxSig = 1:size(signalDist, 2)
            sigSel = signalDist(:, indxSig) == 1;

            [phaseComp_rad(sigSel)] = MultiSineOptimalPhase(freqComp_rps(sigSel), signalPowerRel(sigSel), time_s, [], [], 'squaresum');
        end
end

% Limit the phase to [0, 2*pi]
phaseComp_rad = mod(phaseComp_rad, 2*pi);

%% Check Outputs
