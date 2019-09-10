function [structMultiSine] = MultiSineOptimalPhase(structMultiSine, phaseComp1_rad, costType)
% Determine the phase shift for each multisine frequency component.
%
% Inputs:
%  structMultiSine [structure]
%   freqChan_rps   - component frequencies in the signals (rad/s)
%   ampChan_nd     - relative signal power for each component
%   time_s         - time vector for time history (s)
%   indxChan     - component distribution of the signals [ones]
%  phaseComp1_rad - phase of the first component (rad) [0]
%  costType       - Cost function type ['norm2']
%
% Outputs:
%  structMultiSine [structure]
%   phaseChan_rad - component phases in the signals (rad)
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
narginchk(1, 4)
if nargin < 4, phaseComp1_rad = [];
    if nargin < 3, costType = []; end
    if nargin < 2, indxChan = []; end
end

nargoutchk(1, 1)


%% Default Values and Constants
if isempty(costType), costType = 'norm2'; end
if isempty(phaseComp1_rad), phaseComp1_rad = 0; end


%% Check Inputs


%% Compute the optimal phases to yield minimal peak factor
% Initial guess is based on Schroeder distribution
boundSW = 1;
[structMultiSine] = MultiSineSchroederPhase(structMultiSine, phaseComp1_rad, boundSW);

numChan = length(structMultiSine.indxChan);
for iChan = 1:numChan
    structMultiSine.phaseComp_rad(structMultiSine.indxChan{iChan}) = structMultiSine.phaseChan_rad{iChan};
end


% Optimize the component phases to minimize the peak factor
% Cost function minimizes the average peakfactor for the signals, see Notes
% Setup the Optimizer
normalSW = [];

optProb.x0 = structMultiSine.phaseComp_rad;

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
            norm(CostWrapper(structMultiSine, phaseComp_rad, normalSW), 2);

        optProb.options.MaxIterations = 800;
        optProb.options.MaxFunctionEvaluations = 1e6;

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);
        
        for iChan = 1:structMultiSine.numChan
            structMultiSine.phaseChan_rad{iChan} = phaseComp_rad(structMultiSine.indxChan{iChan});
        end

    case 'squaresum'
        optProb.objective = @(phaseComp_rad) ...
            sum(CostWrapper(structMultiSine, phaseComp_rad, normalSW).^2);

        optProb.options.MaxIterations = 800;
        optProb.options.MaxFunctionEvaluations = 1e6;

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);
        
        for iChan = 1:structMultiSine.numChan
            structMultiSine.phaseChan_rad{iChan} = phaseComp_rad(structMultiSine.indxChan{iChan});
        end

    case 'max'
        optProb.objective = @(phaseComp_rad) ...
            max(CostWrapper(structMultiSine, phaseComp_rad, normalSW));

        % Compute the optimal phases
        [phaseComp_rad, fval, exitflag, output] = fminunc(optProb);
        
        for iChan = 1:structMultiSine.numChan
            structMultiSine.phaseChan_rad{iChan} = phaseComp_rad(structMultiSine.indxChan{iChan});
        end
end

% Limit the phase to [0, 2*pi]
for iChan = 1:structMultiSine.numChan
    structMultiSine.phaseChan_rad{iChan} = mod(structMultiSine.phaseChan_rad{iChan}, 2*pi);
end

%% Check Outputs

end

%% [Function] CostWrapper
function [cost] = CostWrapper(structMultiSine, phaseComp_rad, normalSW)

numChan = structMultiSine.numChan;
structMultiSine.phaseChan_rad = cell(1, numChan);
for iChan = 1:structMultiSine.numChan
    structMultiSine.phaseChan_rad{iChan} = phaseComp_rad(structMultiSine.indxChan{iChan});
end

structMultiSine = MultiSineAssemble(structMultiSine, normalSW);
cost = PeakFactor(structMultiSine.signals);

end