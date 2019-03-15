function [tfParams] = Loes(freq_rps, gain_dB, phase_deg, typeTF, tfParamsKnown, gain2_dB, phase2_deg, plotTitleDesc, saveFile, outFile)
% Solve the Lower Order Equivalent Systems fit for a frequency response.
%
%Usage:  [tfParams] = Loes(freq_rps, gain_dB, phase_deg, typeTF, tfParamsKnown, gain2_dB, phase2_deg, plotTitleDesc, saveFile, outFile);
%
%Inputs:
% freq_rps      - frequency of frequency response (rad/sec)
% gain_dB       - magnitude of 1st input frequency response (dB)
% phase_deg     - phase of 1st input frequency response (deg)
% typeTF        - transfer function type; see: 'LoesTFparams'
% tfParamsKnown - known transfer function parameters []
% gain2_dB      - magnitude of 2nd input frequency response (dB) []
% phase2_deg    - phase of 2nd input frequency response (deg) []
% plotTitleDesc - plot title description (and switch) []
% saveFile      - figure save file (and switch) []
% outFile       - output save file (and switch) []
%
%Outputs:
% tfParams - structure containing transfer function parameter values
%
%Notes:
% 'tfParamsKnown' is useful if a parameter value is already known,
% Unkown parameter values must be set to 'NaN'.
%   Example: For the 'q/dep' type response the parameter 'L_alpha'
%   can be approximated from the slope of 'nz/alpha';
%   tfParamsKnown = [NaN, NaN, NaN, NaN, 0.9];
%
%Dependency:
% LoesBound
% LoesCost
% LoesFunc
% LoesPlot
% LoesSummary
% LoesTFparams
%
%Reference: 
% TODO: Add References!!
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release, based on Stoliker's routines (v1.0)
%


%% Check I/O Arguments
error(nargchk(4, 10, nargin, 'struct'))
if nargin < 10, outFile = [];
    if nargin < 9, saveFile = []; end
    if nargin < 8, plotTitleDesc = []; end
    if nargin < 7, phase2_deg = []; end
    if nargin < 6, gain2_dB = []; end
    if nargin < 5, tfParamsKnown = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
r2d = 180/pi;
d2r = pi/180;


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);
gain2_dB = gain2_dB(:);
phase2_deg = phase2_deg(:);

% Unwrap the phase angle
phase_deg = unwrap(phase_deg * d2r) * r2d;
phase2_deg = unwrap(phase2_deg * d2r) * r2d;


%% Retrieve the Transfer Function information for the type
switch typeTF
    case {3, 4, 5}
        [tfParamsInit, tfParamsLB, tfParamsUB, tfParamsParams, tfParamsInfo, tfParamsName, tfParamsTF, tfParamsName2, tfParamsTF2] = LoesTFparams(typeTF);
    otherwise
        [tfParamsInit, tfParamsLB, tfParamsUB, tfParamsParams, tfParamsInfo, tfParamsName, tfParamsTF] = LoesTFparams(typeTF);
end

% Replace the initial value and bounds with known values, if present
if ~isempty(tfParamsKnown)
    if length(tfParamsKnown) ~= length(tfParamsInit)
        error('known transfer function parameters do not match type specified')
    end

    indxKnownParam = find(~isnan(tfParamsKnown));
    tfParamsInit(indxKnownParam) = tfParamsKnown(indxKnownParam);
    tfParamsLB(indxKnownParam) = tfParamsKnown(indxKnownParam);
    tfParamsUB(indxKnownParam) = tfParamsKnown(indxKnownParam);
end


%% Solve
% set the cost function weighting
weightG = 1;
weightP = 0.0175;

% create a function handle, TF parameters is the dependent variable
switch typeTF
    case {3, 4, 5}
        costFncHandle = @(tfParamsInit) LoesCost(freq_rps, gain_dB, phase_deg, typeTF, tfParamsInit, weightG, weightP, gain2_dB, phase2_deg);
    otherwise
        costFncHandle = @(tfParamsInit) LoesCost(freq_rps, gain_dB, phase_deg, typeTF, tfParamsInit, weightG, weightP);
end

% Call the optimization routine
options = optimset('Display', 'off', 'LargeScale', 'off');
[tfParamsValue, cost, exitflag, output] = fmincon(costFncHandle, tfParamsInit, [], [], [], [], tfParamsLB, tfParamsUB, [], options);

% save transfer function parameter values in a structure
tfParams = struct();
for i = 1:length(tfParamsValue)
    tfParams = setfield(tfParams, tfParamsParams{i}, tfParamsValue(i));
end

%% Evaluate the solution to get the frequency response
switch typeTF
    case {3, 4, 5}
        [gainLoes_dB, phaseLoes_deg, gain2Loes_dB, phase2Loes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps);
    otherwise
        [gainLoes_dB, phaseLoes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps);
end


%% Bounds of MUAD
[gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg] = LoesBound(freq_rps);


%% Plot the LOES solution and input frequency response
if ~isempty(plotTitleDesc)
    figure;

    plotTitle = {[tfParamsInfo ' : ' tfParamsName]; plotTitleDesc};

    plotText{1} = tfParamsTF;
    for i = 1:length(tfParamsParams)
        plotText2{i} = [tfParamsParams{i} ': ' num2str(tfParamsValue(i))];
    end
    plotText{2} = plotText2';

    LoesPlot(freq_rps, gain_dB, phase_deg, gainLoes_dB, phaseLoes_deg, gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg, 'absolute', plotTitle, plotText);

    % Return the figure handle object
    figHandle = gcf;

    % Save the plot as a fig
    if saveFile
        saveas(figHandle, saveFile, 'fig');
    end

    switch typeTF
        case {3, 4, 5}
            figure;

            plotTitle = {[tfParamsInfo ' : ' tfParamsName2]; plotTitleDesc};

            plotText{1} = tfParamsTF2;
            for i = 1:length(tfParamsParams)
                plotText2{i} = [tfParamsParams{i} ': ' num2str(tfParamsValue(i))];
            end
            plotText{2} = plotText2';

            LoesPlot(freq_rps, gain2_dB, phase2_deg, gain2Loes_dB, phase2Loes_deg, gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg, 'absolute', plotTitle, plotText);

            % Return the figure handle object
            figHandle2 = gcf;
            figHandle = [figHandle, figHandle2];

            % Save the plot as a fig
            if saveFile
                saveas(figHandle2, [saveFile '2'], 'fig');
            end
    end
end


%% Write summary file
if outFile
    LoesSummary(outFile, typeTF, tfParamsValue, cost);
end


%% Check Outputs
