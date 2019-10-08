function [figHandle] = BodePlot(frf, optPlot)
% Plot the Gain, Phase, and Coherence versus frequency.
%
%Usage:  [figHandle] = BodePlot(frf, title, saveFile);
%
%Inputs:
% frf
%   freq       - frequency of response
%   gain_dB    - gain of response (dB)
%   phase_deg  - phase of response (deg)
%   xyC        - coherence of response []
% optPlot
%   coherLimit - coherence threshold []
%   title      - title of plot ['Bode Plot']
%   saveFile   - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
%
%Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 2);
if nargin < 2, optPlot = struct(); end

nargoutchk(0, 1);


%% Default Values and Constants
if ~isfield(optPlot, 'title'), optPlot.title = []; end
if isempty(optPlot.title), optPlot.title = 'Bode Plot'; end

if ~isfield(optPlot, 'coherLimit'), optPlot.coherLimit = []; end

%% Check Inputs
if ~isfield(frf, 'coher')
    frf.coher = [];
end

if ~isfield(frf, 'gain_dB') | ~isfield(frf, 'phase_deg')
    [frf.gain_dB, frf.phase_deg] = GainPhase(frf.T);
end


%% Plot
% New figure with handle
figHandle = figure;

% Set number of plots
if ~isempty(frf.coher)
    numPlots = 3;
else
    numPlots = 2;
end

% Min and Max Freq axis for plotting
plotFreqMin = min(frf.freq);
plotFreqMax = max(frf.freq);

% Gain Plot
subplot(numPlots, 1, 1);
semilogx(frf.freq, frf.gain_dB);
grid on; xlim([plotFreqMin, plotFreqMax]); ylim('auto');
ylabel('Gain (dB)');
title(optPlot.title, 'Interpreter', 'none');

% Phase Plot
subplot(numPlots, 1, 2);
semilogx(frf.freq, frf.phase_deg);
grid on; xlim([plotFreqMin, plotFreqMax]); ylim('auto');
ylabel('Phase (deg)');

% Coherence Plot
if ~isempty(frf.coher)
    subplot(numPlots, 1, 3);
    semilogx(frf.freq, frf.coher);
    grid on; xlim([plotFreqMin, plotFreqMax]);
    ylabel('Coherence');

    % Min and Max Coherence axis for plotting, and threshold line
    if ~isempty(optPlot.coherLimit) % coherLimit defined
        plotCoherMin = optPlot.coherLimit - (1 - optPlot.coherLimit);
        plotCoherMax = 1;

        % Coherence theshold line
        line([plotFreqMin plotFreqMax], [optPlot.coherLimit, optPlot.coherLimit], 'LineStyle', '--');

    else % coherLimit not defined
        plotCoherMin = max(min(frf.coher));
        plotCoherMax = 1;
    end

    if plotCoherMin == 1 %prevent min = max
        plotCoherMin = 0;
    end

    % Coherence plot limits
    ylim([plotCoherMin, plotCoherMax]);
end

% Add Common x-axis label, and link the axes together and apply tight limits
subplot(numPlots, 1, numPlots); xlabel('Frequency');
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');


%% Save the plot as a fig
if isfield (optPlot, 'saveFile')
    saveas(figHandle, optPlot.saveFile, 'fig');
end


%% Check Outputs
