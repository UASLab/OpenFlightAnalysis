function [figHandle] = BodePlot(freq, gain_dB, phase_deg, xyC, coherLimit, plotTitle, saveFile)
% Plot the Gain, Phase, and Coherence versus frequency.
%
%Usage:  [figHandle] = BodePlot(freq, gain_dB, phase_deg, xyC, coherLimit, plotTitle, saveFile);
%
%Inputs:
% freq       - frequency of response
% gain_dB    - gain of response (dB)
% phase_deg  - phase of response (deg)
% xyC        - coherence of response []
% coherLimit - coherence threshold []
% plotTitle  - title of plot ['Bode Plot']
% saveFile   - file to save figure []
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
narginchk(3, 7);
if nargin < 7, saveFile = [];
    if nargin < 6, plotTitle = []; end
    if nargin < 5, coherLimit = []; end
    if nargin < 4, xyC = []; end
end

nargoutchk(0, 1);


%% Default Values and Constants
if isempty(plotTitle), plotTitle = 'Bode Plot'; end


%% Check Inputs


%% Plot
% New figure with handle
figHandle = figure;

% Set number of plots
if ~isempty(xyC)
    numPlots = 3;
else
    numPlots = 2;
end

% Min and Max Freq axis for plotting
plotFreqMin = min(freq);
plotFreqMax = max(freq);

% Gain Plot
subplot(numPlots, 1, 1);
semilogx(freq, gain_dB);
grid on; xlim([plotFreqMin, plotFreqMax]); ylim('auto');
ylabel('Gain (dB)');
title(plotTitle, 'Interpreter', 'none');

% Phase Plot
subplot(numPlots, 1, 2);
semilogx(freq, phase_deg);
grid on; xlim([plotFreqMin, plotFreqMax]); ylim('auto');
ylabel('Phase (deg)');

% Coherence Plot
if ~isempty(xyC)
    subplot(numPlots, 1, 3);
    semilogx(freq, xyC);
    grid on; xlim([plotFreqMin, plotFreqMax]);
    ylabel('Coherence');

    % Min and Max Coherence axis for plotting, and threshold line
    if ~isempty(coherLimit) % coherLimit defined
        plotCoherMin = coherLimit - (1 - coherLimit);
        plotCoherMax = 1;

        % Coherence theshold line
        line([plotFreqMin plotFreqMax], [coherLimit, coherLimit], 'LineStyle', '--');

    else % coherLimit not defined
        plotCoherMin = max(min(xyC));
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
if saveFile
    saveas(figHandle, saveFile, 'fig');
end


%% Check Outputs
