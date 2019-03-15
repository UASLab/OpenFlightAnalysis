function [figHandle] = SmithGeddesPlotLevels(phasePilotBW_deg, phase2BW_deg, level2Bound, level1Bound, level2PIOBound, level1PIOBound, plotTitle, plotText)
% Plot the Smith/Geddes level ratings and level criteria.
%
%Usage:  [figHandle] = SmithGeddesPlotLevels(phasePilotBW_deg, phase2BW_deg, level2Bound, level1Bound, level2PIOBound, level1PIOBound, plotTitle, plotText);
%
%Inputs:
% phasePilotBW_deg - phase angle at pilot bandwidth frequency (deg)
% phase2BW_deg     - phase angle at 2nd input bandwidth frequency (deg)
% level2Bound      - level 2 boundary []
% level1Bound      - level 1 boundary []
% level2PIOBound   - PIO type III boundary []
% level1PIOBound   - PIO type I boundary []
% plotTitle        - plot title ['Smith-Geddes Requirements']
% plotText         - plot text []
%
%Outputs:
% figHandle - Handle object for the plot
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(2, 8, nargin, 'struct'))
if nargin < 8, plotText = [];
    if nargin < 7, plotTitle = []; end
    if nargin < 6, level1PIOBound = []; end
    if nargin < 5, level2PIOBound = []; end
    if nargin < 4, level1Bound = []; end
    if nargin < 3, level2Bound = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(plotTitle), plotTitle = 'Smith-Geddes Requirements'; end


%% Check Inputs
% Inputs as column vectors
phasePilotBW_deg = phasePilotBW_deg(:);
phase2BW_deg = phase2BW_deg(:);


%% Plot
% Plot data points
plot(phasePilotBW_deg, phase2BW_deg, '*k');

% Plot Boundary lines
handlePIO2 = line(level2PIOBound(:,1), level2PIOBound(:,2), 'Color', 'c', 'LineStyle', '-', 'LineWidth', 1);
handlePIO1 = line(level1PIOBound(:,1), level1PIOBound(:,2), 'Color', 'b', 'LineStyle', '-', 'LineWidth', 1);

handleLevel2 = line(level2Bound(:,1), level2Bound(:,2), 'Color', 'm', 'LineStyle', '-', 'LineWidth', 1);
handleLevel1 = line(level1Bound(:,1), level1Bound(:,2), 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1);

% Set Properties
grid on; xlim([-220 -90]); ylim([-380 -90]);
xlabel('Phase at Pilot Bandwidth (deg)', 'Interpreter', 'none');
ylabel('Phase parameter at Pilot Bandwidth (deg)', 'Interpreter', 'none');
legend([handleLevel2, handleLevel1, handlePIO2, handlePIO1], {'Level2', 'Level1', 'PIO TypeIII', 'PIO TypeI'}, 'Interpreter', 'none');

% Add title to the plot
title(plotTitle, 'Interpreter', 'none');

% Add text
if ~isempty('plotText')
    text(...
        'Units', 'normalized', ...
        'Position', [0.99, 0.05], ...
        'String', plotText, ...
        'BackgroundColor', [1 1 1], ...
        'EdgeColor', [0 0 0], ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'bottom', ...
        'Interpreter', 'none');
end

% Return the figure handle object
figHandle = gcf;


%% Check Outputs
