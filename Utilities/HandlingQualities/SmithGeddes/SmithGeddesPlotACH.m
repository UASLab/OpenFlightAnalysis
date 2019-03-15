function [figHandle] = SmithGeddesPlotACH(phasePilotBW_deg, aveCH, levelCH, levelCHLow, levelCHUp, plotTitle, plotText)
% Plot the Smith/Geddes Average Cooper-Harper Rating.
%
%Usage: [figHandle] = SmithGeddesPlotACH(phasePilotBW_deg, aveCH, levelCH, levelCHLow, levelCHUp, plotTitle, plotText);
%
%Inputs:
% phasePilotBW_deg - phase at pilot bandwidth frequency (deg)
% aveCH            - average Cooper-Harper Rating
% levelCH          - average Cooper-Harper level line
% levelCHLow       - lower Cooper-Harper level line
% levelCHUp        - upper Cooper-Harper level line
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
error(nargchk(2, 7, nargin, 'struct'))
if nargin < 7, plotText = [];
    if nargin < 6, plotTitle = []; end
    if nargin < 5, levelCHUp = []; end
    if nargin < 4, levelCHLow = []; end
    if nargin < 3, levelCH = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(plotTitle), plotTitle = 'Smith-Geddes Requirements'; end


%% Check Inputs
% Inputs as column vectors
phasePilotBW_deg = phasePilotBW_deg(:);
aveCH = aveCH(:);


%% Plot
% Plot data points
plot(phasePilotBW_deg, aveCH, '*k');

% Plot Boundary lines
line(levelCH(:,1), levelCH(:,2), 'Color', 'b', 'LineStyle', '-', 'LineWidth', 1);
line(levelCHLow(:,1), levelCHLow(:,2), 'Color', 'b', 'LineStyle', '-.', 'LineWidth', 1);
line(levelCHUp(:,1), levelCHUp(:,2), 'Color', 'b', 'LineStyle', '-.', 'LineWidth', 1);

% Set Properties
grid on; xlim([-220 -90]); ylim([1 10]);
xlabel('Phase at Pilot Bandwidth (deg)', 'Interpreter', 'none');
ylabel('Average Cooper-Harper Rating', 'Interpreter', 'none');

% Add title to the plot
title(plotTitle, 'Interpreter', 'none');

% Add text
if ~isempty('plotText')
    text(...
        'Units', 'normalized', ...
        'Position', [0.99, 0.95], ...
        'String', plotText, ...
        'BackgroundColor', [1 1 1], ...
        'EdgeColor', [0 0 0], ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', ...
        'Interpreter', 'none');
end

% Return the figure handle object
figHandle = gcf;


%% Check Outputs

