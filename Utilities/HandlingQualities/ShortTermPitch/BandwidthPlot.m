function [figHandle] = BandwidthPlot(freqBW_rps, tDelay_s, level1Bound, level2Bound, level3Bound, level, plotTitle, plotText)
% Plot the bandwidth requirements with boundaries.
%
%Usage:  [figHandle] = BandwidthPlot(freqBW_rps, tDelay_s, level1Bound, level2Bound, level3Bound, level, plotTitle, plotText)
%
%Inputs:
% freqBW_rps  - bandwidth of 'theta/dep' transfer function (rad/sec)
% tDelay_s    - estimated equivalent time delay (sec)
% level1Bound - level 1 bound []
% level2Bound - level 2 bound []
% level3Bound - level 3 bound []
% level       - level rating [0]
% plotTitle   - plot title ['Bandwidth Requirements']
% plotText    - plot text []
%
%Outputs:
% figHandle - handle object for the plot
%
%Notes:
% 
%
%Reference:
% "Bandwidth - A Criterion for Highly Augmented Airplanes"  Hoh, Mitchell, Hodgkinson
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(2, 8, nargin, 'struct'))
if nargin < 8, plotText = [];
    if nargin < 7, plotTitle = []; end
    if nargin < 6, level = []; end
    if nargin < 5, level3Bound = []; end
    if nargin < 4, level2Bound = []; end
    if nargin < 3, level1Bound = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(level), level = zeros(length(freqBW_rps), 1); end
if isempty(plotTitle), plotTitle = 'Bandwidth Requirements'; end


%% Check Inputs
% Inputs as column vectors
freqBW_rps = freqBW_rps(:);
tDelay_s = tDelay_s(:);
level = level(:);


%% Plot
% Plot data points
plot(...
    freqBW_rps(level==3), tDelay_s(level==3), '*b',...
    freqBW_rps(level==2), tDelay_s(level==2), '*m',...
    freqBW_rps(level==1), tDelay_s(level==1), '*r',...
    freqBW_rps(level==0), tDelay_s(level==0), '*k');

% Plot Boundary lines
handleLine3 = line(level3Bound(:,1), level3Bound(:,2), 'Color', 'b', 'LineWidth', 1);
handleLine2 = line(level2Bound(:,1), level2Bound(:,2), 'Color', 'm', 'LineWidth', 1);
handleLine1 = line(level1Bound(:,1), level1Bound(:,2), 'Color', 'r', 'LineWidth', 1);

% Set Properties
grid on; xlim('auto'); ylim('auto');
xlabel('Bandwidth Frequency (rad/sec)', 'Interpreter', 'none');
ylabel('Time Delay (sec)', 'Interpreter', 'none');
legend([handleLine3, handleLine2, handleLine1], {'Level3', 'Level2', 'Level1'}, 'Interpreter', 'none');

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
