function [figHandle] = ShortPeriodPlot(nzAlpha_gpr, freqSP_rps, level1Bound, level2Bound, level3Bound, level, plotTitle, plotText)
% Plot the short period requirements with boundaries.
%
%Usage:  [figHandle] = ShortPeriodPlot(nzAlpha_gpr, freqSP_rps, level, level3Bound, level2Bound, level1Bound, plotTitle, plotText)
%
%Inputs: nzAlpha_gpr - normal load per angle of attack (g/rad)
% freqSP_rps  - Short period frequency (rad/sec)
% level1Bound - level 1 bound []
% level2Bound - level 2 bound []
% level3Bound - level 3 bound []
% level       - level rating [0]
% plotTitle   - plot title ['Alternate Short-Period Requirements']
% plotText    - plot text []
%
%Outputs:
% figHandle - Handle object for the plot
%
%Notes:
% 
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2, Section A
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
if isempty(level), level = zeros(length(dampSP), 1); end
if isempty(plotTitle), plotTitle = 'Alternate Short-Period Requirements'; end


%% Check Inputs
% Inputs as column vectors
nzAlpha_gpr = nzAlpha_gpr(:);
freqSP_rps = freqSP_rps(:);
level = level(:);


%% Plot
% Plot data points
loglog(...
    nzAlpha_gpr(level==3), freqSP_rps(level==3), '*b',...
    nzAlpha_gpr(level==2), freqSP_rps(level==2), '*m',...
    nzAlpha_gpr(level==1), freqSP_rps(level==1), '*r',...
    nzAlpha_gpr(level==0), freqSP_rps(level==0), '*k');

% Plot Boundary lines
handleLine3 = line(level3Bound(:,1), level3Bound(:,2), 'Color', 'b', 'LineWidth', 1);
handleLine2 = line(level2Bound(:,1), level2Bound(:,2), 'Color', 'm', 'LineWidth', 1);
handleLine1 = line(level1Bound(:,1), level1Bound(:,2), 'Color', 'r', 'LineWidth', 1);

% Set Properties
grid on; xlim('auto'); ylim('auto');
xlabel('Nz/alpha (g/rad)', 'Interpreter', 'none');
ylabel('Short-Period Frequency (rad/sec)', 'Interpreter', 'none');
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
