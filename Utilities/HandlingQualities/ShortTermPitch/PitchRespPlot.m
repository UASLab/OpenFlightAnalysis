function [figHandle] = PitchRespPlot(dampSP, thetaLag_rad, level1Bound, level2Bound, level3Bound, level, plotTitle, plotText)
% Plot the pitch response requirements with boundaries.
%
%Usage:  [figHandle] = PitchRespPlot(dampSP, thetaLag_rad, level1Bound, level2Bound, level3Bound, level, plotTitle, plotText)
%
%Inputs:
% dampSP       - short period damping
% thetaLag_rad - lag between flight path and pitch attitude response (rad)
% level1Bound  - Level 1 boundary []
% level2Bound  - Level 2 boundary []
% level3Bound  - Level 3 boundary []
% level        - level achieved [0]
% plotTitle    - plot title ['Pitch Response Requirements']
% plotText     - plot text []
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
if isempty(plotTitle), plotTitle = 'Pitch Response Requirements'; end


%% Check Inputs
% Inputs as column vectors
dampSP = dampSP(:);
thetaLag_rad = thetaLag_rad(:);
level = level(:);


%% Plot
% Plot data points
loglog(...
    dampSP(level==3), thetaLag_rad(level==3), '*b',...
    dampSP(level==2), thetaLag_rad(level==2), '*m',...
    dampSP(level==1), thetaLag_rad(level==1), '*r',...
    dampSP(level==0), thetaLag_rad(level==0), '*k');

% Plot Boundary lines
handleLine3 = line(level3Bound(:,1), level3Bound(:,2), 'Color', 'b', 'LineWidth', 1);
handleLine2 = line(level2Bound(:,1), level2Bound(:,2), 'Color', 'm', 'LineWidth', 1);
handleLine1 = line(level1Bound(:,1), level1Bound(:,2), 'Color', 'r', 'LineWidth', 1);

% Set Properties
grid on; xlim('auto'); ylim('auto');
xlabel('Short-Period Damping Ratio', 'Interpreter', 'none');
ylabel('Wsp * T_theta2', 'Interpreter', 'none');
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
