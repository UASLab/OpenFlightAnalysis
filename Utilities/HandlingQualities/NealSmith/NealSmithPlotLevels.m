function [figHandle] = NealSmithPlotLevels(pilotLead_deg, resPeak_dB, level1Bound, level2Bound, level, plotTitle, plotText)
% Plot the Neal/Smith requirments with boundaries.
%
%Usage: [figHandle] = NealSmithPlotLevels(pilotLead_deg, resPeak_dB, level1Bound, level2Bound, level, plotTitle, plotText);
%
%Inputs:
% pilotLead_deg - pilot phase Lead (deg)
% resPeak_dB    - resonance peak resulting (dB)
% level1Bound   - level 1 bound []
% level2Bound   - level 2 bound []
% level         - level rating [0]
% plotTitle     - plot title ['Neal-Smith Requirements']
% plotText      - plot text []
%
%Outputs:
% figHandle - handle object for the plot
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(2, 7, nargin, 'struct'))
if nargin < 7, plotText = [];
    if nargin < 6, plotTitle = []; end
    if nargin < 5, level = []; end
    if nargin < 4, level2Bound = []; end
    if nargin < 3, level1Bound = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(level), level = zeros(length(freqBW_rps), 1); end
if isempty(plotTitle), plotTitle = 'Neal-Smith Requirements'; end


%% Check Inputs
% Inputs as column vectors
pilotLead_deg = pilotLead_deg(:);
resPeak_dB = resPeak_dB(:);


%% Plot
% Plot data points
plot(...
    pilotLead_deg(level==2), resPeak_dB(level==2), '*m', ...
    pilotLead_deg(level==1), resPeak_dB(level==1), '*r', ...
    pilotLead_deg(level==0), resPeak_dB(level==0), '*k');

% Plot Boundary lines
handleLine2 = line(level2Bound(:,1), level2Bound(:,2), 'Color', 'm', 'LineWidth', 1);
handleLine1 = line(level1Bound(:,1), level1Bound(:,2), 'Color', 'r', 'LineWidth', 1);

% Set Properties
grid on; xlim([-30 90]); ylim([0 12]);
xlabel('Lead Compensation (deg)', 'Interpreter', 'none');
ylabel('Resonance Peak (dB)', 'Interpreter', 'none');
legend([handleLine2, handleLine1], {'Level2', 'Level1'}, 'Interpreter', 'none');

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
