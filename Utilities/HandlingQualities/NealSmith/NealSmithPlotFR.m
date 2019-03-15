function [figHandle] = NealSmithPlotFR(freq_rps, gain_dB, phase_deg, levelBound, typePlot, plotTitle, plotText)
% Plot the Neal/Smith Close-Loop Frequency Response
%
%Usage: [figHandle] = NealSmithPlotFR(freq_rps, gain_dB, phase_deg, levelBound, typePlot, plotTitle, plotText);
%
%Inputs:
% freq_rps   - frequencies corresponding to gain and phase (rad/sec)
% gain_dB    - magnitude of frequency response (dB)
% phase_deg  - phase of frequency response (deg)
% levelBound - level boundary []
% typePlot   - 'gain' or 'phase' plot ['gain']
% plotTitle  - plot title ['Neal-Smith - Closed-Loop Response Requirements']
% plotText   - plot text []
%
%Outputs:
% figHandle - handle object for the plot
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 7, nargin, 'struct'))
if nargin < 7, plotText = [];
    if nargin < 6, plotTitle = []; end
    if nargin < 5, typePlot = []; end
    if nargin < 4, levelBound = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(typePlot), typePlot = 'gain'; end
if isempty(plotTitle), plotTitle = 'Neal-Smith - Closed-Loop Response Requirements'; end


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);


%% Plot
switch typePlot
    case 'gain'
        % Plot data points
        semilogx(freq_rps, gain_dB, '-*b');

        % Plot Boundary lines
        line(levelBound(:,1), levelBound(:,2), 'Color', 'r', 'LineWidth', 1);

        % Set Properties
        grid on; xlim([0.1 100]); ylim([-6 10]);
        xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
        ylabel('Gain (dB)', 'Interpreter', 'none');

    case 'phase'
        % Plot data points
        semilogx(freq_rps, phase_deg, '-*b');

        % Plot Boundary lines
        line(levelBound(:,1), levelBound(:,2), 'Color', 'r', 'LineWidth', 1);

        % Set Properties
        grid on; xlim([0.1 100]); ylim([-180 20]);
        xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
        ylabel('Phase (deg)', 'Interpreter', 'none');

    otherwise
        warning('') % FIXME: blank warning
end

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
