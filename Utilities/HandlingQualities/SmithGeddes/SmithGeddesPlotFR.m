function [figHandle] = SmithGeddesPlotFR(freq_rps, gain_dB, phase_deg, freqBW_rps, gainFitPoints, typePlot, typeFR, plotTitle, plotText)
% Plot the Smith/Geddes Criterion frequency response.
%
%Usage:  [figHandle] = SmithGeddesPlotFR(freq_rps, gain_dB, phase_deg, freqBW_rps, gainFitPoints, typePlot, typeFR, plotTitle, plotText);
%
%Inputs:
% freq_rps      - frequency of the frequency response (rad/sec)
% gain_dB       - magnitude of the frequency response (dB)
% phase_deg     - phase of frequency responce (deg)
% freqBW_rps    - bandwidth frequency of the pilot response (rps)
% gainFitPoints - start and end point of the gain fit (rps,dB)
% typePlot      - plot type 'gain' or 'phase' ['gain']
% typeFR        - frequency response 'long' or 'latdir' ['long']
% plotTitle     - plot title ['Smith-Geddes Requirements']
% plotText      - plot text []
%
%Outputs:
% figHandle - handle object for the plot
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(5, 9, nargin, 'struct'))
if nargin < 9, plotText = [];
    if nargin < 8, plotTitle = []; end
    if nargin < 7, typeFR = []; end
    if nargin < 6, typePlot = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(typePlot), typePlot = 'gain'; end
if isempty(typeFR), typeFR = 'long'; end
if isempty(plotTitle), plotTitle = 'Smith-Geddes Requirements'; end


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);


%% Calculate the gain and phase at the bandwidth frequency
gainBW_dB = interp1(freq_rps, gain_dB, freqBW_rps);
phaseBW_deg = interp1(freq_rps, phase_deg, freqBW_rps);


%% Plot
switch typePlot
    case 'gain'
        % Plot data points
        semilogx(...
            freq_rps, gain_dB, '-k', ...
            freqBW_rps, gainBW_dB, '*k', ...
            gainFitPoints(:,1), gainFitPoints(:,2), '-b');

        % Set Properties
        grid on; xlim([1 15]); ylim('auto');
        xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
        ylabel('Gain (dB)', 'Interpreter', 'none');

    case 'phase'
        % Plot data points
        semilogx(...
            freq_rps, phase_deg, '-k', ...
            freqBW_rps, phaseBW_deg, '*k');

        % Set Properties
        grid on; xlim([1 15]); ylim('auto');
        xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
        ylabel('Phase (deg)', 'Interpreter', 'none');

    otherwise
        warning(''); %FIXME: blank warning
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
