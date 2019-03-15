function [figHandle] = LoesPlot(freq_rps, gain_dB, phase_deg, gainLOES_dB, phaseLOES_deg, gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg, typeFR, plotTitle, plotText)
% Plot the frequency response, LOES fit, and MUAD bounds.
%
%Usage:  [figHandle] = LoesPlot(freq_rps, gain_dB, phase_deg, gainLOES_dB, phaseLOES_deg, gainLowBound_dB, phaseLowBound_deg, gainUpBound_dB, phaseUpBound_deg, typeFR, plotTitle, plotText);
%
%Inputs:
% freq_rps          - frequency of frequency response (rad/sec)
% gain_dB           - magnitude of frequency response (dB)
% phase_deg         - phase of frequency response (deg)
% gainLOES_dB       - magnitude of LOES fit frequency response (dB)
% phaseLOES_deg     - phase of LOES fit frequency response (deg)
% gainLowBound_dB   - lower gain bound frequency response (dB)
% phaseLowBound_deg - lower phase bound frequency response (deg)
% gainUpBound_dB    - upper gain bound frequency response (dB)
% phaseUpBound_deg  - upper phase bound frequency response (deg)
% typeFR            - 'absolute' or 'relative' LOES plot ['absolute']
% plotTitle         - plot title []
% plotText          - text to place on plot (cell array) []
%
%Outputs:
% figHandle - handle of the figure
%
%Notes:
% 
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(9, 12, nargin, 'struct'))
if nargin < 12, plotText = [];
    if nargin < 11, plotTitle = []; end
    if nargin < 10, typeFR = 'absolute'; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);
gain_dB = gain_dB(:);
phase_deg = phase_deg(:);
gainLOES_dB = gainLOES_dB(:);
phaseLOES_deg = phaseLOES_deg(:);
gainLowBound_dB = gainLowBound_dB(:);
phaseLowBound_deg = phaseLowBound_deg(:);
gainUpBound_dB = gainUpBound_dB(:);
phaseUpBound_deg = phaseUpBound_deg(:);

% Plot text can be 2X1 cell array text (text can be a cell array also)
if iscell(plotText)
    plotText1 = plotText{1};
    plotText2 = plotText{2};
else
    plotText1 = plotText;
    plotText2 = plotText;
end


%% Two plot types, 'absolute' is most common
switch typeFR
    case 'absolute'
        % Adjust the bounds to be based on the LOES fit
        gainLowBound_dB = gainLowBound_dB + gainLOES_dB;
        phaseLowBound_deg = phaseLowBound_deg + phaseLOES_deg - freq_rps*.0072; %???

        gainUpBound_dB = gainUpBound_dB + gainLOES_dB;
        phaseUpBound_deg = phaseUpBound_deg + phaseLOES_deg + freq_rps*.006; %???
    case 'relative'
        % Adjust the frequency response to HOS-LOES
        gain_dB = gain_dB - gainLOES_dB;
        phase_deg = phase_deg - phaseLOES_deg;

        gainLOES_dB = gainLOES_dB - gainLOES_dB;
        phaseLOES_deg = phaseLOES_deg - phaseLOES_deg;
    otherwise
        warning('')
end


%% Gain plot
subplot(2, 1, 1);

% Plot data points
semilogx(...
    freq_rps, gain_dB, '-k', ...
    freq_rps, gainLOES_dB, '--b', ...
    freq_rps, gainUpBound_dB, '-.r', ...
    freq_rps, gainLowBound_dB, '-.r');

% Set Properties
grid on; xlim([min(freq_rps), max(freq_rps)]); ylim('auto');
xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
ylabel('Gain (dB)', 'Interpreter', 'none');


%% Phase plot
subplot(2, 1, 2);

% Plot data points
semilogx(...
    freq_rps, phase_deg, '-k', ...
    freq_rps, phaseLOES_deg, '--b', ...
    freq_rps, phaseUpBound_deg, '-.r', ...
    freq_rps, phaseLowBound_deg, '-.r');

% Set Properties
grid on; xlim([min(freq_rps), max(freq_rps)]); ylim('auto');
xlabel('Frequency (rad/sec)', 'Interpreter', 'none');
ylabel('Phase (deg)', 'Interpreter', 'none');


%% Add plot title
subplot(2, 1, 1);
title(plotTitle, 'Interpreter', 'none');


%% Add plot text
if ~isempty('plotText1')

    % Normalized text position
    posText = [0.98, 0.98];

    subplot(2, 1, 1);
    text(...
        'Units', 'normalized', ...
        'Position', posText, ...
        'String', plotText1, ...
        'BackgroundColor', [1 1 1], ...
        'EdgeColor', [0 0 0], ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', ...
        'Interpreter', 'none');
end
if ~isempty('plotText2')

    % Normalized text position
    posText = [0.98, 0.98];

    subplot(2, 1, 2);
    text(...
        'Units', 'normalized', ...
        'Position', posText, ...
        'String', plotText2, ...
        'BackgroundColor', [1 1 1], ...
        'EdgeColor', [0 0 0], ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', ...
        'Interpreter', 'none');
end


%% Return the figure handle object
figHandle = gcf;


%% Check Outputs
