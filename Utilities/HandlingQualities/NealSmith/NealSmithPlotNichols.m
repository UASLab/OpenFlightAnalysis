function [figHandle] = NealSmithPlotNichols(freq_rps, gain_dB, phase_deg, gainOL_dB, phaseOL_deg, freqBW_rps, droop_dB, plotTitle, plotText)
% Plot the Neal/Smith Nichols chart.
%
%Usage:  [figHandle] = NealSmithPlotNichols(freq_rps, gain_dB, phase_deg, gainOL_dB, phaseOL_deg, freqBW_rps, droop_dB, plotTitle, plotText);
%
%Inputs:
% freq_rps    - frequencies corresponding to gain and phase (rad/sec)
% gain_dB     - magnitude of frequency response (dB)
% phase_deg   - phase of frequency response (deg)
% gainOL_dB   - magnitude of open-loop frequency response (dB)
% phaseOL_deg - phase of open-loop frequency response (deg)
% freqBW_rps  - bandwidth frequency (rad/sec)
% droop_dB    - droop value (dB)
% plotTitle   - plot title ['Neal-Smith Requirements']
% plotText    - plot text []
%
%Outputs:
% figHandle - Handle object for the plot
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(7, 9, nargin, 'struct'))
if nargin < 9, plotText = [];
    if nargin < 8, plotTitle = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(plotTitle), plotTitle = 'Neal-Smith Requirements'; end

r2d = 180/pi;
imaj = sqrt(-1);


%% Check Inputs


%% TODO: Explanation required
% Calculate the gain and phase at the bandwidth frequency
indxBW = min(find(freq_rps >= freqBW_rps));
gainBW_dB = interp1(freq_rps, gain_dB, freqBW_rps);
phaseBW_deg = interp1(freq_rps, phase_deg, freqBW_rps);

% Create boundaries
phaseRange_rad(:,1) = linspace(0, -pi/2, 50);
droopCL = 10 ^ (droop_dB/20) * (cos(phaseRange_rad) + imaj*sin(phaseRange_rad));
droopOL = droopCL ./ (1 - droopCL);
gainDroopOL_dB = 20*log10(abs(droopOL));
phaseDroopOL_deg = angle(droopOL) * r2d;

gainRange_dB(:,1) = linspace(6, droop_dB, 50);
CL90 = 10 .^ (gainRange_dB/20) * (-imaj);
OL90 = CL90 ./ (1 - CL90);
gainOL90_dB = 20*log10(abs(OL90));
phaseOL90_dB = angle(OL90) * r2d;


%% Plot
% Plot data points
plot(...
    phaseOL_deg, gainOL_dB, '-b', ...
    phaseOL_deg(indxBW), gainOL_dB(indxBW), '*b', ...
    phase_deg, gain_dB, '--r', ...
    phaseBW_deg, gainBW_dB, '*r', ...
    phaseOL90_dB, gainOL90_dB, '-g', ...
    phaseOL90_dB, (gainOL90_dB - 0.5), '--g', ...
    phaseDroopOL_deg, gainDroopOL_dB, '-g', ...
    phaseDroopOL_deg, (gainDroopOL_dB - 0.5), '--g');

% Plot Boundary lines

ngrid; xlim('auto'); ylim('auto');
xlabel('Loop Phase (deg)', 'Interpreter', 'none');
ylabel('Loop Gain (dB)', 'Interpreter', 'none');

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
hold off; figHandle = gcf;


%% Check Outputs
