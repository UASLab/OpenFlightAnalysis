function [figHandle] = BodePlot(frf, optPlot)
% Plot the Gain, Phase, and Coherence versus frequency.
%
%Usage:  [figHandle] = BodePlot(frf, optPlot);
%
%Inputs:
% frf
%   freq       - frequency of response
%   gain_dB    - gain of response (dB)
%   phase_deg  - phase of response (deg)
%   xyC        - coherence of response []
% optPlot
%   title      - title of plot ['Bode Plot']
%   saveFile   - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
%
%Notes:
% gain and phase will be computed if xyT is provided
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 2);
if nargin < 2, optPlot = []; end

if isempty(optPlot), optPlot = struct(); end

nargoutchk(0, 1);


%% Default Values and Constants
if ~isfield(optPlot, 'freqUnits'), optPlot.freqUnits = []; end
if isempty(optPlot.freqUnits), optPlot.freqUnits = ''; end

if ~isfield(optPlot, 'freqScale'), optPlot.freqScale = []; end
if isempty(optPlot.freqScale), optPlot.freqScale = 'log'; end

if ~isfield(optPlot, 'gainUnits'), optPlot.gainUnits = []; end
if isempty(optPlot.gainUnits), optPlot.gainUnits = 'dB'; end

if ~isfield(optPlot, 'gainScale'), optPlot.gainScale = []; end
if isempty(optPlot.gainScale), optPlot.gainScale = 'linear'; end

if ~isfield(optPlot, 'phaseUnits'), optPlot.phaseUnits = []; end
if isempty(optPlot.phaseUnits), optPlot.phaseUnits = 'deg'; end

if ~isfield(optPlot, 'phaseUnwrap'), optPlot.phaseUnwrap = []; end
if isempty(optPlot.phaseUnwrap), optPlot.phaseUnwrap = false; end

if ~isfield(optPlot, 'title'), optPlot.title = []; end
if isempty(optPlot.title), optPlot.title = 'Bode Plot'; end

r2d = 180/pi;
d2r = 1/r2d;
hz2rps = 2*pi;
rps2hz = 1/hz2rps;

%% Check Inputs
if ~isfield(frf, 'coher')
    frf.coher = [];
end

if ~isfield(frf, 'gain_dB') || ~isfield(frf, 'phase_deg')
    [frf.gain_dB, frf.phase_deg] = GainPhase(frf.T);
end

switch lower(optPlot.freqUnits)
    case 'hz'
        freq = frf.freq * rps2hz;
    otherwise
        freq = frf.freq;
end

switch lower(optPlot.gainUnits)
    case 'mag'
        gain = DB2Mag(frf.gain_dB);
    otherwise
        gain = frf.gain_dB;
end


switch lower(optPlot.phaseUnits)
    case 'rad'
        phase = frf.phase_deg * d2r;
    otherwise
        phase = frf.phase_deg;
end

%% Plot
% New figure with handle
figHandle = figure;

% Set number of plots
if ~isempty(frf.coher)
    numPlots = 3;
else
    numPlots = 2;
end

% Gain Plot
subplot(numPlots, 1, 1); grid on;
plot(freq, gain); grid on;

if strcmp(optPlot.freqScale, 'log')
    set(gca,'XScale','log');
end
if strcmp(optPlot.gainScale, 'log')
    set(gca,'YScale','log');
end

ylim('auto');
ylabel(['Gain (' optPlot.gainUnits ')']);
title(optPlot.title, 'Interpreter', 'none');

% Phase Plot
subplot(numPlots, 1, 2);
plot(freq, phase); grid on;

if strcmp(optPlot.freqScale, 'log')
    set(gca,'XScale','log');
end

ylim('auto');
ylabel(['Phase (' optPlot.phaseUnits ')']);

% Coherence Plot
if ~isempty(frf.coher)
    subplot(numPlots, 1, 3);
    plot(freq, frf.coher); grid on;
    
    if strcmp(optPlot.freqScale, 'log')
        set(gca,'XScale','log');
    end
    
    ylabel('Coherence');
end

% Add Common x-axis label, and link the axes together and apply tight limits
subplot(numPlots, 1, numPlots);
xlabel(['Frequency (' optPlot.freqUnits ')']);
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

xlim([min(freq), max(freq)]);

%% Save the plot as a fig
if isfield (optPlot, 'saveFile')
    saveas(figHandle, optPlot.saveFile, 'fig');
end


%% Check Outputs
