function [figHandle] = BodePlot(frf, optPlot)
% Plot the Gain, Phase, and Coherence versus frequency.
%
%Usage:  [figHandle] = BodePlot(frf, optPlot);
%
%Inputs:
% frf
%   freq       - frequency of response
%   Gain_dB    - gain of response (dB)
%   Phase_deg  - Phase of response (deg)
%   xyC        - Coherenceence of response []
% optPlot
%   Title      - Title of plot ['Bode Plot']
%   saveFile   - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
%
%Notes:
% gain and Phase will be computed if xyT is provided
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
if ~isfield(optPlot, 'FreqUnits'), optPlot.FreqUnits = []; end
if isempty(optPlot.FreqUnits), optPlot.FreqUnits = ''; end

if ~isfield(optPlot, 'FreqScale'), optPlot.FreqScale = []; end
if isempty(optPlot.FreqScale), optPlot.FreqScale = 'log'; end

if ~isfield(optPlot, 'GainUnits'), optPlot.GainUnits = []; end
if isempty(optPlot.GainUnits), optPlot.GainUnits = 'dB'; end

if ~isfield(optPlot, 'GainScale'), optPlot.GainScale = []; end
if isempty(optPlot.GainScale), optPlot.GainScale = 'linear'; end

if ~isfield(optPlot, 'PhaseUnits'), optPlot.PhaseUnits = []; end
if isempty(optPlot.PhaseUnits), optPlot.PhaseUnits = 'deg'; end

if ~isfield(optPlot, 'PhaseUnwrap'), optPlot.PhaseUnwrap = []; end
if isempty(optPlot.PhaseUnwrap), optPlot.PhaseUnwrap = false; end

if ~isfield(optPlot, 'Title'), optPlot.Title = []; end
if isempty(optPlot.Title), optPlot.Title = 'Bode Plot'; end

r2d = 180/pi;
d2r = 1/r2d;
hz2rps = 2*pi;
rps2hz = 1/hz2rps;

%% Check Inputs

if ~isfield(frf, 'Gain_dB') || ~isfield(frf, 'Phase_deg')
    [frf.Gain_dB, frf.Phase_deg] = GainPhase(frf.FRD);
end

if ~isfield(frf, 'Coher')
    frf.Coherence = [];
end

% Recursive Call
if length(size(frf.Gain_dB)) == 3
   [~, numIn, ~] = size(frf.Gain_dB);

    figHandle = cell(1, numIn);
   
    for iIn = 1:numIn
        tempFrf.freq = frf.freq;
        tempFrf.Gain_dB = squeeze(frf.Gain_dB(:,iIn,:));
        tempFrf.Phase_deg = squeeze(frf.Phase_deg(:,iIn,:));
        if ~isempty(frf.Coherence)
            tempFrf.Coherence = squeeze(frf.Coherence(:,iIn,:));
        end
        
        figHandle{iIn} = BodePlot(tempFrf, optPlot);
        
    end
    
    return
end

switch lower(optPlot.FreqUnits)
    case 'hz'
        Freq = frf.Frequency * rps2hz;
    otherwise
        Freq = frf.Frequency;
end

switch lower(optPlot.GainUnits)
    case 'mag'
        Gain = DB2Mag(frf.Gain_dB);
    otherwise
        Gain = frf.Gain_dB;
end


switch lower(optPlot.PhaseUnits)
    case 'rad'
        Phase = frf.Phase_deg * d2r;
    otherwise
        Phase = frf.Phase_deg;
end

%% Plot
% New figure with handle
figHandle = figure;

% Set number of plots
if ~isempty(frf.Coherence)
    numPlots = 3;
else
    numPlots = 2;
end

% Gain Plot
subplot(numPlots, 1, 1); grid on;
plot(Freq, Gain); grid on;

if strcmp(optPlot.FreqScale, 'log')
    set(gca,'XScale','log');
end
if strcmp(optPlot.GainScale, 'log')
    set(gca,'YScale','log');
end

ylim('auto');
ylabel(['Gain (' optPlot.GainUnits ')']);
Title(optPlot.Title, 'Interpreter', 'none');

% Phase Plot
subplot(numPlots, 1, 2);
plot(Freq, Phase); grid on;

if strcmp(optPlot.FreqScale, 'log')
    set(gca,'XScale','log');
end

ylim('auto');
ylabel(['Phase (' optPlot.PhaseUnits ')']);

% Coherence Plot
if ~isempty(frf.Coherence)
    subplot(numPlots, 1, 3);
    plot(Freq, frf.Coherence); grid on;
    
    if strcmp(optPlot.FreqScale, 'log')
        set(gca,'XScale','log');
    end
    
    ylabel('Coherence');
end

% Add Common x-axis label, and link the axes together and apply tight limits
subplot(numPlots, 1, numPlots);
xlabel(['Frequency (' optPlot.FreqUnits ')']);
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

xlim([min(Freq), max(Freq)]);

%% Save the plot as a fig
if isfield (optPlot, 'SaveFile')
    saveas(figHandle, optPlot.SaveFile, 'fig');
end


%% Check Outputs
