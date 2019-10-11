function [figHandle] = NyquistPlot(frf, optPlot)
% Plot the Real and Imaginary portion of a TF, also plot Coherence.
%
%Usage:  [figHandle] = NyquistPlot(frf, optPlot);
%
%Inputs:
% frf
%   freq     - frequency of response
%   T        - gain of response (dB)
%   coher    - coherence of response []
% optPlot
%   coherPlot  - plot Coherence [false]
%   title      - title of plot ['Nyquist Plot']
%   saveFile   - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure

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

if ~isfield(optPlot, 'coherPlot'), optPlot.coherPlot = []; end
if isempty(optPlot.coherPlot), optPlot.coherPlot = true; end

if ~isfield(optPlot, 'title'), optPlot.title = []; end
if isempty(optPlot.title), optPlot.title = 'Nyquist Plot'; end

r2d = 180/pi;
d2r = 1/r2d;
hz2rps = 2*pi;
rps2hz = 1/hz2rps;



%% Check Inputs
if ~isfield(frf, 'coher')
    frf.coher = [];
end

switch lower(optPlot.freqUnits)
    case 'hz'
        freq = frf.freq * rps2hz;
    otherwise
        freq = frf.freq;
end

xyT = frf.T;
xyC = frf.coher;


%% Plot
% New figure with handle
figHandle = figure;

% Set number of plots
if optPlot.coherPlot
    numPlots = 3;
else
    numPlots = 2;
end

% Nyquist Plot
subplot(numPlots, 1, 1:2);
plot(real(xyT)', imag(xyT)'); grid on;
xlim('auto'); ylim('auto');
xlabel('Real Component');
ylabel('Imaginary Component');
title(optPlot.title, 'Interpreter', 'none');

% Coherence Plot
if ~isempty(xyC)
    subplot(numPlots, 1, 3);

    plot(freq, xyC); grid on;
    
    if strcmp(optPlot.freqScale, 'log')
        set(gca,'XScale','log');
    end
    
    xlim([min(freq), max(freq)]);
    xlabel(['Frequency (' optPlot.freqUnits ')']);
    ylabel('Coherence');
end


%% Save the plot as a fig
if isfield (optPlot, 'saveFile')
    saveas(figHandle, optPlot.saveFile, 'fig');
end

