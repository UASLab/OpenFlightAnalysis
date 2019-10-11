function [figHandle] = SpectPlot(xSpect, optPlot)
% Plot the Pseudo-Spectral-Density.
%
%Inputs:
% xSpect
%   freq      - frequency of response
%   P         - magnitude of the response spectrum
% optPlot
%   freqUnits - units of the frequency vector for plotting ['']
%   powerUnits - units of the Power vector for plotting ['']
%   plotTitle - title of plot ['PSD Plot']
%   saveFile  - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
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

if ~isfield(optPlot, 'powerUnits'), optPlot.powerUnits = []; end
if isempty(optPlot.powerUnits), optPlot.powerUnits = 'dB'; end

if ~isfield(optPlot, 'powerScale'), optPlot.powerScale = []; end
if isempty(optPlot.powerScale), optPlot.powerScale = 'linear'; end

if ~isfield(optPlot, 'title'), optPlot.title = []; end
if isempty(optPlot.title), optPlot.title = 'Power Spectrum Plot'; end


hz2rps = 2*pi;
rps2hz = 1/hz2rps;

%% Check Inputs
switch lower(optPlot.freqUnits)
    case 'hz'
        freq = xSpect.freq * rps2hz;
    otherwise
        freq = xSpect.freq;
end
        
switch lower(optPlot.powerUnits)
    case 'db'
        P = Mag2DB(xSpect.P);
    otherwise
        P = xSpect.P;
end



%% Plot
% New figure with handle
figHandle = figure;

plot(freq, P); grid on;

if strcmp(optPlot.freqScale, 'log')
    set(gca,'XScale','log');
end
if strcmp(optPlot.powerScale, 'log')
    set(gca,'YScale','log');
end

xlabel(['Frequency (' optPlot.freqUnits ')']);
ylabel(['Power (' optPlot.powerUnits ')']);
title(optPlot.title, 'Interpreter', 'none');

% figure handle
figHandle = gcf;


%% Save the plot as a fig
if isfield (optPlot, 'saveFile')
    saveas(figHandle, optPlot.saveFile, 'fig');
end

