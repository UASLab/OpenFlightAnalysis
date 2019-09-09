function [figHandle] = SpectPlot(freq, xxP, freqUnits, plotTitle, saveFile)
% Plot the Pseudo-Spectral-Density.
%
%Inputs:
% freq      - frequency of response
% xxP       - magnitude of the response spectrum
% freqUnits - units of the frequency vector for plotting ['']
% plotTitle - title of plot ['PSD Plot']
% saveFile  - file to save figure []
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
narginchk(2, 5);
if nargin < 5, saveFile = [];
    if nargin < 4, plotTitle = []; end
    if nargin < 3, freqUnits = []; end
end

nargoutchk(0, 1);


%% Default Values and Constants
if isempty(freqUnits), freqUnits = ''; end
if isempty(plotTitle), plotTitle = 'PSD Plot'; end


%% Check Inputs
[widthFreq, lenFreq] = size(freq);

% Transpose
if widthFreq > lenFreq
    freq = freq';
    xxP = xxP';
    %[widthFreq, lenFreq] = size(freq);
    %[widthPsd, lenPsd] = size(xxP);
end


%% Plot
figure;
plot(freq', Mag2DB(xxP'));
semilogx(freq', Mag2DB(xxP'));
grid on; xlabel(['Frequency (' freqUnits ')']); ylabel('Amplitude (dB)');
title(plotTitle, 'Interpreter', 'none');

% figure handle
figHandle = gcf;


%% Save the plot as a fig
if saveFile
    saveas(figHandle, saveFile, 'fig');
end


%% Check Outputs
