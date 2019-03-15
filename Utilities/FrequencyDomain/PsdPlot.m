function [figHandle] = PsdPlot(freq, magPsd, freqUnits, plotTitle, saveFile)
% Plot the Pseudo-Spectral-Density.
%
%Inputs:
% freq      - frequency of response
% magPsd    - magnitude of the PSD
% freqUnits - units of the frequency vector for plotting ['']
% plotTitle - title of plot ['PSD Plot']
% saveFile  - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
%
%Notes:
% 
%
%Dependency:
% Mag2DB
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release
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
    magPsd = magPsd';
    %[widthFreq, lenFreq] = size(freq);
    %[widthPsd, lenPsd] = size(magPsd);
end


%% Plot
figure;
plot(freq', Mag2DB(magPsd'));
semilogx(freq', Mag2DB(magPsd'));
grid on; xlabel(['Frequency (' freqUnits ')']); ylabel('Amplitude (dB)');
title(plotTitle, 'Interpreter', 'none');

% figure handle
figHandle = gcf;


%% Save the plot as a fig
if saveFile
    saveas(figHandle, saveFile, 'fig');
end


%% Check Outputs
