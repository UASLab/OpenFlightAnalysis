function [figHandle] = NyquistPlot(freq_rps, xyT, xyC, coherLimit, plotTitle, saveFile)
% Plot the Real and Imaginary portion of a TF, also plot Coherence.
%
%Usage:  [figHandle] = NyquistPlot(freq_rps, xyT, xyC, coherLimit, plotTitle, saveFile);
%
%Inputs:
% freq_rps   - frequency of response (rad/sec)
% xyT        - complex transfer function
% xyC        - coherence of response []
% coherLimit - coherence threshold []
% plotTitle  - title of plot ['Nyquist Plot']
% saveFile   - file to save figure []
%
%Outputs:
% figHandle - object handle of the figure
%
%Notes:
% 
%

%Version History: Version 1.5
% 08/31/2006  C. Regan     Initial Release (v1.0)
% 09/15/2006  C. Regan     added color input, changed notation (v1.1)
% 09/21/2006  C. Regan     made coherence limit plot optional, removed default value (v1.2)
% 09/26/2006  C. Regan     Nyquist plot takes up 2/3 or 2/2 axes positions (v1.3)
% 09/27/2006  C. Regan     removed color input
%                          use 'set(0, 'DefaultAxesColorOrder',...) in higher level (v1.4)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.5)
%


%% Check I/O Arguments
error(nargchk(2, 6, nargin, 'struct'))
if nargin < 6, saveFile = [];
    if nargin < 5, plotTitle = []; end
    if nargin < 4, coherLimit = []; end
    if nargin < 3, xyC = []; end
end

error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
if isempty(plotTitle), plotTitle = 'Nyquist Plot'; end


%% Check Inputs


%% Plot
% Set number of plots
if ~isempty(xyC)
    numPlots = 3;
else
    numPlots = 2;
end

% Min and Max Freq axis for plotting
plotFreqMin = min(freq_rps);
plotFreqMax = max(freq_rps);

% Nyquist Plot
figure;
subplot(numPlots, 1, 1:2);
plot(real(xyT), imag(xyT));
grid on; xlim('auto'); ylim('auto');
xlabel('Real Component');
ylabel('Imaginary Component');
title(plotTitle, 'Interpreter', 'none');

% Coherence Plot
if ~isempty(xyC)
    subplot(numPlots, 1, 3);
    semilogx(freq_rps, xyC);
    grid on; xlim([plotFreqMin, plotFreqMax]);
    xlabel('Frequency (rad/sec)'); ylabel('Coherence');
    
    % Min and Max Coherence axis for plotting
    if ~isempty(coherLimit) % coherLimit defined
        plotCoherMin = coherLimit - (1 - coherLimit);
        plotCoherMax = 1;
    
        % Coherence theshold line
        line([plotFreqMin plotFreqMax], [coherLimit, coherLimit], 'LineStyle', '--');
    
    else % coherLimit not defined
        plotCoherMin = max(min(xyC), 0);
        plotCoherMax = 1;
    end
    
    % Coherence plot limits
    ylim([plotCoherMin, plotCoherMax]);
end

% figure handle
figHandle = gcf;


%% Save the plot as a fig
if saveFile
    saveas(figHandle, saveFile, 'fig');
end


%% Check Inputs
