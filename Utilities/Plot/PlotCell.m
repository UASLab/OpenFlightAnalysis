function [figHand] = PlotCell(xList, yList, optionStruct)
% Create new plot and subplots based on the structure and fieldnames provided
%
%Inputs:
% xList - cell array list of names for the x axis
% yList - cell array list of names for the y axis
% optionStruct
%  (xNames) - cell array list of x-axis labels
%  (yNames) - cell array list of y-axis labels
%  (legNames) - cell array list of legend labels
%  (titleStr) - Plot Title string
%  (indxSlice) - index to of xList and yList data to plot
%
%Outputs:
% figHand - Figure Handle
%
%Notes:
%

%Version History: Version 0.1
% 09/23/2016  Name     Initial Release (v0.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v0.1)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 3);
if nargin < 3
    optionStruct = struct();
end

% Parse the Optional Input Structure, set Default Values
if isfield(optionStruct, 'xNames'), xNames = optionStruct.xNames; else xNames = cell(length(xList), 1); end
if isfield(optionStruct, 'yNames'), yNames = optionStruct.yNames; else yNames = cell(length(xList), 1);  end
if isfield(optionStruct, 'legNames'), legNames = optionStruct.legNames; else legNames = cell(length(xList), 1);  end
if isfield(optionStruct, 'titleStr'), titleStr = optionStruct.titleStr; else titleStr = ''; end
if isfield(optionStruct, 'indxSlice'), indxSlice = optionStruct.indxSlice; else indxSlice = NaN; end

% Check the number of outputs
nargoutchk(0, 1);


%% Check Inputs
% Catch the condition where xList is a single signal, expand to cell array
numX = length(xList);
numY = length(yList);
if (numX == 1)
    xList = repmat(xList, numY, 1);
    numX = length(xList);
end

% Catch the condition where indxSlice is not specified
if isnan(indxSlice)
    indxSlice = 1:length(xList{1});
end



%% Computation
numSub = length(yList);

figHand = figure;
for indxSub = 1:numSub
    subplot(numSub, 1, indxSub);
    numVar = length(yList{indxSub});
    
    for indxVar = 1:numVar
        plot(xList{indxSub}(indxSlice), yList{indxSub}{indxVar}(indxSlice)); hold on; grid on;
    end
    xlabel(xNames{indxSub}, 'Interpreter', 'none'); % FIXIT - allow optional
    ylabel(yNames{indxSub}, 'Interpreter', 'none'); % FIXIT - allow optional
    legend(legNames{indxSub}, 'Interpreter', 'none'); % FIXIT - allow optional
end
linkaxes(findobj(gcf, 'Type', 'axes'), 'x');

subplot(numSub, 1, 1); title(titleStr, 'Interpreter', 'none');


%% Outputs

