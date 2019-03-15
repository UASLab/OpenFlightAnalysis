function [xWin] = WindowSignal(x, winType, varargin)
% Apply any of several windows to data. 
%
%Usage:  [xWin] = WindowSignal(x, winType, varargin);
%
%Inputs:
% x        - data to be windowed
% winType  - window type ['rectwin']
% varargin - additional options for 'window' function
%
%Outputs:
% xWin - windowed version of the input data
%
%Notes:
% Usually for use with fast fourier transforms.
%
%TODO: This function is a mess, consider reworking.

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 3)
if nargin < 2, winType = [];
    if nargin < 3, varargin = []; end
end

nargoutchk(0, 1)


%% Default Values and Constants
if isempty(winType), winType = 'rectwin'; end


%% Check Inputs
% size of input vector
[widthX, lenX] = size(x);

% Transpose
if lenX < widthX
    transposeWinFlag = 1;
    [widthX, lenX] = size(x');
else
    transposeWinFlag = 0;
end
    

%% Windowing Options
switch winType
    case {'rectwin', 'rect'} % Rectangular
        w = ones(widthX, lenX);
        
    case {'cosi', 'cosine'} % Cosine taper
        % Taper ratio determines the length of the cosine sections
        if nargin > 2
            taperRatio = varargin{1};
            if taperRatio < 2/lenX
                taperRatio = 2/lenX;
                warning([mfilename ' - Taper ratio for ''' winType ''' must be in the range (0 0.5]']); % FIXME: message ID
            elseif taperRatio > 0.5
                taperRatio = 0.5;
                warning([mfilename ' - Taper ratio for ''' winType ''' must be in the range (0 0.5]']); % FIXME: message ID
            end
        else
            taperRatio = [];
        end

        % Set default taper ratio value
        if isempty(taperRatio), taperRatio = 0.1; end

        % Taper length
        lenTaper = round(taperRatio * lenX);

        % Compute the window weight vector
        w = ones(widthX, lenX);
        w(1:lenTaper) = 0.5 * (1 - cos(pi*(0:lenTaper-1)/(lenTaper-1)));
        w(end-lenTaper:end) = 0.5 * (1 - cos(pi*(lenTaper:-1:0)/(lenTaper)));
        
    otherwise % Polynomial Based Windows
        switch winType
            case {'hann', 'hanning', 'han'} % Hann
                c0 = 0.5;
                c1 = 0.5;
                c2 = 0;
                c3 = 0;
                c4 = 0;
            case {'hamming', 'hamm', 'ham'} % Hamming
                c0 = 0.54;
                c1 = 0.46;
                c2 = 0;
                c3 = 0;
                c4 = 0;
            case {'blackman', 'black'} % Blackman
                c0 = 0.42;
                c1 = 0.5;
                c2 = 0.08;
                c3 = 0;
                c4 = 0;
            otherwise
                error([mfilename ' - Unkown window type: ''' winType ''''])
        end
        
        temp = (0:lenX-1)/(lenX-1);
        w = c0 - c1*cos(2*pi*temp) + c2*cos(4*pi*temp) - c3*cos(6*pi*temp) + c4*cos(8*pi*temp);
end

% Transpose window data
if transposeWinFlag == 1
    w = w';
end

%% Apply the window to the data
xWin = x .* w;


%% Check Outputs
