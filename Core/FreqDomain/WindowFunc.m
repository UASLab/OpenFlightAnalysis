function [win] = WindowFunc(optWin)
% Apply any of several windows to data. 
%
%Inputs:
% optWin - Structured Window function options
%   len  - Length of window
%   type - Type of window ['rect']
%   [options]
%
%Outputs:
% win - window data
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan

%% Check I/O Arguments
narginchk(1, 1)
nargoutchk(0, 1)

%% Check Inputs
if ~isfield(optWin, 'len')
    error([mfilename ' - len must be specified.']);
end
if ~isfield(optWin, 'type')
    optWin.type = [];
end

if isempty(optWin.type), optWin.type = 'rect'; end


%% Windowing Options
switch lower(optWin.type)
    case {'rectwin', 'rect'} % Rectangular
        win = ones(1, optWin.len);
        
    case {'cosi', 'cosine', 'tukey', 'tukeywin'} % Cosine taper
        % Taper ratio determines the length of the cosine sections
        if ~isfield(optWin, 'taperRatio')
            error([mfilename ' - taperRatio must be specified.']);
        end
            
        if optWin.taperRatio < 2/optWin.len
            optWin.taperRatio = 2/optWin.len;
            warning([mfilename ' - taperRatio must be in the range (0 0.5]']); % FIXME: message ID
        elseif optWin.taperRatio > 0.5
            optWin.taperRatio = 0.5;
            warning([mfilename ' - taperRatio must be in the range (0 0.5]']); % FIXME: message ID
        end

        % Taper length
        lenTaper = round(optWin.taperRatio * optWin.len);

        % Compute the window weight vector
        win = ones(1, optWin.len);
        win(1:lenTaper) = 0.5 * (1 - cos(pi*(0:lenTaper-1)/(lenTaper-1)));
        win(end-(lenTaper-1):end) = 0.5 * (1 - cos(pi*(lenTaper-1:-1:0)/(lenTaper-1)));
        
    otherwise % Polynomial Based Windows
        switch lower(optWin.type)
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
                error([mfilename ' - Unkown window type: ''' optWin ''''])
        end
        
        temp = (0:optWin.len-1)/(optWin.len-1);
        win = c0 - c1*cos(2*pi*temp) + c2*cos(4*pi*temp) - c3*cos(6*pi*temp) + c4*cos(8*pi*temp);
end

