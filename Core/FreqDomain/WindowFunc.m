function [Window] = WindowFunc(Opt)
% Apply any of several windows to data. 
%
%Inputs:
% Opt - Structured Window function options
%   Length  - Length of window
%   Type - Type of window ['rect']
%   [options]
%
%Outputs:
% Window - Window data
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
if ~isfield(Opt, 'Length')
    error([mfilename ' - Length must be specified.']);
end
if ~isfield(Opt, 'Type')
    Opt.Type = [];
end

if isempty(Opt.Type), Opt.Type = 'Rect'; end


%% Windowing Options
switch lower(Opt.Type)
    case {'rectwin', 'rect'} % Rectangular
        Window = ones(1, Opt.Length);
        
    case {'cosi', 'cosine', 'tukey', 'tukeywin'} % Cosine taper
        % Taper ratio determines the length of the cosine sections
        if ~isfield(Opt, 'TaperRatio')
            error([mfilename ' - TaperRatio must be specified.']);
        end
            
        if Opt.TaperRatio < 2/Opt.Length
            Opt.TaperRatio = 2/Opt.Length;
            warning([mfilename ' - TaperRatio must be in the range (0 0.5]']); % FIXME: message ID
        elseif Opt.TaperRatio > 0.5
            Opt.TaperRatio = 0.5;
            warning([mfilename ' - TaperRatio must be in the range (0 0.5]']); % FIXME: message ID
        end

        % Taper length
        lenTaper = round(Opt.TaperRatio * Opt.Length);

        % Compute the window weight vector
        Window = ones(1, Opt.Length);
        Window(1:lenTaper) = 0.5 * (1 - cos(pi*(0:lenTaper-1)/(lenTaper-1)));
        Window(end-(lenTaper-1):end) = 0.5 * (1 - cos(pi*(lenTaper-1:-1:0)/(lenTaper-1)));
        
    otherwise % Polynomial Based Windows
        switch lower(Opt.Type)
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
                error([mfilename ' - Unknown window Type: ''' Opt.Type ''''])
        end
        
        temp = (0:Opt.Length-1)/(Opt.Length-1);
        Window = c0 - c1*cos(2*pi*temp) + c2*cos(4*pi*temp) - c3*cos(6*pi*temp) + c4*cos(8*pi*temp);
end

