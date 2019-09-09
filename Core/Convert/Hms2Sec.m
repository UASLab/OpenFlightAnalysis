function [timeSec] = Hms2Sec(hrs, min, sec, msec)
% OpenFlightAnalysis - Hms2Sec
%   Convert hours, minutes, seconds, and milliseconds to seconds after midnight.
%
% Inputs:
%  hrs  - hours after midnight (see Note)
%  min  - minutes after the hour
%  sec  - seconds after the minute
%  msec - milliseconds after the second
%
% Outputs:
%  timeSec - seconds after midnight
%
% Notes:
% if only one input is used then:
%   'hrs' is actually a string of the form:
%     day:hour:min:sec.msec ('040:13:09:45.456') or
%     hour:min:sec.msec ('13:09:45.456')
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 05/02/2017  C. Regan     Initial Release
%

%% Check I/O Arguments
narginchk(1, 4)
if nargin < 4, msec = [];
    if nargin < 3, sec = []; end
    if nargin < 2, min = []; end
end

nargoutchk(0, 1)

%% Default Values and Constants
if isempty(msec), msec = 0; end
if isempty(sec), sec = 0; end
if isempty(min), min = 0; end

%% Check Inputs
% If only 'hrs' is provided, parse into seperate strings (see Note)
if nargin == 1
    timeStr = hrs;

    indxCol = findstr(timeStr, ':');
    indxDec = findstr(timeStr, '.');

    switch length(indxCol)
        case 2
            hrs = str2num(timeStr(1:indxCol(1)-1));
            min = str2num(timeStr(indxCol(1)+1:indxCol(2)-1));
            sec = str2num(timeStr(indxCol(2)+1:indxDec(1)-1));
            msec = str2num(timeStr(indxDec(1)+1:end));
        case 3
            day = str2num(timeStr(1:indxCol(1)-1));
            hrs = str2num(timeStr(indxCol(1)+1:indxCol(2)-1));
            min = str2num(timeStr(indxCol(2)+1:indxCol(3)-1));
            sec = str2num(timeStr(indxCol(3)+1:indxDec(1)-1));
            msec = str2num(timeStr(indxDec(1)+1:end));
        otherwise
            warning('unkown time format');
    end
end


%% Compute the time after midnight
timeSec = hrs*60*60 + min*60 + sec + msec/1000;
