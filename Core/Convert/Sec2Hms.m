function [hrs, min, sec, msec] = Sec2Hms(timeSec)
% OpenFlightAnalysis - Hms2Sec
%  Convert time in seconds after midnight to hours, minutes, seconds, and milliseconds.
%
% Inputs:
%  timeSec - seconds after midnight
%
% Outputs:
%  hrs  - hours after midnight (see Note)
%  min  - minutes after the hour
%  sec  - seconds after the minute
%  msec - milliseconds after the second
%
% Notes:
%  if only one output is used then:
%    'hrs' is actually a string of the form:
%      hour:min:sec.msec ('13:09:45.456')
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 05/02/2017  C. Regan     Initial Release
%

%% Check I/O Arguments
narginchk(1, 1)
nargoutchk(0, 4)

%% Calculate the hour, minute, second, and milisecond
hrs = floor(timeSec/3600);
min = floor(timeSec/60 - 60*hrs);
sec = floor(timeSec - 3600*hrs - 60*min);
msec = round((timeSec - 3600*hrs - 60*min - sec)*1000);


%% Check Outputs
% If only one output is requested then form 'hrs' as a string (see Note)
if nargout <= 1
    hrs = sprintf('%0.2d:%0.2d:%0.2d.%0.3d', hrs, min, sec, msec);
end
