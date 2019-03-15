function [hrs, min, sec, msec] = Sec2Hms(timeSec)
% Convert time in seconds after midnight to hours, minutes, seconds, and milliseconds.
%
%Usage:  [hrs, min, sec, msec] = Sec2Hms(timeSec);
%
%Inputs:
% timeSec - seconds after midnight
%
%Outputs:
% hrs  - hours after midnight (see Note)
% min  - minutes after the hour
% sec  - seconds after the minute
% msec - milliseconds after the second
%
%Notes:
% if only one output is used then:
%   'hrs' is actually a string of the form:
%     hour:min:sec.msec ('13:09:45.456')
%

%Version History: Version 1.1
% 04/27/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(1, 1, nargin, 'struct'))
error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


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
