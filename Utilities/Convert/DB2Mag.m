function gain_mag = DB2Mag(gain_dB)
% Convert gain data in Decibels into magnitude.
%
%Usage:  gain_mag = DB2Mag(gain_dB);
%
%Inputs:
% gain_dB - gain (dB)
%
%Outputs:
% gain_mag - gain (mag)
%
%Notes:
% 
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release
%

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(0, 1);


%% Default Values and Constants


%% Check Inputs


%% Convert gain dB to magnitude
gain_mag = power(10, gain_dB/20);


%% Check Outputs
