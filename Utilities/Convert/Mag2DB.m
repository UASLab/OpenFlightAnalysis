function gain_dB = Mag2DB(gain_mag)
% Convert gain data in magnitude into Decibels.
%
%Usage:  gain_dB = Mag2DB(gain_mag);
%
%Inputs:
% gain_mag - gain (mag)
%
%Outputs:
% gain_dB - gain (dB)
%
%Notes:
% Slightly modified from Mathworks Control Toolbox
%

%Version History: Version 1.0
% 05/02/2017  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
narginchk(1, 1)
nargoutchk(0, 1)


%% Default Values and Constants


%% Check Inputs


%% Convert gain magnitude to dB, with negative value protection
gain_magPos = gain_mag > 0;

gain_dB = zeros(size(gain_mag));
gain_dB(gain_magPos) = 20*log10(gain_mag(gain_magPos));
gain_dB(~gain_magPos) = -Inf;


%% Check Outputs
