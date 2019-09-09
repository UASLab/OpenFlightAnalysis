function gain_mag = DB2Mag(gain_dB)
% OpenFlightAnalysis - DB2Mag
%   Convert gain data in Decibels into magnitude.
%
% Inputs:
%  gain_dB - gain (dB)
%
% Outputs:
%  gain_mag - gain (mag)
%
% Notes:
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 05/02/2017  C. Regan     Initial Release
%

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(0, 1);

%% Convert gain dB to magnitude
gain_mag = power(10, gain_dB/20);
