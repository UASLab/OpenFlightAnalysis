function gain_dB = Mag2DB(gain_mag)
% OpenFlightAnalysis - Mag2DB
%   Convert gain data in magnitue into Decibels.
%
% Inputs:
%  gain_dB - gain (mag)
%
% Outputs:
%  gain_mag - gain (dB)
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
narginchk(1, 1)
nargoutchk(0, 1)

%% Convert gain magnitude to dB, with negative value protection
gain_magPos = gain_mag > 0;

gain_dB = zeros(size(gain_mag));
gain_dB(gain_magPos) = 20*log10(gain_mag(gain_magPos));
gain_dB(~gain_magPos) = -Inf;
