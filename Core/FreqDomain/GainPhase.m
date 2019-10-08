function [gain_dB, phase_deg] = GainPhase(xyT)
% Compute the gain and phase from transfer function data.
%
%Usage:  [gain_dB, phase_deg] = GainPhase(xyT);
%
%Inputs:
% xyT - complex transfer function
%
%Outputs:
% gain_dB   - magnitude of transfer function (db)
% phase_deg - phase angle of transfer function (deg)
%
%Notes:
%

% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details
% Author: Chris Regan
%

%% Check I/O Arguments
narginchk(1, 1);
nargoutchk(0, 2);


%% Default Values and Constants
% Constants
r2d = 180/pi;


%% Compute magnitude and phase from complex transfer function
gain_dB = Mag2DB(abs(xyT));
phase_deg = angle(xyT) * r2d;
%phase_deg = atan2(imag(xyT), real(xyT)) * r2d; % Equivalent
%phase_deg = imag(log(xyT)) * r2d; % Equivalent but slower

