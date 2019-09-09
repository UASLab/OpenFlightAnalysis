function [r_E_m] = D2E(r_D_ddm)
% OpenFlightAnalysis - D2E
%  Convert Geodetic to ECEF Coordinates
%
% Inputs:
%  r_D_ddm - Position in Geodetic Coordinates (deg, deg, m)
%
% Outputs:
%  r_E_m - Position in ECEF Coordinates (m, m, m)
%
% Notes:
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 10/07/2016  C. Regan     Initial Release
% 10/17/2016  C. Regan     Changed to simple and faster input parsing
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 1);

% Default Values

% Check the number of outputs
nargoutchk(0, 1);


%% Constants
r2d = 180/pi;
d2r = 1/r2d;

%%
% Change units
r_D_rrm = [d2r; d2r; 1] .* r_D_ddm;

% Parameters for WGS84
R_m = 6378137.0; % Earth semimajor axis (m)
f_nd = 1/298.257223563; % reciprocal flattening (nd)

% Derived parameters
eSquared_nd = 2*f_nd - f_nd^2; % eccentricity squared
Rew = R_m ./ sqrt(1 - eSquared_nd * sin(r_D_rrm(1)).^2); % Radius East-West at Latitude

%% Convert
r_E_m = NaN(size(r_D_rrm));
r_E_m(1) = (Rew + r_D_rrm(3)).*cos(r_D_rrm(1)).*cos(r_D_rrm(2));
r_E_m(2) = (Rew + r_D_rrm(3)).*cos(r_D_rrm(1)).*sin(r_D_rrm(2));
r_E_m(3)= (Rew * (1-eSquared_nd) + r_D_rrm(3)).*sin(r_D_rrm(1));
