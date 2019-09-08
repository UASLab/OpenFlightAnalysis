function [T_E2L, r_L_m] = E2L(rRef_D_ddm, r_E_m)
% OpenFlightAnalysis - E2L
%  Convert ECEF Coordinates to Local Level
%
% Inputs:
%  rRef_D_ddm - Reference Position in Geodetic Coordinates (deg, deg, m)
%  r_E_m      - Position in ECEF Coordinates (m) []
%
% Outputs:
%  T_E2L - Transformation Matrix from ECEF to Local Level Coordinates
%  r_L_m - Position in Local Level Coordinates (m)
%
% Notes:
%  T_E2L = R2(270-lat)*R3(long)
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
narginchk(1, 2);
if nargin < 2
    r_E_m = [];
end

% Default Values
if isempty(r_E_m), r_E_m = []; end

% Check the number of outputs
nargoutchk(0, 2);

%% Constants


%% Transfer the Coordinates
% Compute the Transformation Matrix at the Reference Location
angleList = [(270 - rRef_D_ddm(1)), rRef_D_ddm(2)];
[T_E2L] = RotMat('32', angleList, 'deg');

% Transform Coordinates from ECEF to NED
if (nargout == 2) && ~isempty(r_E_m)
    r_L_m = T_E2L * r_E_m;
end
