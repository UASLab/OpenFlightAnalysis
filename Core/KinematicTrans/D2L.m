function [r_PL_L_m] = D2L(rRef_LD_D_ddm, r_PD_D_ddm)
% OpenFlightAnalysis - D2L
%  Convert Geodetic Coordinates to Local Level
%
% %Inputs:
% rRef_LD_D_ddm - Reference Position in Geodetic Coordinates (deg, deg, m)
% r_PD_D_ddm    - Position in Geodetic Coordinates (deg, deg, m)
%
%Outputs:
% r_PL_L_m - Position in Local Level Coordinates (m)
%
% Notes:
%  r_PD_D_ddm can be r_PE_D_ddm, r_PI_D_ddm
%  Uses D2E and E2L
%  Fixit - this is inefficient!
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
narginchk(2, 2);

% Default Values

% Check the number of outputs
nargoutchk(0, 1);


%% Conversion
[r_PE_E_m] = D2E(r_PD_D_ddm);
[r_LD_E_m] = D2E(rRef_LD_D_ddm); % Reference location of L wrt D in ECEF
r_PL_E_m = r_PE_E_m - r_LD_E_m; % Distance Moved in ECEF
[~, r_PL_L_m] = E2L(rRef_LD_D_ddm, r_PL_E_m);
