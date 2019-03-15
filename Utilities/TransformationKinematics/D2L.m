function [r_PL_L_m] = D2L(rRef_LD_D_ddm, r_PD_D_m)
% Convert ECEF Coordinates to Local Level
%
% %Inputs:
% rRef_LD_D_ddm - Reference Position in Geodetic Coordinates (ddm)
% r_PD_D_m      - Position in Geodetic Coordinates (ddm)
%
%Outputs:
% r_PL_L_m - Position in Local Level Coordinates (m)
%
%Notes:
% Uses D2E and E2L
% Fixit - this is inefficient!

%Version History: Version 1.1
% 10/07/2016  C. Regan     Initial Release (v1.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.1)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 2);

% Default Values
    
% Check the number of outputs
nargoutchk(0, 1);


%% Conversion
[r_PE_E_m] = D2E(r_PD_D_m);
[r_LD_E_m] = D2E(rRef_LD_D_ddm); % Reference location of L wrt D in ECEF
r_PL_E_m = r_PE_E_m - r_LD_E_m; % Distance Moved in ECEF
[~, r_PL_L_m] = E2L(rRef_LD_D_ddm, r_PL_E_m);


