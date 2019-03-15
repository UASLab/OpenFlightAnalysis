function [T_LE, r_PL_L_m] = E2L(rRef_LD_D_ddm, r_PL_E_m)
% Convert ECEF Coordinates to Local Level
%
% %Inputs:
% rRef_LD_D_ddm - Reference Position in Geodetic Coordinates (ddm)
% r_PE_E_m      - Position in ECEF Coordinates (m)
%
%Outputs:
% T_LE - Transformation Matrix from ECEF to Local Level Coordinates
% r_L_m - Position in Local Level Coordinates (m)
%
%Notes:
% T_LE = R2(270-lat)*R3(long)

%Version History: Version 1.1
% 10/07/2016  C. Regan     Initial Release (v1.0)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.1)
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(1, 2);
if nargin < 2
    r_PL_E_m = r_PL_E_m;
end

% Default Values
if isempty(r_PL_E_m), r_PL_E_m = []; end

% Check the number of outputs
nargoutchk(0, 2);

%% Constants


%% Transfor the Coordinates
% Compute the Transformation Matrix at the Reference Location
angleList = [(270 - rRef_LD_D_ddm(1)), rRef_LD_D_ddm(2)];
[T_LE] = RotMat('32', angleList, 'deg');

% Transform Coordinates from ECEF to NED
if (nargout == 2) && ~isempty(r_PL_E_m)
    r_PL_L_m = T_LE * r_PL_E_m;
end
