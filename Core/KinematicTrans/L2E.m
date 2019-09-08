function [T_LE, r_PL_E_m] = L2E(rRef_LD_D_ddm, r_PL_L_m)
% OpenFlightAnalysis - L2E
%  Convert Local Level Coordinates to ECEF
%
% Inputs:
%  rRef_LD_D_ddm - Reference Position in Geodetic Coordinates (deg, deg, m)
%  r_PL_L_m      - Position in Local Level Coordinates (m)
%
% Outputs:
%  T_EL - Transformation Matrix from ECEF to Local Level Coordinates
%  r_PL_E_m - Position in ECEF Coordinates (m)
%
% Notes:
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
%


%% Check I/O Arguments
% Check the number of inputs


% Default Values

% Check the number of outputs

%% Constants


%% Compute the Relative Velocity
vel_BO = vel_AO + vel_BA + cross(omega_AO, pos_BA);


%% Outputs
