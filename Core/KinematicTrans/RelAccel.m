function [accel_BO] = RelAccel(accel_AO, omega_AO, omegaDot_AO, pos_BA, accel_BA, vel_BA)
% OpenFlightAnalysis - RelVel
%  Compute the relative acceleration.
%
% Inputs:
%  accel_AO    - Acceleration of frame A wrt frame O
%  omega_AO    - Rotation rate of frame A wrt O
%  omegaDot_AO - Rotation rate of frame A wrt O
%  pos_BA      - Position of frame B wrt A
%  accel_BA    - Acceleration of frame B wrt A [0; 0; 0]
%  vel_BA      - Velocity of frame B wrt A [0; 0; 0]
%
% Outputs:
%  accel_BO   - Acceleration of frame B wrt frame O
%
% Notes:
%  There are no internal unit conversions.
%  The vectors must all be in the same coordinate system
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 09/28/2016  C. Regan     Initial Release
% 10/17/2016  C. Regan     Changed to simple and faster input parsing
%


%% Check I/O Arguments
% Check the number of inputs
narginchk(4, 6);
if nargin < 6, vel_BA = [];
    if nargin < 5, accel_BA = []; end
end

% Default Values
if isempty(accel_BA), accel_BA = [0; 0; 0]; end
if isempty(vel_BA), vel_BA = [0; 0; 0]; end

% Check the number of outputs
nargoutchk(0, 1);


%% Compute the Relative Velocity
accel_BO = accel_AO + accel_BA + cross(omegaDot_AO, pos_BA) + cross(omega_AO, cross(omega_AO, pos_BA)) + 2 * cross(omega_AO, vel_BA);


%% Outputs
