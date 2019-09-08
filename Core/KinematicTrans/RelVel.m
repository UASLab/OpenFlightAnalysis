function [vel_BO] = RelVel(vel_AO, omega_AO, pos_BA, vel_BA)
% OpenFlightAnalysis - RelVel
%  Compute the relative velocity.
%
% Inputs:
%  vel_AO   - Velocity of frame A wrt frame O
%  omega_AO - Rotation rate of frame A wrt O
%  pos_BA   - Position of frame B wrt A
%  vel_BA   - Velocity of frame B wrt A [0; 0; 0]
%
% Outputs:
%  vel_BO   - Velocity of frame B wrt frame O
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
narginchk(3, 4);
if nargin < 4
    vel_BA = [];
end

% Default Values
if isempty(vel_BA), vel_BA = [0; 0; 0]; end

% Check the number of outputs
nargoutchk(0, 1);


%% Constants


%% Compute the Relative Velocity
vel_BO = vel_AO + vel_BA + cross(omega_AO, pos_BA);


%% Outputs
