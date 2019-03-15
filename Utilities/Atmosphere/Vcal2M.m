function [mach] = Vcal2M(vCal_kts, alt_ft)
% Convert calibrated airspeed to Mach number at a given altitude
%
%Usage:  [mach] = Vcal2M(vCal_kts, alt_ft);
%
%Inputs:
% vCal_kts - calibrated airspeed (knots)
% alt_ft   - pressure altitude (ft)
%
%Outputs:
% mach - mach number
%
%Notes:
% 
%
%Dependency:
% StdAtmos
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(2, 2, nargin, 'struct'))
error(nargoutchk(0, 1, nargout, 'struct'))


%% Default Values and Constants
kt2fps = 1.6878099;
gamma = 1.4;


%% Check Inputs


%% Lookup Pressure and Density at SL and altitude
[vSoundSL_fps, densSL_spft3, presSL_lbpft2] = StdAtmos(0);
[vSound_fps, dens_spft3, pres_lbpft2] = StdAtmos(alt_ft);


%% Compute Mach Number
% Convert to ft/s
vCal_fps = vCal_kts * kt2fps;

% Total Pressure
presT_lbpft2 = presSL_lbpft2*(((densSL_spft3*(gamma-1)*vCal_fps^2)/(2*gamma*presSL_lbpft2) + 1)^(gamma/(gamma-1)) - 1) + pres_lbpft2;

% Mach number
mach = sqrt((2/(gamma-1))*((presT_lbpft2/pres_lbpft2)^((gamma-1)/gamma) - 1));


%% Check Outputs
