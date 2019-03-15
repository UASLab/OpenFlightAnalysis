function vCal_kts = M2Vcal(mach, alt_ft)
% Convert Mach number to calibrated airspeed at a given altitude.
%
%Usage:  [vCal_kts] = M2Vcas(mach, alt_ft);
%
%Inputs:
% mach    - mach number
% alt_ft  - pressure altitude (ft)
%
%Outputs:
% vCal_kts - calibrated airspeed (knots)
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


%% Compute Calibrated Airspeed
% Total Pressure
presT_lbpft2 = pres_lbpft2*((1 + ((gamma-1)/2)*mach^2)^(gamma/(gamma-1)));

% Calibrated Airspeed at altitude
vCal_fps = sqrt((2/(gamma-1))*(gamma*presSL_lbpft2/densSL_spft3)*(((presT_lbpft2-pres_lbpft2)/presSL_lbpft2 + 1)^((gamma-1)/gamma) - 1));

% Convert to knots
vCal_kts = vCal_fps / kt2fps;


%% Check Outputs
