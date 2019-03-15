function [vSound_fps, dens_spft3, pres_lbpft2, temp_R, grav_fps2] = StdAtmos(alt_ft)
% Compute standard atmosphere parameters (1976 model).
%
%Usage:  [vSound_fps, dens_spft3, pres_lbpft2, temp_R, grav_fps2] = StdAtmos(alt_ft);
%
%Inputs:
% alt_ft - geometric altitude (ft)
%
%Outputs:
% vSound_fps  - velocity of sound (ft/s)
% dens_spft3  - ambient air density (slugs/ft^3)
% pres_lbpft2 - ambient static pressure (lbf/ft^2)
% temp_R      - ambient air temperature (deg R)
% grav_fps2   - acceleration due to gravity (ft/s^2)
%
%Notes:
% 
%
%Dependency:
% StdAtmosData1976
%

%Version History: Version 1.1
% 03/07/2006  C. Regan     Initial Release, based on Robert Clarke's routine (v1.0)
% 11/07/2006  C. Regan     Added I/O argument checking (v1.1)
%


%% Check I/O Arguments
error(nargchk(1, 1, nargin, 'struct'))
error(nargoutchk(0, 5, nargout, 'struct'))


%% Default Values and Constants
% Sandard sea level conditions
vSoundSL_fps = 1116.45; % speed of sound at SL (ft/s)
densSL_spft3 = 2.3769e-3; % density at SL (slug/ft3)
gravSL_fps2 = 32.1742; % gravity at SL (ft/s2)
presSL_lbpft2 = 2116.22; % pressure at SL (lbf/ft2)


%% Check Inputs


%% Return lookup data for 1976 US Standard Atmosphere
[altTbl_ft, vSoundRatioTbl, densRatioTbl, gravRatioTbl, presRatioTbl, tempTbl_R] = StdAtmosData1976();


%% Lookup data at specified altitude
% Limit altitude
if alt_ft > max(altTbl_ft)
    alt_ft = max(altTbl_ft);
elseif alt_ft < min(altTbl_ft)
    alt_ft = min(altTbl_ft);
end

vSoundRatio = interp1(altTbl_ft, vSoundRatioTbl, alt_ft, 'linear');
densRatio   = interp1(altTbl_ft, densRatioTbl  , alt_ft, 'linear');
gravRatio   = interp1(altTbl_ft, gravRatioTbl  , alt_ft, 'linear');
presRatio   = interp1(altTbl_ft, presRatioTbl  , alt_ft, 'linear');


%% Compute ambient conditions at altitude
vSound_fps = vSoundRatio * vSoundSL_fps;
dens_spft3 = densRatio * densSL_spft3;
grav_fps2 = gravRatio * gravSL_fps2;
pres_lbpft2 = presRatio * presSL_lbpft2;

% Temperature table is a direct lookup
temp_R = interp1(altTbl_ft, tempTbl_R, alt_ft, 'linear');


%% Check Outputs
