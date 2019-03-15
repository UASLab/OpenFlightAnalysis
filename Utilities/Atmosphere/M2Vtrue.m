function vTrue_fps = M2Vtrue(mach, alt_ft)
% Convert Mach number to true airspeed at a given altitude.
%
%Usage:  [vTrue_fps] = M2Vtrue(mach, alt_ft);
%
%Inputs:
% mach   - Mach number
% alt_ft - pressure altitude (ft)
%
%Outputs:
% vTrue_fps - true airspeed (ft/sec)
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


%% Check Inputs


%% Return the speed of sound at altitude
[vSound_fps] = StdAtmos(alt_ft);


%% Convert mach number to true airspeed
vTrue_fps = vSound_fps * mach;


%% Check Outputs
