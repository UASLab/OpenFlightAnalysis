function [levelCH, levelCHLow, levelCHUp, aveCH] = SmithGeddesACH(phasePilotBW_deg)
% Average Cooper-Harper Rating Levels based on the Smith/Geddes criterion.
%
%Usage:  [levelCH, levelCHLow, levelCHUp, aveCH] = SmithGeddesACH(phasePilotBW_deg);
%
%Inputs:
% phasePilotBW_deg - phase angle at pilot bandwidth frequency (deg)
%
%Outputs:
% levelCH    - average Cooper-Harper level line
% levelCHLow - lower Cooper-Harper level line
% levelCHUp  - upper Cooper-Harper level line
% aveCH      - average Cooper-Harper Rating
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(0, 1, nargin, 'struct'))
if nargin < 1
    phasePilotBW_deg = [];
end

error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs


%% Levels, (x,y) coordinates
% Average level line
levelCH(:,1) = [-80, -140, -180, -240];
levelCH(:,2) = [2.5, 3.7, 8, 9.2];

% Lower level line
levelCHLow(:,1) = [-90, -145, -190, -240];
levelCHLow(:,2) = [2, 3, 8, 8];

% Upper level line
levelCHUp(:,1) = [-80, -135, -175, -230];
levelCHUp(:,2) = [3, 4.2, 8.5, 10];


%% Calculate the Average Cooper-Harper
if ~isempty(phasePilotBW_deg)
    if ((phasePilotBW_deg > -240) && (phasePilotBW_deg < -80))
        aveCH = interp1(levelCH(:,1), levelCH(:,2), phasePilotBW_deg);
    else
        aveCH = NaN;
    end
else
    aveCH = [];
end


%% Check Outputs
