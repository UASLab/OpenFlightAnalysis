function [level1Bound, level2Bound, level] = NealSmithCrit(category, pilotLead_deg, resPeak_dB)
% Neal/Smith rating level and boundaries.
%
%Usage:  [level2Bound, level1Bound, level] = NealSmithCrit(category, pilotLead_deg, resPeak_dB);
%
%Inputs:
% category      - requirement category 'new' or 'old'  ['new']
% pilotLead_deg - required pilot lead (deg)
% resPeak_dB    - resonance peak resulting (dB)
%
%Outputs:
% level1Bound - level 1 bound
% level2Bound - level 2 bound
% level       - level rating
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 3, nargin, 'struct'))
if nargin < 3, resPeak_dB = [];
    if nargin < 2, pilotLead_deg = []; end
    if nargin < 1, category = []; end
end

error(nargoutchk(0, 3, nargout, 'struct'))


%% Default Values and Constants
if isempty(category), category = 'new'; end


%% Check Inputs
% Inputs as column vectors
pilotLead_deg = pilotLead_deg(:);
resPeak_dB = resPeak_dB(:);


%% Level boudaries, (x,y) coordinates, counterclockwise
% TODO: Check bounary values
switch category
    case 'new'
        % Level 2 boundaries
        level2Bound(:,1) = [90, -30];
        level2Bound(:,2) = [5.4375, 12.1875];

        % Level 1 boundaries
        level1Bound(:,1) = [90, 45, 30, -15, -30];
        level1Bound(:,2) = [-17, 2, 3, 3, -5];
        
    case 'old'
        % Level 2 boundaries
        level2Bound(:,1) = [90, 0, -30];
        level2Bound(:,2) = [6.5, 11, 11];

        % Level 1 boundaries
        level1Bound(:,1) = [90, 42.5, 15, -15, -30];
        level1Bound(:,2) = [-18.8889, 7/6, 3, 3, -5];

    otherwise
        warning('Unkown category'); % FIXME: warning ID
end


%% Deterine the level achieved
level = zeros(length(pilotLead_deg), 1); % FIXME: zeroed for lack of solution


%% Check Outputs
