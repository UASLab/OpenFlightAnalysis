function [level1Bound, level2Bound, level1PIOBound, level2PIOBound] = SmithGeddesCrit(category)
% Smith/Geddes rating level and boundaries.
%
%Usage:  [level1Bound, level2Bound, level1PIOBound, level2PIOBound] = SmithGeddesCrit(category);
%
%Inputs:
% category - requirement category ['normal']
%
%Outputs:
% level1Bound    - level 1 boundary
% level2Bound    - level 2 boundary
% level1PIOBound - PIO type I boundary
% level2PIOBound - PIO type III boundary
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(0, 1, nargin, 'struct'))
if nargin < 1
    category = [];
end

error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants
if isempty(category), category = 'normal'; end


%% Check Inputs


%% Level Boudaries, (x,y) coordinates, counterclockwise
% TODO: Check bounary values
% Level 2 boundaries
level2Bound(:,1) = [-148 -148 -165 -165];
level2Bound(:,2) = [-380 -220 -220 -90];

% Level 1 boundaries
level1Bound(:,1) = [-123 -123 -130 -130];
level1Bound(:,2) = [-380 -160 -160 -90];

switch category
    case 'normal'
        % Type III PIO boundaries
        level2PIOBound(:,1) = [-180 -180];
        level2PIOBound(:,2) = [-380 -90];
        
        % Type I PIO boundaries
        level1PIOBound(:,1) = [-165 -165 -220];
        level1PIOBound(:,2) = [-380 -180 -180];
        
    case 'A'
        % Type III PIO boundaries
        level2PIOBound(:,1) = [-181 -181];
        level2PIOBound(:,2) = [-380 -90];
        
        % Type I PIO boundaries
        level1PIOBound(:,1) = [-166 -166 -220];
        level1PIOBound(:,2) = [-380 -182 -182];
        
    case 'B'
        % Type III PIO boundaries
        level2PIOBound(:,1) = [-182 -182];
        level2PIOBound(:,2) = [-380 -90];
        
        % Type I PIO boundaries
        level1PIOBound(:,1) = [-167 -167 -220];
        level1PIOBound(:,2) = [-380 -184 -184];

    otherwise
        warning('Unkown category'); % FIXME: warning ID
end


%% Check Outputs
