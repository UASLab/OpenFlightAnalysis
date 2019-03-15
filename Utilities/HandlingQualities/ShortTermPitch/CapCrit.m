function [cap, level, level1Bound, level2Bound, level3Bound] = CapCrit(freqSP_rps, dampSP, nzAlpha_gpr, category)
% Control anticipation parameter criterion rating level and boundaries.
%
%Usage:  [cap, level, level1Bound, level2Bound, level3Bound] = CapCrit(freqSP_rps, dampSP, nzAlpha_gpr, category)
%
%Inputs:
% freqSP_rps  - short-period frequency (rad/sec)
% dampSP      - short-period damping
% nzAlpha_gpr - normal load per angle of attack (g/rad)
% category    - flight phase category designation
%          
%Outputs:
% cap         - Control Anticipation Parameter (1/(g*sec*sec))
% level       - level rating
% level1Bound - level 1 bound
% level2Bound - level 2 bound
% level3Bound - level 3 bound
%
%Notes:
% 
%
%Reference:
% MIL-STD-1797A, Appendix A, Paragraph 4.2.1.2, Section A
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(4, 4, nargin, 'struct'))
error(nargoutchk(2, 5, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freqSP_rps = freqSP_rps(:);
dampSP = dampSP(:);
nzAlpha_gpr = nzAlpha_gpr(:);


%% Calculate the Control Anticipation Factor
cap = freqSP_rps .* freqSP_rps ./ nzAlpha_gpr;


%% Level boudaries, (x,y) coordinates, counterclockwise loop
% TODO: Check these values
switch category
    case 'A' % CategoryA
        % Level 1 boundaries
        level1Bound(:,1) = [0.35, 1.3, 1.3, 0.35, 0.35];
        level1Bound(:,2) = [0.28, 0.28, 3.6, 3.6, 0.28];
        
        % Level 2 boundaries
        level2Bound(:,1) = [0.25, 2, 2, 0.25, 0.25];
        level2Bound(:,2) = [0.16, 0.16, 10, 10, 0.16];
        
        % Level 3 boundaries, may be relaxed
        level3Bound(:,1) = [0.15, 0.15];
        level3Bound(:,2) = [0.01, 10]; %infinite
        
    case 'B' % CategoryB
        % Level 1 boundaries
        level1Bound(:,1) = [0.3, 2, 2, 0.3, 0.3];
        level1Bound(:,2) = [0.085, 0.085, 3.6, 3.6, 0.085];
        
        % Level 2 boundaries
        level2Bound(:,1) = [0.2, 2, 2, 0.2, 0.2];
        level2Bound(:,2) = [0.038, 0.038, 10, 10, 0.038];
        
        % Level 3 boundaries
        level3Bound(:,1) = [0.1, 0.1];
        level3Bound(:,2) = [0.01, 10]; %infinite
        
    case 'C' % Category C
        % Level 1 boundaries
        level1Bound(:,1) = [0.35, 1.3, 1.3, 0.35, 0.35];
        level1Bound(:,2) = [0.16, 0.16, 3.6, 3.6, 0.16];
        
        % Level 2 boundaries
        level2Bound(:,1) = [0.25, 2, 2, 0.25, 0.25];
        level2Bound(:,2) = [0.05, 0.05, 10, 10, 0.05];
        
        % Level 3 boundaries, may be relaxed
        level3Bound(:,1) = [0.15, 0.15];
        level3Bound(:,2) = [0.01, 10]; % infinite
        
    otherwise
        warning('Unkown category'); % FIXME: warning ID missing
end


%% Deterine the level achieved
% FIXME: This could be improved to be more general
level = zeros(length(cap),1);

% Test for Level 3
level(dampSP > min(level3Bound(:,1))) = 3;

% Test for Level 2
level(...
    (dampSP > min(level2Bound(:,1))) & (dampSP < max(level2Bound(:,1))) & ...
    (cap > min(level2Bound(:,2))) & (cap < max(level2Bound(:,2)))...
    ) = 2;
    
% Test for Level 1
level(...
    (dampSP > min(level1Bound(:,1))) & (dampSP < max(level1Bound(:,1))) & ...
    (cap > min(level1Bound(:,2))) & (cap < max(level1Bound(:,2)))...
    ) = 1;


%% Check Outputs
